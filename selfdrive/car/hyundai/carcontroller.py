from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfahda_mfc, create_steer_command
from selfdrive.car.hyundai.values import Buttons, CarControllerParams, CAR, SteerLimitParams
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert

def calc_steering_torque_hold(angle, vEgo):
  hold_BP = [-40.0, -6.0, -4.0, -3.0, -2.0, -1.0, -0.5,  0.5,  1.0,  2.0,  3.0,  4.0,  6.0, 40.0]
  hold_V  = [-12.0, -5.7, -5.0, -4.5, -4.0, -3.3, -2.5,  2.5,  3.3,  4.0,  4.5,  5.0,  5.7, 12.0]
  return interp(angle, hold_BP, hold_V) #todo substract angle offset

SAMPLING_FREQ = 100 #Hz

# Steer angle limits
ANGLE_MAX_BP = [5., 15., 30]  #m/s
ANGLE_MAX = [200., 30., 15.] #deg
#ANGLE_MAX = [200., 20., 10.] #deg   (dzids original)
ANGLE_RATE_BP = [0., 5., 15.]
ANGLE_RATE_WINDUP = [500., 80., 15.]     #deg/s windup rate limit
ANGLE_RATE_UNWIND = [500., 350., 40.]  #deg/s unwind rate limit


def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):
  sys_warning = (visual_alert == VisualAlert.steerRequired)

  # initialize to no line visible
  sys_state = 1
  if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in [CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in [CAR.GENESIS_G90, CAR.GENESIS_G80] else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.p = CarControllerParams(CP)
    # self.packer = CANPacker(dbc_name)
    self.packer = CANPacker('hyundai_kia_generic')

    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.steer_rate_limited = False
    self.last_resume_frame = 0

    # StepperServo variables, redundant safety check with the board
    self.last_steer_tq = 0
    self.last_controls_enabled = False
    self.last_target_angle_lim = 0
    self.angle_control = False
    self.steer_angle_enabled = False
    self.last_fault_frame = -200
    self.planner_cnt = 0
    self.inertia_tq = 0.
    self.target_angle_delta = 0
    self.steer_tq_r = 0

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart):
    # Steering Torque
    new_steer = actuators.steer * self.p.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    self.steer_rate_limited = new_steer != apply_steer

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    lkas_active = enabled and abs(CS.out.steeringAngleDeg) < CS.CP.maxSteeringAngleDeg

    # fix for Genesis hard fault at low speed
    if CS.out.vEgo < 16.7 and self.car_fingerprint == CAR.HYUNDAI_GENESIS:
      lkas_active = False

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer



# ############################# New Steer Logik ####################################

    # Cut steering for 2s after fault
    steer_tq = 0
    if not enabled or (frame - self.last_fault_frame < 200):   # I don't think I have last_fault_frame, use old statement below
    #if not enabled or abs(CS.out.steeringRateDeg) > 100:
       apply_steer_req = 0
    else:
      apply_steer_req = 1

    # steer angle
    angle_lim = interp(CS.out.vEgo, ANGLE_MAX_BP, ANGLE_MAX)
    target_angle_lim = clip(actuators.steeringAngleDeg, -angle_lim, angle_lim)
      
    if enabled:
      # windup slower
      if (self.last_target_angle_lim * target_angle_lim) > 0. and abs(target_angle_lim) > abs(self.last_target_angle_lim): #todo revise last_angle
        angle_rate_max = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_WINDUP) 
      else:
        angle_rate_max = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_UNWIND)
      
      # steer angle - don't allow too large delta
      MAX_SEC_BEHIND = 1 #seconds behind target. Target deltas behind more than 1s will be rejected by bmw_safety #todo implement real (speed) rate limiter?? check with panda. Replace MAX_SEC_BEHIND with a Hz?
      target_angle_lim = clip(target_angle_lim, self.last_target_angle_lim - angle_rate_max*MAX_SEC_BEHIND, self.last_target_angle_lim + angle_rate_max*MAX_SEC_BEHIND)
      
      self.target_angle_delta =  target_angle_lim - CS.out.steeringAngleDeg
      angle_step_max = angle_rate_max / SAMPLING_FREQ  #max angle step per single sample
      angle_step = clip(self.target_angle_delta, -angle_step_max, angle_step_max) #apply angle step
      self.steer_rate_limited = self.target_angle_delta != angle_step #advertise steer beeing rate limited
      
      # steer torque
      I_steering = 0 #estimated moment of inertia
      
      PLANNER_SAMPLING_SUBRATE = 6 #planner updates target angle every 4 or 6 samples
      if target_angle_lim != self.last_target_angle_lim or self.planner_cnt >= PLANNER_SAMPLING_SUBRATE-1:
        steer_acc = (target_angle_lim - self.last_target_angle_lim) * SAMPLING_FREQ  #desired acceleration
        remaining_steer_torque = self.inertia_tq * (PLANNER_SAMPLING_SUBRATE - self.planner_cnt -1) #remaining torque to be applied if target_angle_lim was updated earlier than PLANNER_SAMPLING_SUBRATE
        self.inertia_tq = I_steering * steer_acc / PLANNER_SAMPLING_SUBRATE * CV.DEG_TO_RAD  #kg*m^2 * rad/s^2 = N*m (torque)
        self.inertia_tq += remaining_steer_torque / PLANNER_SAMPLING_SUBRATE
        self.planner_cnt = 0
      else:
        self.planner_cnt += 1
      
      # add feed-forward and inertia compensation
      feedforward = calc_steering_torque_hold(target_angle_lim, CS.out.vEgo)
      steer_tq = feedforward + actuators.steer + self.inertia_tq
      # explicitly clip torque before sending on CAN
      steer_tq = clip(steer_tq, -SteerLimitParams.MAX_STEERING_TQ, SteerLimitParams.MAX_STEERING_TQ)
      self.steer_tq_r = steer_tq * (-1)    # Switch StepperServo rotation
      #self.steer_tq_r = steer_tq * (1)    # Non-switch StepperServo rotation
      
      # can_sends.append(create_new_steer_command(self.packer, apply_steer_req, self.target_angle_delta, self.steer_tq_r, frame))
      # *** control msgs ***
      # if (frame % 10) == 0: #slow print
      #   print("SteerAngle {0} Inertia  {1} Brake {2}, frame {3}".format(target_angle_lim,
      #                                                            self.inertia_tq,
      #                                                            actuators.brake, speed_diff_req))
    elif not enabled and self.last_controls_enabled: #falling edge - send cancel CAN message
      self.target_angle_delta = 0
      steer_tq = 0
      self.steer_tq_r = 0
      can_sends.append(create_steer_command(self.packer, apply_steer_req, self.target_angle_delta, self.steer_tq_r, frame)) 
      
      # if (frame % 100) == 0: #slow print when disabled
      #   print("SteerAngle {0} SteerSpeed {1}".format(CS.out.steeringAngleDeg,
                                                                #  CS.out.steeringRateDeg))
#     self.last_target_angle_lim = target_angle_lim
  
      self.last_steer_tq = steer_tq
      self.last_target_angle_lim = target_angle_lim
      # self.last_accel = apply_accel
      # self.last_standstill = CS.out.standstill
      self.last_controls_enabled = enabled
  
  
# ########################################## End of new Steer Logik #################################################





    sys_warning, sys_state, left_lane_warning, right_lane_warning = \
      process_hud_alert(enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    can_sends = []
    # can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
    #                                CS.lkas11, sys_warning, sys_state, enabled,
    #                                left_lane, right_lane,
    #                                left_lane_warning, right_lane_warning))

    if pcm_cancel_cmd:
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL))
    elif CS.out.cruiseState.standstill:
      # send resume at a max freq of 10Hz
      if (frame - self.last_resume_frame)*DT_CTRL > 0.1:
        # send 25 messages at a time to increases the likelihood of resume being accepted
        can_sends.extend([create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL)] * 25)
        self.last_resume_frame = frame

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in [CAR.SONATA, CAR.PALISADE, CAR.IONIQ, CAR.KIA_NIRO_EV, CAR.IONIQ_EV_2020]:
      can_sends.append(create_lfahda_mfc(self.packer, enabled))

    return can_sends
