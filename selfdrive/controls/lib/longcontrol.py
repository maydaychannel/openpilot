from cereal import log
from common.numpy_fast import clip, interp
from selfdrive.controls.lib.pid import LongPIDController
from selfdrive.controls.lib.dynamic_gas import DynamicGas
from common.op_params import opParams

LongCtrlState = log.ControlsState.LongControlState

STOPPING_EGO_SPEED = 0.5    # Original value
#STOPPING_EGO_SPEED = 8.
STOPPING_TARGET_SPEED_OFFSET = 0.01
#STOPPING_TARGET_SPEED_OFFSET = 8.01
STARTING_TARGET_SPEED = 0.5
BRAKE_THRESHOLD_TO_PID = 0.2
GdMAX_V = [7, 14, 22, 28, 35]
#GdMAX_OUT = [0.002, 0.005, 0.01, 0.02, 0.05]
GdMAX_OUT = [0.002, 0.002, 0.005, 0.01, 0.02]
#GdMAX_OUT = [0.0005, 0.001, 0.003, 0.01, 0.02]

BRAKE_STOPPING_TARGET = 0.5  # apply at least this amount of brake to maintain the vehicle stationary

RATE = 100.0
def long_control_state_trans(active, long_control_state, v_ego, v_target, v_pid,
                             output_gb, brake_pressed, cruise_standstill, min_speed_can):
  """Update longitudinal control state machine"""
  stopping_target_speed = min_speed_can + STOPPING_TARGET_SPEED_OFFSET
  starting_condition = v_target > STARTING_TARGET_SPEED and not cruise_standstill

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if active:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.starting

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif output_gb >= -BRAKE_THRESHOLD_TO_PID:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl():
  def __init__(self, CP, compute_gb, candidate):
    self.long_control_state = LongCtrlState.off  # initialized to off
    kdBP = [0., 16., 35.]
    kdV = [0.05, 0.2, 0.5]

    self.pid = LongPIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                                 (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                                 (kdBP, kdV),
                                 rate=RATE,
                                 sat_limit=0.8,
                                 convert=compute_gb)
    self.v_pid = 0.0
    self.last_output_gb = 0.0
    self.rst = 0
    self.count = 0
    self.accel_limiter = 0
    
    self.op_params = opParams()
    self.dynamic_gas = DynamicGas(CP, candidate)

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, v_target, v_target_future, a_target, CP, extras):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Actuation limits
    gas_max = interp(CS.vEgo, CP.gasMaxBP, CP.gasMaxV)
    brake_max = interp(CS.vEgo, CP.brakeMaxBP, CP.brakeMaxV)

    # Update state machine
    output_gb = self.last_output_gb
    self.long_control_state = long_control_state_trans(active, self.long_control_state, CS.vEgo,
                                                       v_target_future, self.v_pid, output_gb,
                                                       CS.brakePressed, CS.cruiseState.standstill, CP.minSpeedCan)

    v_ego_pid = max(CS.vEgo, CP.minSpeedCan)  # Without this we get jumps, CAN bus reports 0 when speed < 0.3

    if self.long_control_state == LongCtrlState.off or CS.gasPressed:
      self.reset(v_ego_pid)
      output_gb = 0.
    # tracking objects and driving
    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target
      self.pid.pos_limit = gas_max
      self.pid.neg_limit = - brake_max

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      prevent_overshoot = not CP.stoppingControl and CS.vEgo < 1.5 and v_target_future < 0.7
      deadzone = interp(v_ego_pid, CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV)

      output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, deadzone=deadzone, feedforward=a_target, freeze_integrator=prevent_overshoot)

      gb_limit = interp(CS.vEgo, GdMAX_V, GdMAX_OUT)
      
      #if self.accel_limiter and not lead_car:
      if self.accel_limiter and output_gb > 0:   # Test if this is good for lead car also
        if (output_gb - self.last_output_gb) > gb_limit:
          output_gb = self.last_output_gb + gb_limit
      
      if prevent_overshoot:
        output_gb = min(output_gb, 0.0)
      
      self.count = self.count + 1
      if self.count > 200:
        self.accel_limiter = 1
        self.count = 0

    # Intention is to stop, switch to a different brake control until we stop
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      if not CS.standstill or output_gb > -BRAKE_STOPPING_TARGET:
        #output_gb -= CP.stoppingBrakeRate / RATE
        output_gb -= CP.stoppingBrakeRate / 10
      #output_gb = clip(output_gb, -brake_max, gas_max)    #Orig
      output_gb = clip(output_gb, -.45, gas_max)

      self.reset(CS.vEgo)

    # Intention is to move again, release brake fast before handing control to PID
    elif self.long_control_state == LongCtrlState.starting:
      #if output_gb < -0.2:                           # Original
        # output_gb += CP.startingBrakeRate / RATE    # Original value
      if output_gb < -0.05:
        output_gb += CP.startingBrakeRate / 2
      self.reset(CS.vEgo)

    self.last_output_gb = output_gb
    final_gas = clip(output_gb, 0., gas_max)
    final_brake = -clip(output_gb, -brake_max, 0.)

    return float(final_gas), float(final_brake)
