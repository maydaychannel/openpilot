from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD, TSS2_CAR, NO_STOP_TIMER_CAR


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["AGS_1"]['GEAR_SELECTOR']

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
    self.accurate_steer_angle_seen = True
    self.angle_offset = 0.
    # Initialize variables to store the min and max error values
    self.steeringAngle_aligned = False
    self.min_error = 0
    self.max_error = 0


  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = False
    ret.seatbeltUnlatched = False

    ret.brakePressed = cp.vl["POWERTRAIN_DATA"]['BRAKE_SWITCH'] != 0
    ret.brakeLights = bool(cp.vl["POWERTRAIN_DATA"]['BRAKE_SWITCH'] or ret.brakePressed)
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
      ret.gasPressed = ret.gas > 15
    else:
      ret.gas = cp.vl["POWERTRAIN_DATA"]['BRAKE_SWITCH']
      ret.gasPressed = cp.vl["POWERTRAIN_DATA"]['GAS_PRESSED'] > 0.05

    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.01    #Changed this from 0.001 to 0.1 to 0.01 bc longcontrol.py uses this to detect when car is stopped
    if self.accurate_steer_angle_seen:
      # else:
      #   ret.steeringAngleDegSSC = cp.vl["STEERING_STATUS"]['STEERING_ANGLE'] - self.angle_offset
      if self.needs_angle_offset:
        angle_wheel = (cp.vl["STEERING_EPS_DATA"]['STEER_ANGLE'])
        if abs(angle_wheel) > 1e-3:
          self.needs_angle_offset = False
          ret.steeringAngleDeg = angle_wheel
          self.angle_offset = cp.vl["STEERING_STATUS"]['STEERING_ANGLE'] - angle_wheel
      else:
        # After angle_offset has been set, start measuring aligned SSC angle
        ret.steeringAngleDegSSC = cp.vl["STEERING_STATUS"]['STEERING_ANGLE'] - self.angle_offset
        if abs(ret.steeringAngleDeg - ret.steeringAngleDegSSC) < 0.1:
          self.steeringAngle_aligned = True
    ret.steeringAngleDeg = cp.vl["STEERING_EPS_DATA"]['STEER_ANGLE']
    ret.steeringRateDeg = cp.vl["STEERRING_EPS_DATA"]['STEER_ANGLE_RATE']

    if self.steeringAngle_aligned:
      # Calculate the error (difference) between the two sensor readings
      ret.steeringAngleDegError = (ret.steeringAngleDegSSC * 0.96) -  ret.steeringAngleDeg

      # Track the minimum and maximum error values
      if abs(ret.steeringAngleDeg) < 90:
        self.max_error = max(self.max_error, ret.steeringAngleDegError)
        self.min_error = min(self.min_error, ret.steeringAngleDegError)

      ret.steeringAngleDegDivergence = self.max_error - self.min_error

    self.pcm_acc_status = cp.vl["CRUISE_STATUS"]['CRUISE_ON']
    #can_gear = int(cp.vl["AGS_1"]['GEAR_SELECTOR'])
    #ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl['SCM_FEEDBACK']["LEFT_BLINKER"] == 1
    ret.rightBlinker = cp.vl['SCM_FEEDBACK']["RIGHT_BLINKER"] == 1

    # emulate driver steering torque - allows lane change assist on blinker hold
    ret.steeringPressed = ret.gasPressed # E-series doesn't have torque sensor, so lightly pressing the gas indicates driver intention
    if ret.steeringPressed and ret.leftBlinker:
      ret.steeringTorque = 1
    elif ret.steeringPressed and  ret.rightBlinker:
      ret.steeringTorque = -1
    else:
      ret.steeringTorque = 0

    ret.steeringTorqueEps = cp.vl["STEERING_STATUS"]['STEERING_TORQUE']
    ret.steerWarning = False
    #ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]['MAIN_ON'] != 0
    #ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]['SET_SPEED'] * CV.KPH_TO_MS
    #self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]['LOW_SPEED_LOCKOUT'] == 2
    #self.pcm_acc_status = cp.vl["PCM_CRUISE"]['CRUISE_STATE']
    if self.CP.carFingerprint in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor:
      # ignore standstill in hybrid vehicles, since pcm allows to restart without
      # receiving any special command. Also if interceptor is detected
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["CRUISE_STATUS"]['CRUISE_ON'])
    #ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]['CRUISE_STATE'] in [1, 2, 3, 4, 5, 6]

    ret.stockAeb = bool(cp_cam.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_cam.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    ret.espDisabled = cp.vl["POWERTRAIN_DATA"]['BRAKE_SWITCH'] != 0
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = 2

    ret.epsDisabled = (True if ret.genericToggle == 0 else False)

    return ret

  @staticmethod
  def get_can_parser(CP):
    
    signals = [
      # sig_name, sig_address, default
      ("STEER_ANGLE", "STEERING_EPS_DATA", 0),
      ("STEER_ANGLE_RATE", "STEERING_EPS_DATA", 0),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
      ("BRAKE_PRESSED", "POWERTRAIN_DATA", 0),
      ("LEFT_BLINKER", "SCM_FEEDBACK", 0),
      ("RIGHT_BLINKER", "SCM_FEEDBACK", 0),
      ("STEERING_TORQUE", "STEERING_STATUS", 0),
      ("STEERING_ANGLE", "STEERING_STATUS", 0),
      ("GAS PRESSED", "POWERTRAIN_DATA", 0),
      ("CRUISE_ON", "CRUISE_STATUS", 0)
    ]

    checks = [
      ("WHEEL_SPEEDS", 50),
      ("POWERTRAIN_DATA", 100),
      ("STEERING_EPS_DATA", 100)
    ]


    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
    ]

    # use steering message to check if panda is connected to frc
    checks = [
      #("STEERING_STATUS", 100),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)
