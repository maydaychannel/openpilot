from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
    self.accurate_steer_angle_seen = CP.hasZss
    self.angle_offset = 0.

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()
    ret.brakePressed = 0 != 0
    ret.brakeLights = bool(ret.brakePressed)
    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    self.low_speed_lockout = False

    ret.standstill = ret.vEgoRaw < 0.01    #Changed this from 0.001 to 0.1 to 0.01 bc longcontrol.py uses this to detect when car is stopped

    if self.CP.carFingerprint == CAR.OLD_CAR: # STILL NEED TO CHECK THIS, AND STEERINGRATE
      ret.steeringAngleDeg = -(cp.vl["STEERING_EPS_DATA"]['STEER_ANGLE'])
    
    if self.CP.carFingerprint == CAR.OLD_CAR: # Steering rate sensor is code differently on CIVIC
      if cp.vl["STEERING_EPS_DATA"]['STEER_ANGLE_RATE'] == 0:
        ret.steeringRateDeg = (cp.vl["STEERING_EPS_DATA"]['STEER_ANGLE_RATE'])

    
    #ret.leftBlinker = cp.vl["SCM_FEEDBACK"]['LEFT_BLINKER']
    #ret.rightBlinker = cp.vl["SCM_FEEDBACK"]['RIGHT_BLINKER']

    ret.steeringTorque = cp.vl["STEERING_STATUS"]['STEERING_TORQUE']
    ret.steeringTorqueEps = cp.vl["STEERING_STATUS"]['STEERING_TORQUE']
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerWarning = False

    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = 2 #standby for testing


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
      ("STEERING_TORQUE", "STEERING_STATUS", 0)
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
      # signal name, message name, default value
      #("STEERING_ANGLE", "STEERING_STATUS", 0),
      #("STEERING_TORQUE", "STEERING_STATUS", 0),
      #("STEERING_SPEED", "STEERING_STATUS", 0),
      #("CONTROL_STATUS", "STEERING_STATUS", 0),
      #("TEMPERATURE", "STEERING_STATUS", 0)
    ]
    checks = [
      # refresh frequency Hz
      #("STEERING_STATUS", 100),
    ] 
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)  # 2: Actuator-CAN
