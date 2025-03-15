import copy
from cereal import car
from common.numpy_fast import mean
from selfdrive.car.hyundai.values import DBC, STEER_THRESHOLD, FEATURES, EV_HYBRID
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV

GearShifter = car.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    # SSC assembly integrity tracking
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
    self.ssc_steer_angle = True    # This is first set to false so we ignore the SCC angle stuff
    self.angle_offset = 0.0
    # Initialize variables to store the min and max error values
    self.steeringAngle_aligned = False
    self.min_error = 0.0
    self.max_error = 0.0

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()
    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.doorOpen = False
    ret.seatbeltUnlatched = False

    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = cp.vl["STEERING_EPS_DATA"]['STEER_ANGLE']
    ret.steeringRateDeg = cp.vl["STEERING_EPS_DATA"]['STEER_ANGLE_RATE']
    ret.leftBlinker = cp.vl['SCM_FEEDBACK']["LEFT_BLINKER"] == 1
    ret.rightBlinker = cp.vl['SCM_FEEDBACK']["RIGHT_BLINKER"] == 1

    #ret.steeringTorqueEps = cp.vl["VSM2"]['CR_Mdps_OutTq']
    ret.steeringTorqueEps = cp_cam.vl["STEERING_STATUS"]['STEERING_TORQUE']
    #ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerWarning = False

    # EPS and SSC angle difference tracking for assembly integrity monitoring
    if self.ssc_steer_angle:
      if self.needs_angle_offset:
        self.angle_offset = (cp_cam.vl["STEERING_STATUS"]['STEERING_ANGLE']) - ret.steeringAngleDeg
        self.needs_angle_offset = False
      else:
        # After angle_offset has been set, start measuring aligned SSC angle
        ret.steeringAngleDegSSC = (cp_cam.vl["STEERING_STATUS"]['STEERING_ANGLE']) - self.angle_offset
        if abs(ret.steeringAngleDeg - ret.steeringAngleDegSSC) < 0.1:
          self.steeringAngle_aligned = True
        # Calculate the error (difference) between the two sensor readings
        ret.steeringAngleDegError = ret.steeringAngleDegSSC - ret.steeringAngleDeg

        # Track the minimum and maximum error values
        if self.steeringAngle_aligned == True:
          self.max_error = max(self.max_error, ret.steeringAngleDegError)
          self.min_error = min(self.min_error, ret.steeringAngleDegError)

        ret.steeringAngleDegDivergence = self.max_error - self.min_error

    ret.cruiseState.available = bool(cp.vl["CRUISE_STATUS"]['CRUISE_ON'])
    ret.cruiseState.enabled = bool(cp.vl["CRUISE_STATUS"]['CRUISE_ON'])
    ret.cruiseState.standstill = False
    ret.cruiseState.speed = 0


    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["POWERTRAIN_DATA"]['BRAKE_PRESSED'] != 0

    # TODO: Check this
    ret.brakeLights = bool(cp.vl["POWERTRAIN_DATA"]['BRAKE_PRESSED'] or ret.brakePressed)
    ret.gas = cp.vl["POWERTRAIN_DATA"]['GAS_PRESSED']
    ret.gasPressed = cp.vl["POWERTRAIN_DATA"]['GAS_PRESSED'] > 0.05

    ret.gearShifter = GearShifter.drive	# Force D-gear otherwise because my car is manual

    # emulate driver steering torque - allows lane change assist on blinker hold
    ret.steeringPressed = ret.gasPressed # i30 doesn't have good driver intervention detection yet, so lightly pressing the gas indicates driver intention
    if ret.steeringPressed and ret.leftBlinker:
      ret.steeringTorque = 1
    elif ret.steeringPressed and  ret.rightBlinker:
      ret.steeringTorque = -1
    else:
      ret.steeringTorque = 0

    ret.stockAeb = False
    ret.stockFcw = False

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = False

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
      ("GAS_PRESSED", "POWERTRAIN_DATA", 0),
      ("LEFT_BLINKER", "SCM_FEEDBACK", 0),
      ("RIGHT_BLINKER", "SCM_FEEDBACK", 0),
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
      # sig_name, sig_address, default
      ("STEERING_TORQUE", "STEERING_STATUS", 0),
      ("STEERING_ANGLE", "STEERING_STATUS", 0)
    ]

    checks = [
      ("STEERING_STATUS", 20)    # Checks if SSC is connected
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
