
def create_new_steer_command(packer, mode, steer_delta, steer_tq, frame):
  """Creates a CAN message for the actuator STEERING_COMMAND"""
#  packer = CANPacker('ocelot_controls')
  values = {
    "SERVO_COUNTER": frame % 0xF,
    "STEER_MODE": mode,
    "STEER_ANGLE": steer_delta,
    "STEER_TORQUE": steer_tq,
  }
  msg = packer.make_can_msg("STEERING_COMMAND", 0, values)
  addr = msg[0]
  dat  = msg[2]

  values["SERVO_CHECKSUM"] = calc_checksum_8bit(dat, addr)

  return packer.make_can_msg("STEERING_COMMAND", 0, values) #bus 2 is the actuator CAN bus

def calc_checksum_8bit(work_data, msg_id): # 0xb8 0x1a0 0x19e 0xaa 0xbf
  checksum = msg_id
  for byte in work_data: #checksum is stripped from the data
    checksum += byte     #add up all the bytes

  checksum = (checksum & 0xFF) + (checksum >> 8); #add upper and lower Bytes
  checksum &= 0xFF #throw away anything in upper Byte
  return checksum

def create_fcw_command(packer, fcw):
  values = {
    "FCW": fcw,
    "SET_ME_X20": 0x20,
    "SET_ME_X10": 0x10,
    "SET_ME_X80": 0x80,
  }
  return packer.make_can_msg("ACC_HUD", 0, values)

