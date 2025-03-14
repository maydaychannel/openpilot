const int HYUNDAI_MAX_STEER = 384;             // like stock
const int HYUNDAI_MAX_RT_DELTA = 112;          // max delta torque allowed for real time checks
const uint32_t HYUNDAI_RT_INTERVAL = 250000;   // 250ms between real time checks
const int HYUNDAI_MAX_RATE_UP = 3;
const int HYUNDAI_MAX_RATE_DOWN = 7;
const int HYUNDAI_DRIVER_TORQUE_ALLOWANCE = 50;
const int HYUNDAI_DRIVER_TORQUE_FACTOR = 2;
const int HYUNDAI_STANDSTILL_THRSLD = 30;  // ~1kph
const CanMsg HYUNDAI_TX_MSGS[] = {
  {558, 2, 5}, // SSC Bus 2
  // {1056, 0, 8}, //   SCC11,  Bus 0
  // {1057, 0, 8}, //   SCC12,  Bus 0
  // {1290, 0, 8}, //   SCC13,  Bus 0
  // {905, 0, 8},  //   SCC14,  Bus 0
  // {1186, 0, 8}  //   4a2SCC, Bus 0
 };

// TODO: missing checksum for wheel speeds message,worst failure case is
//       wheel speeds stuck at 0 and we don't disengage on brake press
AddrCheckStruct hyundai_rx_checks[] = {
  {.msg = {{0x156, 0, 6, .check_checksum = false, .expected_timestep = 10000U}}}, // 156 is 342, steering eps data
  {.msg = {{0x17C, 0, 8, .check_checksum = false, .expected_timestep = 10000U}}}, // 17c is 380, powertrain data
  {.msg = {{0x1D0, 0, 8, .check_checksum = false, .expected_timestep = 30000U}}}, // 1d0 is 464, wheel speeds
  {.msg = {{0x309, 0, 8, .check_checksum = false, .expected_timestep = 25000U}}}, // 309 is 777, car speed
  {.msg = {{0x22F, 2, 8, .check_checksum = false, .max_counter = 15U, .expected_timestep = 10000U}}},  // SSC (559)
};
const int HYUNDAI_RX_CHECK_LEN = sizeof(hyundai_rx_checks) / sizeof(hyundai_rx_checks[0]);

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_legacy_rx_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{902, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{916, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
};
const int HYUNDAI_LEGACY_RX_CHECK_LEN = sizeof(hyundai_legacy_rx_checks) / sizeof(hyundai_legacy_rx_checks[0]);

bool hyundai_legacy = false;

static uint8_t hyundai_get_counter(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t cnt;
  if (addr == 129) {
    cnt = GET_BYTE(to_push, 7) & 0xF;
  } else if (addr == 357) {
    cnt = GET_BYTE(to_push, 6) & 0xF;
  } else if (addr == 559) {
    cnt = GET_BYTE(to_push, 1) & 0xF;
  } else if (addr == 608) {
    cnt = (GET_BYTE(to_push, 7) >> 4) & 0x3;
  } else if (addr == 688) {
    cnt = GET_BYTE(to_push, 4) & 0xF;
  } else if (addr == 902) {
    cnt = ((GET_BYTE(to_push, 3) >> 6) << 2) | (GET_BYTE(to_push, 1) >> 6);
  } else if (addr == 916) {
    cnt = (GET_BYTE(to_push, 1) >> 5) & 0x7;
  } else if (addr == 1057) {
    cnt = GET_BYTE(to_push, 7) & 0xF;
  } else {
    cnt = 0;
  }
  return cnt;
}

static uint8_t hyundai_get_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum;
  if (addr == 129) {
    chksum = (GET_BYTE(to_push, 7) >> 4) & 0xF;
  } else if (addr == 357) {
    chksum = GET_BYTE(to_push, 7);
  } else if (addr == 559) {
    chksum = GET_BYTE(to_push, 0);
  } else if (addr == 608) {
    chksum = GET_BYTE(to_push, 7) & 0xF;
  } else if (addr == 688) {
    chksum = (GET_BYTE(to_push, 4) >> 4) & 0xF;
  } else if (addr == 916) {
    chksum = GET_BYTE(to_push, 6) & 0xF;
  } else if (addr == 1057) {
    chksum = GET_BYTE(to_push, 7) >> 4;
  } else {
    chksum = 0;
  }
  return chksum;
}

//static uint8_t hyundai_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
//  int addr = GET_ADDR(to_push);

//  uint8_t chksum = 0;
  // same algorithm, but checksum is in a different place
//  for (int i = 0; i < 8; i++) {
//    uint8_t b = GET_BYTE(to_push, i);
//    if (((addr == 608) && (i == 7)) || ((addr == 916) && (i == 6)) || ((addr == 129) && (i == 7))) {
//      // b &= (addr == 1057) ? 0x0FU : 0xF0U; // remove checksum
//      b &= (addr == 129) ? 0x0FU : 0xF0U; // remove checksum
//    }
//    chksum += (b % 16U) + (b / 16U);
//  }
//  return (16U - (chksum %  16U)) % 16U;
//}

static uint8_t hyundai_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);
  uint8_t chksum = 0;
  uint8_t data_length = 8;

  if (addr == 357 || addr == 688) {
    data_length = (addr == 688) ? 5 : 7;
    // Use XOR checksum algorithm on the first 7 bytes for 357 and 5 bytes for 688
    for (int i = 0; i < data_length; i++) {
      uint8_t b = GET_BYTE(to_push, i);
      // Remove checksum nibble based on address and byte position
      if (addr == 688 && i == 4) {
        b &= 0x0FU;  // Mask checksum byte
      }
      chksum ^= b;
    }
    if (addr == 688) {
      //chksum &= 0x0FU;
      chksum = (chksum & 0x0F) ^ (chksum >> 4);
    }
  } else if (addr == 559) {
    uint16_t ssc_chksum = addr;
    data_length = 7;
    // 0 byte is the checksum
    for (int i = 1; i < data_length; i++) {
      uint8_t b = GET_BYTE(to_push, i);
      ssc_chksum += b;
    }
    // Add upper and lower bytes of the checksum
    ssc_chksum = (ssc_chksum & 0xFF) + (ssc_chksum >> 8);

    // Mask to keep only the lower 8 bits
    chksum = ssc_chksum & 0xFF;
  } else {
    // Standard checksum algorithm for addresses 608, 916, and 129
    for (int i = 0; i < data_length; i++) {
      uint8_t b = GET_BYTE(to_push, i);

      // Remove checksum nibble based on address and byte position
      if ((addr == 608 && i == 7) || (addr == 129 && i == 7)) {
        b &= (addr == 129) ? 0x0FU : 0xF0U;  // Mask checksum byte
      }

      // Sum the nibbles (4-bit parts of the byte)
      chksum += (b % 16U) + (b / 16U);
    }

    // Final checksum calculation with modulo 16 for other addresses
    chksum = (16U - (chksum % 16U)) % 16U;
  }

  return chksum;
}

static int hyundai_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid;
  if (hyundai_legacy) {
    valid = addr_safety_check(to_push, hyundai_legacy_rx_checks, HYUNDAI_LEGACY_RX_CHECK_LEN,
                              hyundai_get_checksum, hyundai_compute_checksum,
                              hyundai_get_counter);

  } else {
    valid = addr_safety_check(to_push, hyundai_rx_checks, HYUNDAI_RX_CHECK_LEN,
                              hyundai_get_checksum, hyundai_compute_checksum,
                              hyundai_get_counter);
  }

  if (valid && (GET_BUS(to_push) == 0)) {
    int addr = GET_ADDR(to_push);

    // Skip the steer torque as now, add in SSC stuff later
    // if (addr == 593) {
    //   int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ff) * 0.79) - 808; // scale down new driver torque signal to match previous one
    //   // update array of samples
    //   update_sample(&torque_driver, torque_driver_new);
    // }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (addr == 356) {  // 0x164
      // 5th bit is CRUISE_ACTIVE
      int cruise_engaged = GET_BYTE(to_push, 0) & 0x20;
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      cruise_engaged_prev = cruise_engaged;

    }

    if (!gas_interceptor_detected) {
      if (addr == 0x17C) {
        gas_pressed = (GET_BYTE((to_push), 4) & 0x80) != 0;
      }
    }


    // sample wheel speed, averaging opposite corners
    if (addr == 344) {  // 0x158
      // first 2 bytes
      vehicle_moving = GET_BYTE(to_push, 0) | GET_BYTE(to_push, 1);

    if (addr == 380) {    // 0x17C
      brake_pressed = (GET_BYTE((to_push), 6) & 0x20) != 0;
    }

    // generic_rx_checks((addr == 832));
    generic_rx_checks(false);        // This was used in dzids safety_bmw, make work later!!!
  }
  return valid;
}

static int hyundai_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (!msg_allowed(to_send, HYUNDAI_TX_MSGS, sizeof(HYUNDAI_TX_MSGS)/sizeof(HYUNDAI_TX_MSGS[0]))) {
    tx = 0;
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // LKA STEER: safety check
  if (addr == 832) {
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ff) - 1024;
    uint32_t ts = TIM2->CNT;
    bool violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, HYUNDAI_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((addr == 1265) && !controls_allowed) {
    if ((GET_BYTES_04(to_send) & 0x7) != 4) {
      tx = 0;
    }
  }

  // 1 allows the message through
  return tx;
}

static int hyundai_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  // forward cam to ccan and viceversa, except lkas cmd
  if (!relay_malfunction) {
    if (bus_num == 0) {
      bus_fwd = 2;
    }
    if ((bus_num == 2) && (addr != 832) && (addr != 1157)) {
      bus_fwd = 0;
    }
  }
  return bus_fwd;
}

static void hyundai_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();

  hyundai_legacy = false;
}

static void hyundai_legacy_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();

  hyundai_legacy = true;
}

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
  .addr_check = hyundai_rx_checks,
  .addr_check_len = sizeof(hyundai_rx_checks) / sizeof(hyundai_rx_checks[0]),
};

const safety_hooks hyundai_legacy_hooks = {
  .init = hyundai_legacy_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
  .addr_check = hyundai_legacy_rx_checks,
  .addr_check_len = sizeof(hyundai_legacy_rx_checks) / sizeof(hyundai_legacy_rx_checks[0]),
};
