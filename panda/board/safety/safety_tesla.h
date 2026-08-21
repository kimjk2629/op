// ============================================================================
// AP1 / AP2 Model S (full-bypass harness) constants -- unchanged from upstream op3t.
// ============================================================================
const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_UP = {
    {2., 7., 17.},
    {5., .8, .25}};

const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_DOWN = {
    {2., 7., 17.},
    {5., 3.5, .8}};

const int TESLA_DEG_TO_CAN = 10;
const float TESLA_MAX_ACCEL = 2.0;  // m/s^2
const float TESLA_MIN_ACCEL = -3.5; // m/s^2

const int TESLA_FLAG_POWERTRAIN = 1;
const int TESLA_FLAG_LONGITUDINAL_CONTROL = 2;

const CanMsg TESLA_TX_MSGS[] = {
  {0x488, 0, 4},  // DAS_steeringControl
  {0x45, 0, 8},   // STW_ACTN_RQ
  {0x45, 2, 8},   // STW_ACTN_RQ
  {0x2b9, 0, 8},  // DAS_control
};
#define TESLA_TX_LEN (sizeof(TESLA_TX_MSGS) / sizeof(TESLA_TX_MSGS[0]))

const CanMsg TESLA_PT_TX_MSGS[] = {
  {0x2bf, 0, 8},  // DAS_control
};
#define TESLA_PT_TX_LEN (sizeof(TESLA_PT_TX_MSGS) / sizeof(TESLA_PT_TX_MSGS[0]))

const int TESLA_NO_ACCEL_VALUE = 375;  // value sent when not requesting acceleration

// ============================================================================
// Model Y / Model 3 (HW3/HW4 "party bus" harness) constants.
// Ported from carrot-wip (opendbc/safety/safety/safety_tesla.h) on 2026-08-21 and rewritten
// against op3t's older panda safety API (AddrCheckStruct / msg_allowed / addr_safety_check
// instead of RxCheck / BUILD_SAFETY_CFG / steer_angle_cmd_checks). NOT independently verified:
// this has not been run through comma's safety unit test suite in this repo, nor bench/road
// tested on a real vehicle. Do not rely on it for anything beyond a starting point for your
// own verification. See TESLA_PORT_README.md at the repo root.
//
// IMPORTANT: this safety mode is currently only compiled in when ALLOW_DEBUG is set (see
// safety.h) -- i.e. it is NOT part of a normal release panda firmware build. That mirrors how
// this fork already treats the existing AP1/AP2 Tesla support (also debug-gated,
// dashcamOnly-only). Re-enabling it for a release build is a deliberate decision the user
// must make after validating this code, not a default of this patch.
// ============================================================================
const struct lookup_t TESLA_MY_LOOKUP_ANGLE_RATE_UP = {
    {0., 5., 25.},
    {2.5, 1.5, .2}};

const struct lookup_t TESLA_MY_LOOKUP_ANGLE_RATE_DOWN = {
    {0., 5., 25.},
    {5.0, 2.0, .3}};

const float TESLA_MY_MAX_ACCEL = 2.0;    // m/s^2
const float TESLA_MY_MIN_ACCEL = -3.48;  // m/s^2
const int TESLA_MY_NO_ACCEL_VALUE = 375; // value sent when not requesting acceleration (same encoding as AP1/AP2)

const int TESLA_FLAG_MODEL3_Y = 4;
const int TESLA_FLAG_MODEL3_Y_LONG_CONTROL = 8;
const int TESLA_FLAG_MODEL3_Y_FSD_14 = 16;

const CanMsg TESLA_MY_TX_MSGS[] = {
  {0x488, 0, 4},  // DAS_steeringControl
  {0x2b9, 0, 8},  // DAS_control
  {0x27d, 0, 3},  // APS_eacMonitor (steering-allowed heartbeat)
};
#define TESLA_MY_TX_LEN (sizeof(TESLA_MY_TX_MSGS) / sizeof(TESLA_MY_TX_MSGS[0]))

AddrCheckStruct tesla_my_addr_checks[] = {
  {.msg = {{0x2b9, 2, 8, .expected_timestep = 40000U}, { 0 }, { 0 }}},   // DAS_control on bus 2 (25Hz)
  {.msg = {{0x257, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},   // DI_speed (50Hz)
  {.msg = {{0x370, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},   // EPAS3S_sysStatus (100Hz)
  {.msg = {{0x118, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},   // DI_systemStatus (100Hz)
  {.msg = {{0x145, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},   // ESP_status (50Hz)
  {.msg = {{0x286, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},  // DI_state (10Hz)
  {.msg = {{0x311, 0, 7, .expected_timestep = 100000U}, { 0 }, { 0 }}},  // UI_warning (10Hz)
};
#define TESLA_MY_ADDR_CHECK_LEN (sizeof(tesla_my_addr_checks) / sizeof(tesla_my_addr_checks[0]))
addr_checks tesla_my_rx_checks = {tesla_my_addr_checks, TESLA_MY_ADDR_CHECK_LEN};

bool tesla_model3_y = false;
bool tesla_my_longitudinal = false;
bool tesla_my_fsd_14 = false;
bool tesla_my_stock_steering_control = false;
bool tesla_my_stock_steering_control_prev = false;
bool tesla_my_summon = false;
bool tesla_my_summon_prev = false;

static int tesla_my_get_steer_ctrl_type(int ctrl_type) {
  int out = ctrl_type;
  if (tesla_my_fsd_14) {
    if (ctrl_type == 1) { out = 2; }
    else if (ctrl_type == 2) { out = 1; }
    else { out = ctrl_type; }
  }
  return out;
}

// ============================================================================

AddrCheckStruct tesla_addr_checks[] = {
  {.msg = {{0x370, 0, 8, .expected_timestep = 40000U}, { 0 }, { 0 }}},   // EPAS_sysStatus (25Hz)
  {.msg = {{0x108, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},   // DI_torque1 (100Hz)
  {.msg = {{0x118, 0, 6, .expected_timestep = 10000U}, { 0 }, { 0 }}},   // DI_torque2 (100Hz)
  {.msg = {{0x20a, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},   // BrakeMessage (50Hz)
  {.msg = {{0x368, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},  // DI_state (10Hz)
  {.msg = {{0x318, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},  // GTW_carState (10Hz)
};
#define TESLA_ADDR_CHECK_LEN (sizeof(tesla_addr_checks) / sizeof(tesla_addr_checks[0]))
addr_checks tesla_rx_checks = {tesla_addr_checks, TESLA_ADDR_CHECK_LEN};

AddrCheckStruct tesla_pt_addr_checks[] = {
  {.msg = {{0x106, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},   // DI_torque1 (100Hz)
  {.msg = {{0x116, 0, 6, .expected_timestep = 10000U}, { 0 }, { 0 }}},   // DI_torque2 (100Hz)
  {.msg = {{0x1f8, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},   // BrakeMessage (50Hz)
  {.msg = {{0x256, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},  // DI_state (10Hz)
};
#define TESLA_PT_ADDR_CHECK_LEN (sizeof(tesla_pt_addr_checks) / sizeof(tesla_pt_addr_checks[0]))
addr_checks tesla_pt_rx_checks = {tesla_pt_addr_checks, TESLA_PT_ADDR_CHECK_LEN};

bool tesla_longitudinal = false;
bool tesla_powertrain = false;  // Are we the second panda intercepting the powertrain bus?

bool tesla_stock_aeb = false;

static int tesla_rx_hook(CANPacket_t *to_push) {
  addr_checks *current_rx_checks = &tesla_rx_checks;
  if (tesla_model3_y) {
    current_rx_checks = &tesla_my_rx_checks;
  } else if (tesla_powertrain) {
    current_rx_checks = &tesla_pt_rx_checks;
  } else {
  }

  bool valid = addr_safety_check(to_push, current_rx_checks, NULL, NULL, NULL);

  if (valid) {
    int bus = GET_BUS(to_push);
    int addr = GET_ADDR(to_push);

    if (tesla_model3_y) {
      // ---- Model Y / Model 3 party-bus rx ----
      if (bus == 0) {
        if (addr == 0x370) {
          // EPAS3S_internalSAS: (0.1 * val) - 819.2 in deg. Store in 1/10 deg to match steering request.
          int angle_meas_new = (((GET_BYTE(to_push, 4) & 0x3FU) << 8) | GET_BYTE(to_push, 5)) - 8192U;
          update_sample(&angle_meas, angle_meas_new);
        }

        if (addr == 0x257) {
          // DI_speed / DI_vehicleSpeed: (val * 0.08) - 40, kph -> m/s
          vehicle_speed = ((((GET_BYTE(to_push, 2) << 4) | (GET_BYTE(to_push, 1) >> 4)) * 0.08) - 40) * 0.2778;
          vehicle_moving = ABS(vehicle_speed) > 0.1;
        }

        if (addr == 0x118) {
          // DI_systemStatus / DI_accelPedalPos
          gas_pressed = (GET_BYTE(to_push, 4) != 0U);
        }

        if (addr == 0x145) {
          // ESP_status / ESP_driverBrakeApply == 2 (Driver_applying_brakes)
          brake_pressed = (((GET_BYTE(to_push, 3) >> 5) & 0x03U) == 2U);
        }

        if (addr == 0x286) {
          // DI_state: autopark/summon + cruise state
          int autopark_state = (GET_BYTE(to_push, 3) >> 1) & 0x0FU;
          bool summon_now = (autopark_state == 3) ||  // ACTIVE
                            (autopark_state == 4) ||   // COMPLETE
                            (autopark_state == 9);     // SELFPARK_STARTED

          if (summon_now && !tesla_my_summon_prev && !cruise_engaged_prev) {
            tesla_my_summon = true;
          }
          if (!summon_now) {
            tesla_my_summon = false;
          }
          tesla_my_summon_prev = summon_now;

          int cruise_state = (GET_BYTE(to_push, 1) >> 4) & 0x07U;
          bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                                (cruise_state == 3) ||   // STANDSTILL
                                (cruise_state == 4) ||   // OVERRIDE
                                (cruise_state == 6) ||   // PRE_FAULT
                                (cruise_state == 7);     // PRE_CANCEL
          cruise_engaged = cruise_engaged && !tesla_my_summon;

          if (cruise_engaged && !cruise_engaged_prev) {
            controls_allowed = 1;
          }
          if (!cruise_engaged) {
            controls_allowed = 0;
          }
          cruise_engaged_prev = cruise_engaged;
        }
      }

      if (bus == 2) {
        if (tesla_my_longitudinal && (addr == 0x2b9)) {
          // DAS_control "AEB_ACTIVE"
          tesla_stock_aeb = (GET_BYTE(to_push, 2) & 0x03U) == 1U;
        }

        if (addr == 0x488) {
          // DAS_steeringControl: detect stock steering control (LDA/ELDA/Autopark)
          int steering_control_type = GET_BYTE(to_push, 2) >> 6;
          bool stock_now = steering_control_type != 0;  // any non-NONE

          if (stock_now && !tesla_my_stock_steering_control_prev) {
            tesla_my_stock_steering_control = true;
          }
          if (!stock_now) {
            tesla_my_stock_steering_control = false;
          }
          tesla_my_stock_steering_control_prev = stock_now;
        }
      }

      generic_rx_checks((addr == 0x488) && (bus == 0));  // DAS_steeringControl should not appear from us on bus 0
      generic_rx_checks((addr == 0x27d) && (bus == 0));  // APS_eacMonitor should not appear from us on bus 0
      if (tesla_my_longitudinal) {
        generic_rx_checks((addr == 0x2b9) && (bus == 0));
      }
    } else {
      // ---- AP1 / AP2 rx (unchanged) ----
      if (bus == 0) {
        if (!tesla_powertrain) {
          if (addr == 0x370) {
            int angle_meas_new = (((GET_BYTE(to_push, 4) & 0x3FU) << 8) | GET_BYTE(to_push, 5)) - 8192U;
            update_sample(&angle_meas, angle_meas_new);
          }
        }

        if (addr == (tesla_powertrain ? 0x116 : 0x118)) {
          vehicle_speed = (((((GET_BYTE(to_push, 3) & 0x0FU) << 8) | (GET_BYTE(to_push, 2))) * 0.05) - 25) * 0.447;
          vehicle_moving = ABS(vehicle_speed) > 0.1;
        }

        if (addr == (tesla_powertrain ? 0x106 : 0x108)) {
          gas_pressed = (GET_BYTE(to_push, 6) != 0U);
        }

        if (addr == (tesla_powertrain ? 0x1f8 : 0x20a)) {
          brake_pressed = (((GET_BYTE(to_push, 0) & 0x0CU) >> 2) != 1U);
        }

        if (addr == (tesla_powertrain ? 0x256 : 0x368)) {
          int cruise_state = (GET_BYTE(to_push, 1) >> 4);
          bool cruise_engaged = (cruise_state == 2) ||
                                (cruise_state == 3) ||
                                (cruise_state == 4) ||
                                (cruise_state == 6) ||
                                (cruise_state == 7);

          if (cruise_engaged && !cruise_engaged_prev) {
            controls_allowed = 1;
          }
          if (!cruise_engaged) {
            controls_allowed = 0;
          }
          cruise_engaged_prev = cruise_engaged;
        }
      }

      if (tesla_powertrain) {
        generic_rx_checks((addr == 0x2bf) && (bus == 0));
      } else {
        generic_rx_checks((addr == 0x488) && (bus == 0));
      }
    }
  }

  return valid;
}


static int tesla_tx_hook(CANPacket_t *to_send, bool longitudinal_allowed) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  bool violation = false;

  if (tesla_model3_y) {
    // ---- Model Y / Model 3 tx ----
    if (!msg_allowed(to_send, TESLA_MY_TX_MSGS, TESLA_MY_TX_LEN)) {
      tx = 0;
    }

    if (tesla_my_summon) {
      violation = true;
    }

    if (addr == 0x488) {
      // Steering control: (0.1 * val) - 1638.35 in deg. We use 1/10 deg as a unit here.
      int raw_angle_can = (((GET_BYTE(to_send, 0) & 0x7FU) << 8) | GET_BYTE(to_send, 1));
      int desired_angle = raw_angle_can - 16384;
      int steer_control_type = GET_BYTE(to_send, 2) >> 6;
      int angle_ctrl_type = tesla_my_get_steer_ctrl_type(1);
      int lkas_ctrl_type = tesla_my_get_steer_ctrl_type(2);
      bool steer_control_enabled = (steer_control_type == angle_ctrl_type) || (steer_control_type == lkas_ctrl_type);

      if (controls_allowed && steer_control_enabled) {
        float delta_angle_float;
        delta_angle_float = (interpolate(TESLA_MY_LOOKUP_ANGLE_RATE_UP, vehicle_speed) * TESLA_DEG_TO_CAN);
        int delta_angle_up = (int)(delta_angle_float) + 1;
        delta_angle_float = (interpolate(TESLA_MY_LOOKUP_ANGLE_RATE_DOWN, vehicle_speed) * TESLA_DEG_TO_CAN);
        int delta_angle_down = (int)(delta_angle_float) + 1;
        int highest_desired_angle = desired_angle_last + ((desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
        int lowest_desired_angle = desired_angle_last - ((desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

        violation |= max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle);
      }
      desired_angle_last = desired_angle;

      if (!controls_allowed && ((desired_angle < (angle_meas.min - 1)) || (desired_angle > (angle_meas.max + 1)))) {
        violation = true;
      }

      if (!controls_allowed && steer_control_enabled) {
        violation = true;
      }

      bool valid_steer_control_type = (steer_control_type == 0) ||
                                      (steer_control_type == angle_ctrl_type) ||
                                      (steer_control_type == lkas_ctrl_type);
      if (!valid_steer_control_type) {
        violation = true;
      }

      if (tesla_my_stock_steering_control) {
        // Don't fight LDA / ELDA / Autopark
        violation = true;
      }
    }

    if (addr == 0x2b9) {
      // DAS_control: longitudinal control / cancel message
      int aeb_event = GET_BYTE(to_send, 2) & 0x03U;
      if (aeb_event != 0) {
        violation = true;
      }

      if (tesla_stock_aeb) {
        violation = true;
      }

      int raw_accel_max = ((GET_BYTE(to_send, 6) & 0x1FU) << 4) | (GET_BYTE(to_send, 5) >> 4);
      int raw_accel_min = ((GET_BYTE(to_send, 5) & 0x0FU) << 5) | (GET_BYTE(to_send, 4) >> 3);
      int acc_state = GET_BYTE(to_send, 1) >> 4;
      float accel_max = (0.04 * raw_accel_max) - 15;
      float accel_min = (0.04 * raw_accel_min) - 15;

      if (tesla_my_longitudinal) {
        if ((raw_accel_max < TESLA_MY_NO_ACCEL_VALUE) && (raw_accel_min < TESLA_MY_NO_ACCEL_VALUE)) {
          // both requesting deceleration below the "no accel" baseline is not a valid combination
          violation = true;
        }
        if ((accel_max > TESLA_MY_MAX_ACCEL) || (accel_min > TESLA_MY_MAX_ACCEL)) {
          violation = true;
        }
        if ((accel_max < TESLA_MY_MIN_ACCEL) || (accel_min < TESLA_MY_MIN_ACCEL)) {
          violation = true;
        }
        if (!longitudinal_allowed) {
          if ((raw_accel_max != TESLA_MY_NO_ACCEL_VALUE) || (raw_accel_min != TESLA_MY_NO_ACCEL_VALUE)) {
            violation = true;
          }
        }
      } else {
        // Only a silent-cancel message is allowed when not doing longitudinal control
        if (acc_state != 13) {  // ACC_CANCEL_GENERIC_SILENT
          violation = true;
        }
        if ((raw_accel_max != TESLA_MY_NO_ACCEL_VALUE) || (raw_accel_min != TESLA_MY_NO_ACCEL_VALUE)) {
          violation = true;
        }
      }
    }

    // addr == 0x27d (APS_eacMonitor): no content-level checks beyond msg_allowed's length/bus/counter checks.

    if (violation) {
      tx = 0;
    }

    return tx;
  }

  // ---- AP1 / AP2 tx (unchanged) ----
  if (!msg_allowed(to_send,
                  tesla_powertrain ? TESLA_PT_TX_MSGS : TESLA_TX_MSGS,
                  tesla_powertrain ? TESLA_PT_TX_LEN : TESLA_TX_LEN)) {
    tx = 0;
  }

  if (!tesla_powertrain && (addr == 0x488)) {
    int raw_angle_can = (((GET_BYTE(to_send, 0) & 0x7FU) << 8) | GET_BYTE(to_send, 1));
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = GET_BYTE(to_send, 2) >> 6;
    bool steer_control_enabled = (steer_control_type != 0) &&
                                 (steer_control_type != 3);

    if (controls_allowed && steer_control_enabled) {
      float delta_angle_float;
      delta_angle_float = (interpolate(TESLA_LOOKUP_ANGLE_RATE_UP, vehicle_speed) * TESLA_DEG_TO_CAN);
      int delta_angle_up = (int)(delta_angle_float) + 1;
      delta_angle_float =  (interpolate(TESLA_LOOKUP_ANGLE_RATE_DOWN, vehicle_speed) * TESLA_DEG_TO_CAN);
      int delta_angle_down = (int)(delta_angle_float) + 1;
      int highest_desired_angle = desired_angle_last + ((desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
      int lowest_desired_angle = desired_angle_last - ((desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

      violation |= max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle);
    }
    desired_angle_last = desired_angle;

    if (!controls_allowed && ((desired_angle < (angle_meas.min - 1)) || (desired_angle > (angle_meas.max + 1)))) {
      violation = true;
    }

    if (!controls_allowed && steer_control_enabled) {
      violation = true;
    }
  }

  if (!tesla_powertrain && (addr == 0x45)) {
    int control_lever_status = (GET_BYTE(to_send, 0) & 0x3FU);
    if (control_lever_status != 1) {
      violation = true;
    }
  }

  if (addr == (tesla_powertrain ? 0x2bf : 0x2b9)) {
    if (tesla_longitudinal) {
      int aeb_event = GET_BYTE(to_send, 2) & 0x03U;
      if (aeb_event != 0) {
        violation = true;
      }

      if (tesla_stock_aeb) {
        violation = true;
      }

      int raw_accel_max = ((GET_BYTE(to_send, 6) & 0x1FU) << 4) | (GET_BYTE(to_send, 5) >> 4);
      int raw_accel_min = ((GET_BYTE(to_send, 5) & 0x0FU) << 5) | (GET_BYTE(to_send, 4) >> 3);
      float accel_max = (0.04 * raw_accel_max) - 15;
      float accel_min = (0.04 * raw_accel_min) - 15;

      if ((accel_max > TESLA_MAX_ACCEL) || (accel_min > TESLA_MAX_ACCEL)) {
        violation = true;
      }

      if ((accel_max < TESLA_MIN_ACCEL) || (accel_min < TESLA_MIN_ACCEL)) {
        violation = true;
      }

      if (!longitudinal_allowed) {
        if ((raw_accel_max != TESLA_NO_ACCEL_VALUE) || (raw_accel_min != TESLA_NO_ACCEL_VALUE)) {
          violation = true;
        }
      }
    } else {
      violation = true;
    }
  }

  if (violation) {
    tx = 0;
  }

  return tx;
}

static int tesla_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  if (tesla_model3_y) {
    if (bus_num == 0) {
      bus_fwd = 2;  // party -> autopilot
    }

    if (bus_num == 2) {
      bool block_msg = false;

      if (!tesla_my_summon) {
        if (addr == 0x27d) {
          block_msg = true;
        }
        if ((addr == 0x488) && !tesla_my_stock_steering_control) {
          block_msg = true;
        }
        if (tesla_my_longitudinal && (addr == 0x2b9) && !tesla_stock_aeb) {
          block_msg = true;
        }
      }

      if (!block_msg) {
        bus_fwd = 0;
      }
    }

    return bus_fwd;
  }

  if (bus_num == 0) {
    // Chassis/PT to autopilot
    bus_fwd = 2;
  }

  if (bus_num == 2) {
    // Autopilot to chassis/PT
    int das_control_addr = (tesla_powertrain ? 0x2bf : 0x2b9);

    bool block_msg = false;
    if (!tesla_powertrain && (addr == 0x488)) {
      block_msg = true;
    }

    if (tesla_longitudinal && (addr == das_control_addr)) {
      // "AEB_ACTIVE"
      tesla_stock_aeb = ((GET_BYTE(to_fwd, 2) & 0x03U) == 1U);

      if (!tesla_stock_aeb) {
        block_msg = true;
      }
    }

    if (!block_msg) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static const addr_checks* tesla_init(uint16_t param) {
  tesla_powertrain = GET_FLAG(param, TESLA_FLAG_POWERTRAIN);
  tesla_longitudinal = GET_FLAG(param, TESLA_FLAG_LONGITUDINAL_CONTROL);
  tesla_model3_y = GET_FLAG(param, TESLA_FLAG_MODEL3_Y);
  tesla_my_longitudinal = GET_FLAG(param, TESLA_FLAG_MODEL3_Y_LONG_CONTROL);
  tesla_my_fsd_14 = GET_FLAG(param, TESLA_FLAG_MODEL3_Y_FSD_14);

  tesla_stock_aeb = false;
  tesla_my_stock_steering_control = false;
  tesla_my_stock_steering_control_prev = false;
  tesla_my_summon = false;
  tesla_my_summon_prev = false;

  const addr_checks *checks = &tesla_rx_checks;
  if (tesla_model3_y) {
    checks = &tesla_my_rx_checks;
  } else if (tesla_powertrain) {
    checks = &tesla_pt_rx_checks;
  } else {
  }
  return checks;
}

const safety_hooks tesla_hooks = {
  .init = tesla_init,
  .rx = tesla_rx_hook,
  .tx = tesla_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = tesla_fwd_hook,
};
