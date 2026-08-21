from common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from selfdrive.car.tesla.teslacan import TeslaCAN, TeslaCANModelY
from selfdrive.car.tesla.values import DBC, CANBUS, CAR, CarControllerParams, CarControllerParamsMY

MODEL_Y_3 = (CAR.TESLA_MODEL_Y, CAR.TESLA_MODEL_3)


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.is_model_y_3 = CP.carFingerprint in MODEL_Y_3
    self.last_angle = 0
    self.long_control_counter = 0
    self.frame = 0

    if self.is_model_y_3:
      self.packer = CANPacker(dbc_name)
      self.tesla_can = TeslaCANModelY(CP, self.packer)
    else:
      self.packer = CANPacker(dbc_name)
      self.pt_packer = CANPacker(DBC[CP.carFingerprint]['pt'])
      self.tesla_can = TeslaCAN(self.packer, self.pt_packer)

  def update(self, c, CS, frame, actuators, cruise_cancel):
    if self.is_model_y_3:
      return self.update_model_y_3(c, CS, frame, actuators, cruise_cancel)
    return self.update_ap1_ap2(c, CS, frame, actuators, cruise_cancel)

  # ------------------------------------------------------------------
  # AP1 / AP2 Model S (full-bypass harness) -- unchanged from upstream
  # ------------------------------------------------------------------
  def update_ap1_ap2(self, c, CS, frame, actuators, cruise_cancel):
    can_sends = []

    # Temp disable steering on a hands_on_fault, and allow for user override
    hands_on_fault = (CS.steer_warning == "EAC_ERROR_HANDS_ON" and CS.hands_on_level >= 3)
    lkas_enabled = c.latActive and (not hands_on_fault)

    if lkas_enabled:
      apply_angle = actuators.steeringAngleDeg

      # Angular rate limit based on speed
      steer_up = (self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle))
      rate_limit = CarControllerParams.RATE_LIMIT_UP if steer_up else CarControllerParams.RATE_LIMIT_DOWN
      max_angle_diff = interp(CS.out.vEgo, rate_limit.speed_points, rate_limit.max_angle_diff_points)
      apply_angle = clip(apply_angle, (self.last_angle - max_angle_diff), (self.last_angle + max_angle_diff))

      # To not fault the EPS
      apply_angle = clip(apply_angle, (CS.out.steeringAngleDeg - 20), (CS.out.steeringAngleDeg + 20))
    else:
      apply_angle = CS.out.steeringAngleDeg

    self.last_angle = apply_angle
    can_sends.append(self.tesla_can.create_steering_control(apply_angle, lkas_enabled, frame))

    # Longitudinal control (40Hz)
    if self.CP.openpilotLongitudinalControl and ((frame % 5) in (0, 2)):
      target_accel = actuators.accel
      target_speed = max(CS.out.vEgo + (target_accel * CarControllerParams.ACCEL_TO_SPEED_MULTIPLIER), 0)
      max_accel = 0 if target_accel < 0 else target_accel
      min_accel = 0 if target_accel > 0 else target_accel

      can_sends.extend(self.tesla_can.create_longitudinal_commands(CS.acc_state, target_speed, min_accel, max_accel, self.long_control_counter))
      self.long_control_counter += 1

    # Cancel on user steering override, since there is no steering torque blending
    if hands_on_fault:
      cruise_cancel = True

    if ((frame % 10) == 0 and cruise_cancel):
      # Spam every possible counter value, otherwise it might not be accepted
      for counter in range(16):
        can_sends.append(self.tesla_can.create_action_request(CS.msg_stw_actn_req, cruise_cancel, CANBUS.chassis, counter))
        can_sends.append(self.tesla_can.create_action_request(CS.msg_stw_actn_req, cruise_cancel, CANBUS.autopilot_chassis, counter))

    # TODO: HUD control

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    return new_actuators, can_sends

  # ------------------------------------------------------------------
  # Model Y / Model 3, HW3/HW4 "party bus" harness.
  # Ported from carrot-wip (opendbc/car/tesla/carcontroller.py) on 2026-08-21.
  # Simplified vs. upstream carrot-wip for this initial port: no VehicleModel-based lateral-
  # accel limiting (uses a fixed speed/angle-rate table instead, same style as this repo's
  # existing AP1/AP2 code), no "coop steering" blending, no blinker MITM. The panda safety
  # firmware (safety_tesla.h) is the actual enforcement backstop for the angle/accel limits
  # regardless of what this file computes.
  # ------------------------------------------------------------------
  def update_model_y_3(self, c, CS, frame, actuators, cruise_cancel):
    can_sends = []
    self.frame = frame

    lat_active = c.latActive and CS.hands_on_level < 3

    if frame % CarControllerParamsMY.STEER_STEP == 0:
      apply_angle = actuators.steeringAngleDeg
      steer_up = (self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle))
      rate_limit = CarControllerParamsMY.RATE_LIMIT_UP if steer_up else CarControllerParamsMY.RATE_LIMIT_DOWN
      max_angle_diff = interp(CS.out.vEgoRaw, rate_limit.speed_points, rate_limit.max_angle_diff_points)
      apply_angle = clip(apply_angle, (self.last_angle - max_angle_diff), (self.last_angle + max_angle_diff))

      if not lat_active:
        apply_angle = CS.out.steeringAngleDeg

      self.last_angle = apply_angle
      can_sends.append(self.tesla_can.create_steering_control(apply_angle, lat_active, (frame // CarControllerParamsMY.STEER_STEP) % 16))

    # Required steering-allowed heartbeat -- without this the Tesla Autopilot/FSD computer
    # will not forward our DAS_steeringControl requests onward to the EPS.
    if frame % 10 == 0:
      can_sends.append(self.tesla_can.create_steering_allowed((frame // 10) % 16))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if frame % 4 == 0:
        state = 13 if (cruise_cancel or CS.das_accCancel) else 4  # 13=ACC_CANCEL_GENERIC_SILENT, 4=ACC_ON
        accel = float(clip(actuators.accel, CarControllerParamsMY.ACCEL_MIN, CarControllerParamsMY.ACCEL_MAX))
        long_active = c.longActive and not cruise_cancel
        if not long_active:
          accel = 0.
        cntr = (frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, long_active, CS.cruise_override))
    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if cruise_cancel and CS.das_control is not None:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False, True))

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = self.last_angle

    return new_actuators, can_sends
