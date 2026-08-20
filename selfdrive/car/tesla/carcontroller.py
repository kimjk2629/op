from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_angle_limits
from selfdrive.car.tesla.teslacan import TeslaCAN
from selfdrive.car.tesla.values import CarControllerParams


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.apply_angle_last = 0

    self.packer = CANPacker(dbc_name)
    self.tesla_can = TeslaCAN(self.packer)

  def update(self, CC, CS):
    actuators = CC.actuators
    can_sends = []

    # Tesla's EPS disables steering assist above a hands-on-wheel torque threshold.
    # CS.hands_on_level >= 3 mirrors the panda safety cutoff.
    lat_active = CC.latActive and CS.hands_on_level < 3

    if self.frame % CarControllerParams.STEER_STEP == 0:
      self.apply_angle_last = apply_std_steer_angle_limits(
        actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)
      self.apply_angle_last = max(-CarControllerParams.ANGLE_LIMIT,
                                  min(CarControllerParams.ANGLE_LIMIT, self.apply_angle_last))
      can_sends.append(self.tesla_can.create_steering_control(
        self.apply_angle_last, lat_active, (self.frame // CarControllerParams.STEER_STEP) % 16))

    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        cancel = CC.cruiseControl.cancel or CS.das_accCancel
        state = 13 if cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(max(CarControllerParams.ACCEL_MIN, min(CarControllerParams.ACCEL_MAX, actuators.accel)))
        if not CC.longActive:
          accel = 0.
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))
    else:
      # Stock longitudinal: still forward a cancel request so a disengage always reaches the car
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control_counter + 1) % 8 if CS.das_control_counter is not None else 0
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
