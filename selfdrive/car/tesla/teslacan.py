from selfdrive.car.tesla.values import CANBUS, CarControllerParams


class TeslaCAN:
  def __init__(self, packer):
    self.packer = packer
    self.l_jerk = 0.0

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, angle, enabled, counter):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,  # 1 = ANGLE_CONTROL
      "DAS_steeringControlCounter": counter,
    }
    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)[2]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, cntr, v_ego, active):
    # ACC_ON = 4, ACC_CANCEL_GENERIC_SILENT = 13
    if active:
      self.l_jerk = min(self.l_jerk + CarControllerParams.JERK_UP * (1. / 100.) * 4, CarControllerParams.JERK_LIMIT_MAX)
      set_speed = max(v_ego + accel, 0) * 3.6
    else:
      self.l_jerk = 0.0
      set_speed = v_ego * 3.6

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": self.l_jerk,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[2]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)
