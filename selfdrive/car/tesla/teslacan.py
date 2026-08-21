import copy
import crcmod
from common.conversions import Conversions as CV
from selfdrive.car.tesla.values import CANBUS, CarControllerParams, CarControllerParamsMY, TeslaFlags


class TeslaCAN:
  """AP1 / AP2 Model S (full-bypass harness). Unchanged from upstream op3t."""

  def __init__(self, packer, pt_packer):
    self.packer = packer
    self.pt_packer = pt_packer
    self.crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)

  @staticmethod
  def checksum(msg_id, dat):
    # TODO: get message ID from name instead
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, angle, enabled, frame):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
      "DAS_steeringControlCounter": (frame % 16),
    }

    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.chassis, values)[2]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.chassis, values)

  def create_action_request(self, msg_stw_actn_req, cancel, bus, counter):
    values = copy.copy(msg_stw_actn_req)

    if cancel:
      values["SpdCtrlLvr_Stat"] = 1
      values["MC_STW_ACTN_RQ"] = counter

    data = self.packer.make_can_msg("STW_ACTN_RQ", bus, values)[2]
    values["CRC_STW_ACTN_RQ"] = self.crc(data[:7])
    return self.packer.make_can_msg("STW_ACTN_RQ", bus, values)

  def create_longitudinal_commands(self, acc_state, speed, min_accel, max_accel, cnt):
    messages = []
    values = {
      "DAS_setSpeed": speed * CV.MS_TO_KPH,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": min_accel,
      "DAS_accelMax": max_accel,
      "DAS_controlCounter": (cnt % 8),
      "DAS_controlChecksum": 0,
    }

    for packer, bus in [(self.packer, CANBUS.chassis), (self.pt_packer, CANBUS.powertrain)]:
      data = packer.make_can_msg("DAS_control", bus, values)[2]
      values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
      messages.append(packer.make_can_msg("DAS_control", bus, values))
    return messages


def get_steer_ctrl_type(flags: int, ctrl_type: int) -> int:
  # FSD 14 firmware flips the ANGLE_CONTROL(1) / LANE_KEEP_ASSIST(2) enum values.
  if flags & TeslaFlags.FSD_14:
    return {1: 2, 2: 1}.get(ctrl_type, ctrl_type)
  return ctrl_type


class TeslaCANModelY:
  """Model Y / Model 3, HW3/HW4 "party bus" harness. Ported from carrot-wip
  (opendbc/car/tesla/teslacan.py) on 2026-08-21. NOT independently verified on real hardware
  in this fork -- coop_steering / blinker-MITM features were intentionally dropped for this
  initial port to keep the surface area small; see carstate.py / carcontroller.py for details.
  """

  def __init__(self, CP, packer):
    self.CP = CP
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
      "DAS_steeringControlType": get_steer_ctrl_type(self.CP.flags, 1 if enabled else 0),
      "DAS_steeringControlCounter": counter,
    }

    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)[2]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, cntr, v_ego, active, cruise_override=False):
    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    if active:
      self.l_jerk = 0 if cruise_override else (self.l_jerk + CarControllerParamsMY.JERK_UP * 0.01 * 4)
      set_speed = max(v_ego + accel, 0) * CV.MS_TO_KPH
    else:
      self.l_jerk = 0.0

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParamsMY.JERK_LIMIT_MIN,
      "DAS_jerkMax": min(self.l_jerk, CarControllerParamsMY.JERK_LIMIT_MAX),
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[2]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self, counter):
    # Required heartbeat: without this the Tesla's own Autopilot/FSD computer will not forward
    # our DAS_steeringControl requests to the EPS. Sent every 10 frames (~10 Hz) from carcontroller.
    values = {
      "APS_eacAllow": 1,
      "APS_eacMonitorCounter": counter,
    }

    data = self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)[2]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)
