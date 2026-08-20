from cereal import car
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from common.conversions import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, TeslaFlags

GearShifter = car.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = self.can_define.dv["DI_systemStatus"]["DI_gear"]

    self.hands_on_level = 0
    self.das_accCancel = False
    self.das_control = {}
    self.das_control_counter = None

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Wheel speeds
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["ESP_wheelSpeeds"]["ESP_wheelSpeedFrL"],
      cp.vl["ESP_wheelSpeeds"]["ESP_wheelSpeedFrR"],
      cp.vl["ESP_wheelSpeeds"]["ESP_wheelSpeedReL"],
      cp.vl["ESP_wheelSpeeds"]["ESP_wheelSpeedReR"],
      unit=CV.KPH_TO_MS,
    )

    # NOTE: display-unit (kph/mph) inference from the DBC enum was not ported;
    # this always treats the cluster readout as kph. Fine for KR-market cars,
    # but must be revisited for mph-region vehicles.
    ret.vEgoCluster = cp.vl["DI_speed"]["DI_uiSpeed"] * CV.KPH_TO_MS

    # Gas pedal
    pedal_status = cp.vl["DI_systemStatus"]["DI_accelPedalPos"]
    ret.gas = pedal_status / 100.0
    ret.gasPressed = pedal_status > 0

    # Brake
    ret.brakePressed = cp.vl["ESP_status"]["ESP_driverBrakeApply"] == 2
    ret.brakeLights = cp.vl["ESP_status"]["ESP_brakeLamp"] == 1
    ret.regenBraking = cp.vl["DI_systemStatus"]["DI_regenLight"] != 0
    ret.espDisabled = cp.vl["ESP_status"]["ESP_espFaultLamp"] != 0

    # Steering wheel
    epas_status = cp.vl["EPAS3S_sysStatus"]
    self.hands_on_level = epas_status["EPAS3S_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS3S_internalSAS"]
    ret.steeringRateDeg = -cp.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleSpeed"]
    ret.steeringTorque = -epas_status["EPAS3S_torsionBarTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacStatus"].get(int(epas_status["EPAS3S_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp.vl["DI_state"]["DI_cruiseState"]), None)
    acc_state = cp_cam.vl["DAS_control"]["DAS_accState"]
    self.das_accCancel = acc_state in (0, 1, 2, 12, 13, 14, 15)
    self.das_control = dict(cp_cam.vl["DAS_control"])
    self.das_control_counter = int(cp_cam.vl["DAS_control"]["DAS_controlCounter"])

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    ret.cruiseState.enabled = cruise_enabled
    ret.cruiseState.speed = max(cp.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or cruise_enabled
    ret.cruiseState.standstill = False
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    park_brake_state = self.can_define.dv["DI_state"]["DI_parkBrakeState"].get(int(cp.vl["DI_state"]["DI_parkBrakeState"]), None)
    ret.parkingBrake = park_brake_state == "APPLIED"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors / blinkers / misc
    ret.doorOpen = cp.vl["UI_warning"]["anyDoorOpen"] == 1
    ret.leftBlinker = cp.vl["UI_warning"]["leftBlinkerBlinking"] in (1, 2)
    ret.rightBlinker = cp.vl["UI_warning"]["rightBlinkerBlinking"] in (1, 2)
    ret.genericToggle = cp.vl["UI_warning"]["highBeam"] == 1
    ret.seatbeltUnlatched = cp.vl["UI_warning"]["buckleStatus"] != 1

    # Stock safety systems
    ret.stockAeb = cp_cam.vl["DAS_control"]["DAS_aebEvent"] == 1
    ret.stockFcw = cp_cam.vl["DAS_status"]["DAS_forwardCollisionWarning"] != 0

    # Stock Autosteer must be off for openpilot LKAS to be valid
    if not (self.CP.flags & TeslaFlags.MISSING_DAS_SETTINGS):
      ret.invalidLkasSetting = cp_cam.vl["DAS_status"]["DAS_autopilotState"] not in (0, 1, 2)

    ret.canValid = True
    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      ("DI_vehicleSpeed", "DI_speed"),
      ("DI_uiSpeed", "DI_speed"),
      ("DI_uiSpeedUnits", "DI_speed"),

      ("ESP_wheelSpeedFrL", "ESP_wheelSpeeds"),
      ("ESP_wheelSpeedFrR", "ESP_wheelSpeeds"),
      ("ESP_wheelSpeedReL", "ESP_wheelSpeeds"),
      ("ESP_wheelSpeedReR", "ESP_wheelSpeeds"),

      ("DI_accelPedalPos", "DI_systemStatus"),
      ("DI_regenLight", "DI_systemStatus"),
      ("DI_gear", "DI_systemStatus"),

      ("ESP_driverBrakeApply", "ESP_status"),
      ("ESP_brakeLamp", "ESP_status"),
      ("ESP_espFaultLamp", "ESP_status"),

      ("EPAS3S_handsOnLevel", "EPAS3S_sysStatus"),
      ("EPAS3S_internalSAS", "EPAS3S_sysStatus"),
      ("EPAS3S_torsionBarTorque", "EPAS3S_sysStatus"),
      ("EPAS3S_eacStatus", "EPAS3S_sysStatus"),
      ("EPAS3S_eacErrorCode", "EPAS3S_sysStatus"),

      ("SCCM_steeringAngleSpeed", "SCCM_steeringAngleSensor"),

      ("DI_cruiseState", "DI_state"),
      ("DI_digitalSpeed", "DI_state"),
      ("DI_parkBrakeState", "DI_state"),
      ("DI_speedUnits", "DI_state"),

      ("anyDoorOpen", "UI_warning"),
      ("leftBlinkerBlinking", "UI_warning"),
      ("rightBlinkerBlinking", "UI_warning"),
      ("highBeam", "UI_warning"),
      ("buckleStatus", "UI_warning"),
    ]

    checks = [
      ("DI_speed", 50),
      ("ESP_wheelSpeeds", 50),
      ("DI_systemStatus", 10),
      ("ESP_status", 25),
      ("EPAS3S_sysStatus", 100),
      ("SCCM_steeringAngleSensor", 67),
      ("DI_state", 10),
      ("UI_warning", 10),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CANBUS.party)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      ("DAS_accState", "DAS_control"),
      ("DAS_aebEvent", "DAS_control"),
      ("DAS_controlCounter", "DAS_control"),

      ("DAS_forwardCollisionWarning", "DAS_status"),
      ("DAS_autopilotState", "DAS_status"),
      ("DAS_fusedSpeedLimit", "DAS_status"),
      ("DAS_blindSpotRearLeft", "DAS_status"),
      ("DAS_blindSpotRearRight", "DAS_status"),
    ]

    checks = [
      ("DAS_control", 25),
      ("DAS_status", 2),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CANBUS.autopilot_party, enforce_checks=False)
