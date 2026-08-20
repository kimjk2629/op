from selfdrive.car import dbc_dict, AngleRateLimit
from cereal import car
from enum import IntFlag

Ecu = car.CarParams.Ecu


class CAR:
  TESLA_MODEL_Y = "TESLA MODEL Y"


class CANBUS:
  party = 0           # main vehicle bus (DI_*, ESP_*, EPAS3S_*, UI_*)
  vehicle = 1          # secondary vehicle bus (nav blinker MITM, optional)
  autopilot_party = 2  # DAS_* (stock Autopilot) bus


DBC = {
  CAR.TESLA_MODEL_Y: dbc_dict('tesla_model3_party', None),
}

FINGERPRINTS = {
  # Minimal manual fingerprint. carrot-wip's newer FW-query based auto fingerprinting
  # was not ported here — verify/extend this against a real fingerprint capture before use.
  CAR.TESLA_MODEL_Y: [{
    280: 8, 264: 8, 325: 8, 373: 8, 599: 8, 646: 8, 785: 7, 880: 8, 297: 8,
  }],
}

GEAR_MAP = {
  "DI_GEAR_INVALID": car.CarState.GearShifter.unknown,
  "DI_GEAR_P": car.CarState.GearShifter.park,
  "DI_GEAR_R": car.CarState.GearShifter.reverse,
  "DI_GEAR_N": car.CarState.GearShifter.neutral,
  "DI_GEAR_D": car.CarState.GearShifter.drive,
  "DI_GEAR_SNA": car.CarState.GearShifter.unknown,
}

STEER_THRESHOLD = 1


class TeslaFlags(IntFlag):
  LONG_CONTROL = 1
  MISSING_DAS_SETTINGS = 2


class CarControllerParams:
  # Angle-based steering (carrot-wip's AngleSteeringLimits, translated to the
  # old AngleRateLimit(speed_bp, angle_v) + scalar cap style used by
  # apply_std_steer_angle_limits() in selfdrive/car/__init__.py)
  ANGLE_LIMIT = 360.  # deg, EPAS faults above this
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 25.], angle_v=[2.5, 1.5, 0.2])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 25.], angle_v=[5., 2.0, 0.3])

  STEER_STEP = 2       # steering command sent at 50 Hz (100 / STEER_STEP)
  ACCEL_MAX = 2.0       # m/s^2
  ACCEL_MIN = -3.48     # m/s^2
  JERK_LIMIT_MAX = 4.9  # m/s^3, ACC faults at 5.0
  JERK_LIMIT_MIN = -4.9
  JERK_UP = 1.0

  def __init__(self, CP):
    pass
