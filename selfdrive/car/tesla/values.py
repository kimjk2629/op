from collections import namedtuple
from typing import Dict, List, Union

from selfdrive.car import dbc_dict
from selfdrive.car.docs_definitions import CarInfo
from cereal import car

Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])
AngleRateLimit = namedtuple('AngleRateLimit', ['speed_points', 'max_angle_diff_points'])

Ecu = car.CarParams.Ecu


class CAR:
  AP1_MODELS = "TESLA AP1 MODEL S"
  AP2_MODELS = "TESLA AP2 MODEL S"
  # HW3/HW4 "party bus" integration (community port from carrot-wip, unverified on this fork)
  TESLA_MODEL_Y = "TESLA MODEL Y"
  TESLA_MODEL_3 = "TESLA MODEL 3"
  # NOTE: quotes are double (not single) on purpose -- launch_chffrplus.sh generates
  # /data/params/d/CarList (the manual car-selector dropdown's data source) by grepping
  # values.py files for the literal pattern `= "`. Single-quoted strings are invisible to
  # that grep and silently never appear in the dropdown, regardless of what's in CAR_INFO.


CAR_INFO: Dict[str, Union[CarInfo, List[CarInfo]]] = {
  CAR.AP1_MODELS: CarInfo("Tesla AP1 Model S", "All"),
  CAR.AP2_MODELS: CarInfo("Tesla AP2 Model S", "All"),
  CAR.TESLA_MODEL_Y: CarInfo("Tesla Model Y (HW3/HW4, EXPERIMENTAL/UNVERIFIED port)", "All"),
  CAR.TESLA_MODEL_3: CarInfo("Tesla Model 3 (HW3/HW4, EXPERIMENTAL/UNVERIFIED port)", "All"),
}

FINGERPRINTS = {
  CAR.AP2_MODELS: [
    {
      1: 8, 3: 8, 14: 8, 21: 4, 69: 8, 109: 4, 257: 3, 264: 8, 277: 6, 280: 6, 293: 4, 296: 4, 309: 5, 325: 8, 328: 5, 336: 8, 341: 8, 360: 7, 373: 8, 389: 8, 415: 8, 513: 5, 516: 8, 518: 8, 520: 4, 522: 8, 524: 8, 526: 8, 532: 3, 536: 8, 537: 3, 538: 8, 542: 8, 551: 5, 552: 2, 556: 8, 558: 8, 568: 8, 569: 8, 574: 8, 576: 3, 577: 8, 582: 5, 583: 8, 584: 4, 585: 8, 590: 8, 601: 8, 606: 8, 608: 1, 622: 8, 627: 6, 638: 8, 641: 8, 643: 8, 692: 8, 693: 8, 695: 8, 696: 8, 697: 8, 699: 8, 700: 8, 701: 8, 702: 8, 703: 8, 704: 8, 708: 8, 709: 8, 710: 8, 711: 8, 712: 8, 728: 8, 744: 8, 760: 8, 772: 8, 775: 8, 776: 8, 777: 8, 778: 8, 782: 8, 788: 8, 791: 8, 792: 8, 796: 2, 797: 8, 798: 6, 799: 8, 804: 8, 805: 8, 807: 8, 808: 1, 811: 8, 812: 8, 813: 8, 814: 5, 815: 8, 820: 8, 823: 8, 824: 8, 829: 8, 830: 5, 836: 8, 840: 8, 845: 8, 846: 5, 848: 8, 852: 8, 853: 8, 856: 4, 857: 6, 861: 8, 862: 5, 872: 8, 876: 8, 877: 8, 879: 8, 880: 8, 882: 8, 884: 8, 888: 8, 893: 8, 894: 8, 901: 6, 904: 3, 905: 8, 906: 8, 908: 2, 909: 8, 910: 8, 912: 8, 920: 8, 921: 8, 925: 4, 926: 6, 936: 8, 941: 8, 949: 8, 952: 8, 953: 6, 968: 8, 969: 6, 970: 8, 971: 8, 977: 8, 984: 8, 987: 8, 990: 8, 1000: 8, 1001: 8, 1006: 8, 1007: 8, 1008: 8, 1010: 6, 1014: 1, 1015: 8, 1016: 8, 1017: 8, 1018: 8, 1020: 8, 1026: 8, 1028: 8, 1029: 8, 1030: 8, 1032: 1, 1033: 1, 1034: 8, 1048: 1, 1049: 8, 1061: 8, 1064: 8, 1065: 8, 1070: 8, 1080: 8, 1081: 8, 1097: 8, 1113: 8, 1129: 8, 1145: 8, 1160: 4, 1177: 8, 1281: 8, 1328: 8, 1329: 8, 1332: 8, 1335: 8, 1337: 8, 1353: 8, 1368: 8, 1412: 8, 1436: 8, 1476: 8, 1481: 8, 1497: 8, 1513: 8, 1519: 8, 1601: 8, 1605: 8, 1617: 8, 1621: 8, 1625: 8, 1800: 4, 1804: 8, 1812: 8, 1815: 8, 1816: 8, 1824: 8, 1828: 8, 1831: 8, 1832: 8, 1840: 8, 1848: 8, 1864: 8, 1880: 8, 1892: 8, 1896: 8, 1912: 8, 1960: 8, 1992: 8, 2008: 3, 2015: 8, 2043: 5, 2045: 4
    },
  ],
  CAR.AP1_MODELS: [
    {
      1: 8, 3: 8, 14: 8, 21: 4, 69: 8, 109: 4, 257: 3, 264: 8, 267: 5, 277: 6, 280: 6, 283: 5, 293: 4, 296: 4, 309: 5, 325: 8, 328: 5, 336: 8, 341: 8, 360: 7, 373: 8, 389: 8, 415: 8, 513: 5, 516: 8, 520: 4, 522: 8, 524: 8, 526: 8, 532: 3, 536: 8, 537: 3, 542: 8, 551: 5, 552: 2, 556: 8, 558: 8, 568: 8, 569: 8, 574: 8, 577: 8, 582: 5, 584: 4, 585: 8, 590: 8, 606: 8, 622: 8, 627: 6, 638: 8, 641: 8, 643: 8, 660: 5, 693: 8, 696: 8, 697: 8, 712: 8, 728: 8, 744: 8, 760: 8, 772: 8, 775: 8, 776: 8, 777: 8, 778: 8, 782: 8, 788: 8, 791: 8, 792: 8, 796: 2, 797: 8, 798: 6, 799: 8, 804: 8, 805: 8, 807: 8, 808: 1, 809: 8, 812: 8, 813: 8, 814: 5, 815: 8, 820: 8, 823: 8, 824: 8, 829: 8, 830: 5, 836: 8, 840: 8, 841: 8, 845: 8, 846: 5, 852: 8, 856: 4, 857: 6, 861: 8, 862: 5, 872: 8, 873: 8, 877: 8, 878: 8, 879: 8, 880: 8, 884: 8, 888: 8, 889: 8, 893: 8, 896: 8, 901: 6, 904: 3, 905: 8, 908: 2, 909: 8, 920: 8, 921: 8, 925: 4, 936: 8, 937: 8, 941: 8, 949: 8, 952: 8, 953: 6, 957: 8, 968: 8, 973: 8, 984: 8, 987: 8, 989: 8, 990: 8, 1000: 8, 1001: 8, 1006: 8, 1016: 8, 1026: 8, 1028: 8, 1029: 8, 1030: 8, 1032: 1, 1033: 1, 1034: 8, 1048: 1, 1064: 8, 1070: 8, 1080: 8, 1160: 4, 1281: 8, 1329: 8, 1332: 8, 1335: 8, 1337: 8, 1368: 8, 1412: 8, 1436: 8, 1465: 8, 1476: 8, 1497: 8, 1524: 8, 1527: 8, 1601: 8, 1605: 8, 1611: 8, 1614: 8, 1617: 8, 1621: 8, 1627: 8, 1630: 8, 1800: 4, 1804: 8, 1812: 8, 1815: 8, 1816: 8, 1828: 8, 1831: 8, 1832: 8, 1840: 8, 1848: 8, 1864: 8, 1880: 8, 1892: 8, 1896: 8, 1912: 8, 1960: 8, 1992: 8, 2008: 3, 2043: 5, 2045: 4
    },
  ],
  # NOTE: Model Y/3 (HW3/HW4) are identified via FW query (see FW_VERSIONS below), not static
  # CAN fingerprinting -- there is no reliable fixed message-count signature for these platforms.
}

# ECU firmware versions used to identify TESLA_MODEL_Y / TESLA_MODEL_3 via FW query, and to
# detect "FSD 14" firmware which flips the DAS_steeringControlType enum (see TeslaFlags.FSD_14).
# Copied from carrot-wip (opendbc/car/tesla/fingerprints.py) on 2026-08-21; NOT independently
# verified against a real vehicle in this fork.
FW_VERSIONS = {
  CAR.TESLA_MODEL_3: {
    (Ecu.eps, 0x730, None): [
      b'TeM3_E014p10_0.0.0 (16),E014.17.00',
      b'TeM3_E014p10_0.0.0 (16),EL014.17.00',
      b'TeM3_ES014p11_0.0.0 (25),ES014.19.0',
      b'TeM3_E014p10_0.0.0 (24),E014.20.2',
      b'TeMYG4_DCS_Update_0.0.0 (13),E4014.28.1',
      b'TeMYG4_DCS_Update_0.0.0 (9),E4014.26.0',
      b'TeMYG4_Legacy3Y_0.0.0 (2),E4015.02.0',
      b'TeMYG4_Legacy3Y_0.0.0 (5),E4015.03.2',
      b'TeMYG4_Legacy3Y_0.0.0 (5),E4L015.03.2',
      b'TeMYG4_Main_0.0.0 (59),E4H014.29.0',
      b'TeMYG4_Main_0.0.0 (65),E4H015.01.0',
      b'TeMYG4_Main_0.0.0 (67),E4H015.02.1',
      b'TeMYG4_Main_0.0.0 (77),E4H015.04.5',
      b'TeMYG4_Main_0.0.0 (77),E4HP015.04.5',
      b'TeMYG4_Main_0.0.0 (78),E4HP015.05.0',
      b'TeMYG4_SingleECU_0.0.0 (33),E4S014.27',
      b'TeMYG4_Main_0.0.0 (78),E4H015.05.0',
    ],
  },
  CAR.TESLA_MODEL_Y: {
    (Ecu.eps, 0x730, None): [
      b'TeM3_E014p10_0.0.0 (16),Y002.18.00',
      b'TeM3_E014p10_0.0.0 (16),YP002.18.00',
      b'TeM3_E014p10_0.0.0 (24),YP002.21.2',
      b'TeM3_ES014p11_0.0.0 (16),YS002.17',
      b'TeM3_ES014p11_0.0.0 (25),YS002.19.0',
      b'TeMYG4_DCS_Update_0.0.0 (13),Y4002.27.1',
      b'TeMYG4_DCS_Update_0.0.0 (13),Y4P002.27.1',
      b'TeMYG4_DCS_Update_0.0.0 (9),Y4P002.25.0',
      b'TeMYG4_Legacy3Y_0.0.0 (2),Y4003.02.0',
      b'TeMYG4_Legacy3Y_0.0.0 (2),Y4P003.02.0',
      b'TeMYG4_Legacy3Y_0.0.0 (5),Y4003.03.2',
      b'TeMYG4_Legacy3Y_0.0.0 (5),Y4P003.03.2',
      b'TeMYG4_Legacy3Y_0.0.0 (6),Y4003.04.0',
      b'TeMYG4_Main_0.0.0 (77),Y4003.05.4',
      b'TeMYG4_Main_0.0.0 (78),Y4003.06.0',
      b'TeMYG4_SingleECU_0.0.0 (28),Y4S002.23.0',
      b'TeMYG4_SingleECU_0.0.0 (33),Y4S002.26',
    ],
  },
}

# Cars with this EPS FW have FSD 14 and use TeslaFlags.FSD_14 (steering control type enum flip)
FSD_14_FW = {
  CAR.TESLA_MODEL_3: [
    b'TeMYG4_Main_0.0.0 (77),E4HP015.04.5',
    b'TeMYG4_Main_0.0.0 (78),E4HP015.05.0',
  ],
  CAR.TESLA_MODEL_Y: [
    b'TeMYG4_Legacy3Y_0.0.0 (6),Y4003.04.0',
    b'TeMYG4_Main_0.0.0 (77),Y4003.05.4',
  ],
}

DBC = {
  CAR.AP2_MODELS: dbc_dict('tesla_powertrain', 'tesla_radar', chassis_dbc='tesla_can'),
  CAR.AP1_MODELS: dbc_dict('tesla_powertrain', 'tesla_radar', chassis_dbc='tesla_can'),
  # party-bus DBC doubles as both the "pt"/chassis DBC and the radar DBC key here is unused;
  # radar_interface.py looks up DBC[candidate]['party'] directly for these two cars.
  CAR.TESLA_MODEL_Y: {'party': 'tesla_model3_party', 'radar': 'tesla_radar_continental_generated'},
  CAR.TESLA_MODEL_3: {'party': 'tesla_model3_party', 'radar': 'tesla_radar_continental_generated'},
}


class CANBUS:
  # AP1/AP2 full-bypass harness
  chassis = 0
  radar = 1
  autopilot_chassis = 2

  powertrain = 4
  private = 5
  autopilot_powertrain = 6

  # HW3/HW4 "party bus" harness (Tesla A / Tesla B comma harness) -- DIFFERENT harness hardware
  # from the AP1/AP2 harness above. bus indices below are only meaningful when that harness is used.
  party = 0
  autopilot_party = 2
  vehicle = 1


GEAR_MAP = {
  "DI_GEAR_INVALID": car.CarState.GearShifter.unknown,
  "DI_GEAR_P": car.CarState.GearShifter.park,
  "DI_GEAR_R": car.CarState.GearShifter.reverse,
  "DI_GEAR_N": car.CarState.GearShifter.neutral,
  "DI_GEAR_D": car.CarState.GearShifter.drive,
  "DI_GEAR_SNA": car.CarState.GearShifter.unknown,
}

DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]

# Make sure the message and addr is also in the CAN parser!
BUTTONS = [
  Button(car.CarState.ButtonEvent.Type.leftBlinker, "STW_ACTN_RQ", "TurnIndLvr_Stat", [1]),
  Button(car.CarState.ButtonEvent.Type.rightBlinker, "STW_ACTN_RQ", "TurnIndLvr_Stat", [2]),
  Button(car.CarState.ButtonEvent.Type.accelCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [4, 16]),
  Button(car.CarState.ButtonEvent.Type.decelCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [8, 32]),
  Button(car.CarState.ButtonEvent.Type.cancel, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [2]),
  Button(car.CarState.ButtonEvent.Type.resumeCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [1]),
]

STEER_THRESHOLD = 1


class TeslaFlags:
  # bit flags stored in CarParams.flags for TESLA_MODEL_Y / TESLA_MODEL_3 (party-bus cars only)
  LONG_CONTROL = 1
  FSD_14 = 2


class CarControllerParams:
  """AP1 / AP2 Model S (full-bypass harness) -- unchanged from upstream."""
  RATE_LIMIT_UP = AngleRateLimit(speed_points=[0., 5., 15.], max_angle_diff_points=[5., .8, .15])
  RATE_LIMIT_DOWN = AngleRateLimit(speed_points=[0., 5., 15.], max_angle_diff_points=[5., 3.5, 0.4])
  JERK_LIMIT_MAX = 8
  JERK_LIMIT_MIN = -8
  ACCEL_TO_SPEED_MULTIPLIER = 3


class CarControllerParamsMY:
  """Model Y / Model 3 (HW3/HW4 party-bus harness). Rate tables copied from carrot-wip's
  AngleSteeringLimits (opendbc/car/tesla/values.py); PANDA SAFETY IS THE REAL ENFORCEMENT --
  these are just the values the controller aims for."""
  RATE_LIMIT_UP = AngleRateLimit(speed_points=[0., 5., 25.], max_angle_diff_points=[2.5, 1.5, 0.2])
  RATE_LIMIT_DOWN = AngleRateLimit(speed_points=[0., 5., 25.], max_angle_diff_points=[5.0, 2.0, 0.3])
  STEER_STEP = 2       # angle command sent at 50 Hz
  ACCEL_MAX = 2.0       # m/s^2
  ACCEL_MIN = -3.48     # m/s^2
  JERK_LIMIT_MAX = 4.9  # m/s^3, ACC faults at 5.0
  JERK_LIMIT_MIN = -4.9
  JERK_UP = 1.0
