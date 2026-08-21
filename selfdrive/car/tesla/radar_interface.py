#!/usr/bin/env python3
from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.tesla.values import DBC, CANBUS, CAR
from selfdrive.car.interfaces import RadarInterfaceBase

MODEL_Y_3 = (CAR.TESLA_MODEL_Y, CAR.TESLA_MODEL_3)

# ------------------------------------------------------------------
# AP1 / AP2 Model S (full-bypass harness) radar layout -- unchanged from upstream
# ------------------------------------------------------------------
RADAR_MSGS_A = list(range(0x310, 0x36E, 3))
RADAR_MSGS_B = list(range(0x311, 0x36F, 3))
NUM_POINTS = len(RADAR_MSGS_A)

# ------------------------------------------------------------------
# Model Y / Model 3 (Continental ARS4-B radar, only present on a shrinking subset of
# HW2.5/early-HW3 cars -- most Model Y sold since ~2021 are vision-only and this parser will
# simply never see traffic, which is fine). Ported from carrot-wip on 2026-08-21.
# ------------------------------------------------------------------
RADAR_START_ADDR = 0x410
RADAR_MSG_COUNT = 80  # 40 points * 2 messages each


def get_radar_can_parser(CP):
  if CP.carFingerprint in MODEL_Y_3:
    if 'radar' not in DBC[CP.carFingerprint]:
      return None

    signals = [
      ('shortTermUnavailable', 'RadarStatus'),
      ('sensorBlocked', 'RadarStatus'),
      ('vehDynamicsError', 'RadarStatus'),
    ]
    checks = [('RadarStatus', 16)]

    for i in range(RADAR_MSG_COUNT // 2):
      msg_a = f'RadarPoint{i}_A'
      msg_b = f'RadarPoint{i}_B'
      signals.extend([
        ('LongDist', msg_a),
        ('LongSpeed', msg_a),
        ('LatDist', msg_a),
        ('LongAccel', msg_a),
        ('Meas', msg_a),
        ('Tracked', msg_a),
        ('Index', msg_a),
        ('LatSpeed', msg_b),
        ('Index2', msg_b),
      ])
      checks.extend([(msg_a, 16), (msg_b, 16)])

    return CANParser(DBC[CP.carFingerprint]['radar'], signals, checks, CANBUS.autopilot_party)

  # Status messages
  signals = [
    ('RADC_HWFail', 'TeslaRadarSguInfo'),
    ('RADC_SGUFail', 'TeslaRadarSguInfo'),
    ('RADC_SensorDirty', 'TeslaRadarSguInfo'),
  ]

  checks = [
    ('TeslaRadarSguInfo', 10),
  ]

  # Radar tracks. There are also raw point clouds available,
  # we don't use those.
  for i in range(NUM_POINTS):
    msg_id_a = RADAR_MSGS_A[i]
    msg_id_b = RADAR_MSGS_B[i]

    signals.extend([
      ('LongDist', msg_id_a),
      ('LongSpeed', msg_id_a),
      ('LatDist', msg_id_a),
      ('LongAccel', msg_id_a),
      ('Meas', msg_id_a),
      ('Tracked', msg_id_a),
      ('Index', msg_id_a),

      ('LatSpeed', msg_id_b),
      ('Index2', msg_id_b),
    ])

    checks.extend([
      (msg_id_a, 8),
      (msg_id_b, 8),
    ])

  return CANParser(DBC[CP.carFingerprint]['radar'], signals, checks, CANBUS.radar)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.is_model_y_3 = CP.carFingerprint in MODEL_Y_3
    self.rcp = get_radar_can_parser(CP)
    self.updated_messages = set()
    self.track_id = 0

    if self.is_model_y_3:
      self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    else:
      self.trigger_msg = RADAR_MSGS_B[-1]

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    values = self.rcp.update_strings(can_strings)
    self.updated_messages.update(values)

    if self.trigger_msg not in self.updated_messages:
      return None

    if self.is_model_y_3:
      ret = self._update_model_y_3()
    else:
      ret = self._update_ap1_ap2()

    self.updated_messages.clear()
    return ret

  def _update_ap1_ap2(self):
    ret = car.RadarData.new_message()

    errors = []
    sgu_info = self.rcp.vl['TeslaRadarSguInfo']
    if not self.rcp.can_valid:
      errors.append('canError')
    if sgu_info['RADC_HWFail'] or sgu_info['RADC_SGUFail'] or sgu_info['RADC_SensorDirty']:
      errors.append('fault')
    ret.errors = errors

    for i in range(NUM_POINTS):
      msg_a = self.rcp.vl[RADAR_MSGS_A[i]]
      msg_b = self.rcp.vl[RADAR_MSGS_B[i]]

      if msg_a['Index'] != msg_b['Index2']:
        continue

      if not msg_a['Tracked']:
        if i in self.pts:
          del self.pts[i]
        continue

      if i not in self.pts:
        self.pts[i] = car.RadarData.RadarPoint.new_message()
        self.pts[i].trackId = self.track_id
        self.track_id += 1

      self.pts[i].dRel = msg_a['LongDist']
      self.pts[i].yRel = msg_a['LatDist']
      self.pts[i].vRel = msg_a['LongSpeed']
      self.pts[i].aRel = msg_a['LongAccel']
      self.pts[i].yvRel = msg_b['LatSpeed']
      self.pts[i].measured = bool(msg_a['Meas'])

    ret.points = list(self.pts.values())
    return ret

  def _update_model_y_3(self):
    ret = car.RadarData.new_message()

    errors = []
    if not self.rcp.can_valid:
      errors.append('canError')
    radar_status = self.rcp.vl['RadarStatus']
    if radar_status['shortTermUnavailable']:
      errors.append('fault')
    if radar_status['sensorBlocked'] or radar_status['vehDynamicsError']:
      errors.append('fault')
    ret.errors = errors

    for i in range(RADAR_MSG_COUNT // 2):
      msg_a = self.rcp.vl[f'RadarPoint{i}_A']
      msg_b = self.rcp.vl[f'RadarPoint{i}_B']

      if msg_a['Index'] != msg_b['Index2']:
        continue

      if not msg_a['Tracked']:
        if i in self.pts:
          del self.pts[i]
        continue

      if i not in self.pts:
        self.pts[i] = car.RadarData.RadarPoint.new_message()
        self.pts[i].trackId = self.track_id
        self.track_id += 1

      self.pts[i].dRel = msg_a['LongDist']
      self.pts[i].yRel = msg_a['LatDist']
      self.pts[i].vRel = msg_a['LongSpeed']
      self.pts[i].aRel = msg_a['LongAccel']
      self.pts[i].yvRel = msg_b['LatSpeed']
      self.pts[i].measured = bool(msg_a['Meas'])

    ret.points = list(self.pts.values())
    return ret
