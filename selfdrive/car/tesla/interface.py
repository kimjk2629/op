#!/usr/bin/env python3
from cereal import car
from selfdrive.car import get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.tesla.values import CANBUS, TeslaFlags

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long):
    ret.carName = "tesla"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla)]

    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.radarUnavailable = True  # radar port not carried over in this initial port

    ret.mass = 2072.
    ret.wheelbase = 2.890
    ret.centerToFront = ret.wheelbase * 0.44
    ret.steerRatio = 12.0

    # Model X / early HW2.5 vehicles are missing DAS_settings (0x293 / 659)
    if 659 not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.MISSING_DAS_SETTINGS.value

    # openpilot longitudinal is left OFF by default for this initial port; stock ACC stays
    # in charge and only a cancel request is forwarded. Flip this only after bench/road
    # testing the longitudinal path in isolation.
    ret.openpilotLongitudinalControl = experimental_long
    if ret.openpilotLongitudinalControl:
      ret.flags |= TeslaFlags.LONG_CONTROL.value
      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    return ret
