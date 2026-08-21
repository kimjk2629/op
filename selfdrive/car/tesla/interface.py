#!/usr/bin/env python3
from cereal import car
from panda import Panda
from selfdrive.car.tesla.values import CANBUS, CAR, TeslaFlags, FSD_14_FW
from selfdrive.car import STD_CARGO_KG, gen_empty_fingerprint, scale_rot_inertia, scale_tire_stiffness, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase

Ecu = car.CarParams.Ecu
MODEL_Y_3 = (CAR.TESLA_MODEL_Y, CAR.TESLA_MODEL_3)


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, disable_radar=False):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "tesla"
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerLimitTimer = 1.0

    # Set kP and kI to 0 over the whole speed range to have the planner accel as actuator command
    ret.longitudinalTuning.kpBP = [0]
    ret.longitudinalTuning.kpV = [0]
    ret.longitudinalTuning.kiBP = [0]
    ret.longitudinalTuning.kiV = [0]
    ret.stopAccel = 0.0
    ret.longitudinalActuatorDelayUpperBound = 0.5  # s
    ret.radarTimeStep = (1.0 / 8)  # 8Hz

    if candidate in MODEL_Y_3:
      return CarInterface._get_params_model_y_3(ret, candidate, fingerprint, car_fw)
    return CarInterface._get_params_ap1_ap2(ret, candidate, fingerprint)

  # ------------------------------------------------------------------
  # AP1 / AP2 Model S (full-bypass harness) -- unchanged from upstream
  # ------------------------------------------------------------------
  @staticmethod
  def _get_params_ap1_ap2(ret, candidate, fingerprint):
    # There is no safe way to do steer blending with user torque,
    # so the steering behaves like autopilot. This is not
    # how openpilot should be, hence dashcamOnly
    ret.dashcamOnly = True

    # Check if we have messages on an auxiliary panda, and that 0x2bf (DAS_control) is present on the AP powertrain bus
    # If so, we assume that it is connected to the longitudinal harness.
    if (CANBUS.autopilot_powertrain in fingerprint.keys()) and (0x2bf in fingerprint[CANBUS.autopilot_powertrain].keys()):
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs = [
        get_safety_config(car.CarParams.SafetyModel.tesla, Panda.FLAG_TESLA_LONG_CONTROL),
        get_safety_config(car.CarParams.SafetyModel.tesla, Panda.FLAG_TESLA_LONG_CONTROL | Panda.FLAG_TESLA_POWERTRAIN),
      ]
    else:
      ret.openpilotLongitudinalControl = False
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla, 0)]

    ret.steerActuatorDelay = 0.25

    if candidate in (CAR.AP2_MODELS, CAR.AP1_MODELS):
      ret.mass = 2100. + STD_CARGO_KG
      ret.wheelbase = 2.959
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 15.0
    else:
      raise ValueError(f"Unsupported car: {candidate}")

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)

    return ret

  # ------------------------------------------------------------------
  # Model Y / Model 3, HW3/HW4 "party bus" harness.
  # Ported from carrot-wip (opendbc/car/tesla/interface.py) on 2026-08-21.
  # UNVERIFIED on real hardware in this fork -- ships dashcamOnly=True until independently
  # bench/road tested against the panda safety unit tests. See ../../../../TESLA_PORT_README.md
  # (repo root) for what was and was not carried over from carrot-wip.
  # ------------------------------------------------------------------
  @staticmethod
  def _get_params_model_y_3(ret, candidate, fingerprint, car_fw):
    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1

    ret.mass = (2072. if candidate == CAR.TESLA_MODEL_Y else 1899.) + STD_CARGO_KG
    ret.wheelbase = 2.890 if candidate == CAR.TESLA_MODEL_Y else 2.875
    ret.centerToFront = ret.wheelbase * 0.5
    ret.steerRatio = 12.0
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)

    safety_param = Panda.FLAG_TESLA_MODEL3_Y

    car_fw = car_fw or []
    fsd_14 = any(fw.ecu == Ecu.eps and fw.fwVersion in FSD_14_FW.get(candidate, []) for fw in car_fw)
    if fsd_14:
      ret.flags |= TeslaFlags.FSD_14
      safety_param |= Panda.FLAG_TESLA_MODEL3_Y_FSD_14

    # openpilot longitudinal is opt-in only (comparable to alpha_long in carrot-wip) --
    # left OFF by default pending real-vehicle validation of the panda safety accel checks.
    ret.openpilotLongitudinalControl = False

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla, safety_param)]

    # Ships dashcam-only until this port has been validated (panda safety unit tests +
    # bench/road testing). Flip to False deliberately once you've done that verification.
    ret.dashcamOnly = True

    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    events = self.create_common_events(ret)

    ret.events = events.to_msg()
    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    ret = self.CC.update(c, self.CS, self.frame, c.actuators, c.cruiseControl.cancel)
    self.frame += 1
    return ret
