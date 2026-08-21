# Tesla Model Y / Model 3 port (from carrot-wip) — read before using

This branch adds `TESLA_MODEL_Y` / `TESLA_MODEL_3` support to this fork, backported from
`kimjk2629/openpilot` (`carrot-wip` branch) on 2026-08-21. It sits alongside the existing
`AP1_MODELS` / `AP2_MODELS` (Model S) support, which is untouched.

**Status: unverified on real hardware. Do not drive with this until you have validated it
yourself.** This was written by an AI coding assistant with no access to a real Tesla and no
ability to flash/bench-test panda firmware. The sandbox this was built in also could not reach
PyPI/apt (network is allowlisted), so the official build (SCons + `capnp`/`cereal` codegen +
ARM cross-compile of panda firmware + Cython `opendbc/can` bindings) could not be run
end-to-end. What *was* possible, and was actually done rather than just eyeballed:

- The two new DBC files were compiled through this repo's own `opendbc/can/process_dbc.py` —
  confirmed working, not assumed.
- Every CAN signal name used in `carstate.py`/`teslacan.py` was cross-checked against the raw
  DBC text (not just trusted from carrot-wip's source).
- The bit-level encode/decode formulas in `safety_tesla.h` (steering angle, steering control
  type, accel max/min, acc state, AEB event) were independently checked against each signal's
  DBC bit position/length/byte-order/scale/offset — not just copied and hoped for.
- `panda/board/safety.h` (including the modified `safety_tesla.h`) was compiled standalone on
  host gcc with `-Wall`: zero errors, zero warnings.
- A host-native test harness (`panda/tests_host/`, see its own README) exercises the actual
  compiled Model Y/3 safety logic against synthetic CAN frames: cruise engage/disengage, Summon
  lockout, steering angle-rate-limit accept/reject, msg_allowed bus filtering, longitudinal
  accel-limit accept/reject, AEB blocking, and forward-hook routing — **35/35 passing**,
  including a regression pass proving the existing AP1/AP2 path still behaves as before.

That's real compiled-and-executed verification of the highest-risk part (the panda safety
logic), which is more than a pure read-through gives you — but it is still not comma's official
safety test suite, not real CAN traffic, and not bench/road testing. Treat this as a
substantially-checked starting point, not a finished, drive-ready driver.

## Why this couldn't be a straight copy

`carrot-wip`'s Tesla code targets a completely different generation of openpilot and a
completely different Tesla integration than what this fork already had:

- This fork's existing Tesla support is **AP1/AP2 Model S** only, using a full CAN-bus-bypass
  harness. `carrot-wip`'s Tesla code is for **HW3/HW4 Model 3/Y/X**, using a non-invasive
  "party bus" tap harness (Tesla A/B harness) that talks *alongside* the car's own Autopilot/FSD
  computer instead of replacing it. **These require physically different comma harness
  hardware** — this patch does not and cannot change what's plugged into the car.
- The Python car-interface layer (`values.py`/`carstate.py`/`carcontroller.py`/`interface.py`)
  uses different base classes, different CAN parser APIs, FW-based fingerprinting vs. static
  CAN fingerprinting, and different message sets between the two generations.
- The panda safety firmware API is also a different generation:
  `carrot-wip`'s `safety_tesla.h` uses newer helpers (`RxCheck`, `BUILD_SAFETY_CFG`,
  `steer_angle_cmd_checks`, `longitudinal_accel_checks`) that don't exist in this fork's older
  panda (`AddrCheckStruct`, `addr_safety_check`, `msg_allowed`, `interpolate`+`max_limit_check`).
  The safety logic had to be manually rewritten against the old API, not copy-pasted.

## What was ported

- `selfdrive/car/tesla/values.py` — new `CAR.TESLA_MODEL_Y` / `CAR.TESLA_MODEL_3`, DBC entries,
  `FW_VERSIONS` / `FSD_14_FW` tables (copied verbatim from carrot-wip), new `CANBUS` party-bus
  indices, `CarControllerParamsMY`.
- `opendbc/tesla_model3_party.dbc`, `opendbc/tesla_radar_continental_generated.dbc` — copied
  from carrot-wip. **Verified**: both compile cleanly through this repo's own
  `opendbc/can/process_dbc.py` (i.e. the DBC syntax itself is compatible with this fork's older
  DBC→C++ generator — I actually ran this, not just assumed it).
- `selfdrive/car/tesla/teslacan.py` — new `TeslaCANModelY` class: steering control, longitudinal
  command, and the `APS_eacMonitor` "steering allowed" heartbeat (see below).
- `selfdrive/car/tesla/carstate.py` — branches on `CP.carFingerprint` between the old AP1/AP2
  parsing path and a new Model Y/3 path. Every signal name used was individually checked against
  the actual DBC text (not just trusted from the source code) to make sure it exists.
- `selfdrive/car/tesla/carcontroller.py` — branches similarly. Model Y/3 steering uses a fixed
  speed/angle-rate table (same style as this repo's existing AP1/AP2 code) instead of
  carrot-wip's `VehicleModel`-based limiting — the panda safety firmware is the actual
  enforcement backstop either way.
- `selfdrive/car/tesla/interface.py`, `radar_interface.py` — branch per platform. Radar is wired
  up but will simply stay silent on vision-only Model Y/3 (i.e. almost all cars built since
  ~2021), which is expected and fine.
- `selfdrive/car/fw_versions.py` — added a `"tesla"` entry to `REQUESTS` for FW-based
  fingerprinting (see **known gap** below — it likely won't fire automatically).
- `panda/python/__init__.py` — new `Panda.FLAG_TESLA_MODEL3_Y[...]` constants, distinct bits from
  the existing AP1/AP2 flags so nothing about Model S support changed.
- `panda/board/safety/safety_tesla.h` — new Model Y/3 code path (rx/tx/fwd hooks), added
  alongside the untouched AP1/AP2 code, selected by the new `TESLA_FLAG_MODEL3_Y` safety param
  bit. Steering angle-rate limits, longitudinal accel limits, stock-steering-control conflict
  detection (don't fight LDA/ELDA/Autopark), and Summon/Autopark lockout were all ported.
- `panda/board/safety.h` — un-commented the Tesla safety hook `#include` and its entry in the
  hook dispatch table. **Left inside the existing `#ifdef ALLOW_DEBUG` block** — see below.

## What was intentionally dropped for this first pass (not fundamental blockers, just scope)

- **Blinker MITM** (riding turn signals onto `DAS_bodyControls` on the vehicle bus) and the
  **3-finger infotainment LKAS toggle** — both needed an extra vehicle-bus CAN parser wired
  through this fork's `get_body_can_parser` hook. Skipped to keep the safety-relevant surface
  area smaller for a first, unverified pass.
- **"Coop steering"** driver-blend module (`coop_steering.py` in carrot-wip) — not ported;
  plain angle control only.
- The runtime "suspected FSD 14" auto-detection heuristic — kept the simpler FW-list-based
  `FSD_14` flag only. If your car's firmware isn't in `FSD_14_FW` in `values.py`, steering
  control type may be inverted and won't self-correct at runtime. Check your EPS firmware
  version against that list (or add it) before relying on lateral control.
- `openpilotLongitudinalControl` defaults to `False` for Model Y/3 in `interface.py`. The
  longitudinal path is implemented in `carcontroller.py`/`teslacan.py`/`safety_tesla.h`, but I
  left it off by default pending real validation of the panda accel-limit checks. Flip it in
  `interface.py` once you've verified that path specifically.

## Known gap: FW fingerprinting will probably not detect the car automatically

This fork's `car_helpers.py.fingerprint()` queries ECU firmware on a single **hardcoded bus
(`bus = 1`, OBD-II)**, shared across every brand. But on the HW3/HW4 party-bus harness, the
Tesla EPS firmware-version response arrives on **bus 0**. I did not touch that shared,
multi-brand bus-selection code — this fork is Hyundai-focused and I didn't want to risk
destabilizing Hyundai FW detection to fix a Tesla-only edge case I can't test.

**Practical workaround**: this fork already supports forcing the detected car via the
`CarModel` param (see `CAR_NAME = Params().get("CarModel", ...)` in `car_helpers.py`). Set that
param to `TESLA_MODEL_Y` (or `TESLA_MODEL_3`) to bypass fingerprinting entirely.

## Why the safety mode is gated behind `ALLOW_DEBUG`

The existing AP1/AP2 Tesla safety hook in this fork was **already** commented out of the
release dispatch table before this patch (Tesla wasn't even compiled into a normal panda
firmware build), and the existing AP1/AP2 car interface already sets `dashcamOnly = True`. I
kept that same posture for the new Model Y/3 code: it's real logic, but it only ends up in a
firmware build when built with `ALLOW_DEBUG`, i.e. it is not part of a normal release build.
`interface.py` also sets `dashcamOnly = True` for `TESLA_MODEL_Y` / `TESLA_MODEL_3` by default.

Un-gating either of those (moving the hook outside `ALLOW_DEBUG`, or flipping `dashcamOnly` to
`False`) is a decision you should make deliberately, after you've done your own verification —
not something this patch does for you.

## Before you drive with this — minimum bar

1. Get comma's actual Tesla Model 3/Y ("Tesla A"/"Tesla B") harness — the AP1/AP2 harness this
   fork already supports is different hardware and will not work here.
2. Read `panda/board/safety/safety_tesla.h` yourself, line by line, against the DBC. It compiles
   clean and passes the 35 scenarios in `panda/tests_host/`, and every bit-level formula was
   hand-checked against the DBC — but that's still not comma's official safety unit test suite
   (this fork doesn't have one checked in), and no amount of static review or host-side testing
   substitutes for real bench/road testing. Extend `panda/tests_host/` with more scenarios
   before you trust it further.
3. Bench test with the car in park, wheels off the ground or on a dyno, before any road use.
4. Start with `dashcamOnly = True` (the default) and confirm `CarState` output (speed, steering
   angle, cruise state, gear, doors, blinkers) looks sane on your dash/logs before ever flipping
   it off.
5. Only then consider flipping `dashcamOnly` and moving the safety hook out of `ALLOW_DEBUG`.

## Files touched

```
opendbc/tesla_model3_party.dbc                          (new)
opendbc/tesla_radar_continental_generated.dbc            (new)
selfdrive/car/tesla/values.py                             (modified)
selfdrive/car/tesla/teslacan.py                            (modified)
selfdrive/car/tesla/carstate.py                            (modified)
selfdrive/car/tesla/carcontroller.py                        (modified)
selfdrive/car/tesla/interface.py                             (modified)
selfdrive/car/tesla/radar_interface.py                        (modified)
selfdrive/car/fw_versions.py                                   (modified)
panda/python/__init__.py                                        (modified)
panda/board/safety/safety_tesla.h                                 (modified)
panda/board/safety.h                                                (modified)
```
