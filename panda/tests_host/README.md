# Host-native safety test (Model Y/3 port)

This directory is NOT part of the official build system. It's a standalone, host-compilable
(no ARM cross-toolchain needed) test harness written to verify the Model Y/3 additions to
`../board/safety/safety_tesla.h`, since this fork has no `tests/safety` suite of its own and
the sandbox this was written in couldn't install the real cross-compiler / capnp / SCons stack.

It works because `board/safety.h` and its helpers (`addr_safety_check`, `msg_allowed`,
`interpolate`, etc.) are portable C with no hardware register access — only `board/config.h`'s
transitive includes pull in the real STM32 target, so this harness stubs the handful of
board-specific externs (`current_board`, fault LEDs, `microsecond_timer_get`) instead of
including that chain, then includes the *real*, unmodified `safety.h` (which pulls in the
*real*, unmodified `safety_tesla.h`).

Build and run:

```sh
gcc -Wall -I../board test_tesla_model_y_safety.c -o test_tesla_model_y_safety
./test_tesla_model_y_safety
```

35/35 checks passed when this was last run (2026-08-21): cruise engage/disengage, Summon
lockout, steering angle-rate-limit accept/reject, msg_allowed bus filtering, longitudinal
accel-limit accept/reject, AEB-active blocking, forward-hook bus routing, and — importantly —
a regression pass confirming the existing (untouched) AP1/AP2 code path still behaves as before.

**What this does and does not prove**: it confirms the safety C logic compiles cleanly and
behaves as intended against the specific scenarios written here, on a host machine, using
synthetic CAN frames whose bit-encoding was hand-verified against the DBC text. It is **not**
comma's official safety unit test suite, is not run against real CAN traffic, and is not a
substitute for bench/road testing on real hardware. Treat a passing run as "the logic does what
I intended," not as "this is safe to drive."
