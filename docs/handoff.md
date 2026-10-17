# Handoff: M1 merged; M2 built and bench-passed on `m2`, merge gate next; then M3

2026-10-16. A state file, not a history: at most 80 lines, rewritten (not appended) at every milestone boundary.
History is in git and `docs/decisions/`; the previous handoff is `git show 5365665:docs/handoff.md`.

## Read first
- `CLAUDE.md`, then quad spec §4.0 (the MVP build order), then the milestone you execute. Decisions only when touched.

## Where things are
| Item | State |
| --- | --- |
| L0–L5, L11 | passed, tagged up to `quad-L11-pass` |
| Known failing item | L4 acro cross-axis coupling, strict xfail, owned by the deferred L6 FF (0030) |
| M0 | merged at `e8b6c52`, accepted on the local gate (Luis 2026-10-16) |
| M1 | merged into `master` at `5365665` (local_ci on `6e32b13`: core, gz-l2, gz-l4, gz-l5, gz-l11 PASS). Not tagged (Luis's) |
| M2 | branch `m2`, pushed; P1–P7 reviewed; bench passed (`tests/regression/quad/L09/results/m2-bench-card.md`) |
| M3–M6 | not started |

## The board (madflight FC3v2 Rev-C; decisions 0027, 0034–0041)
- RP2354B, clk_sys 150 MHz, runs from SRAM (EMB-3 U = 0.539). SWD: Pico 2 debugprobe 900678EF479C3A0E;
  `tools/target/flash.sh` flashes and reboots the whole chip (`--probe-only` reboots without writing: the way out of a
  panic; OpenOCD `reset run` leaves USB down). Console `tools/target/readcon.py` / `sendcon.py` (`/dev/serial/by-id`).
- `tools/target/arming_trace.py`: polls the arming state over SWD every ~350 µs without halting (a halt pauses the
  timer, DBGPAUSE); `--stall-imu-after S` clears the IMU interrupt to exercise the stale-IMU panic.
- Tick: LSM6DSV at the chip's own period 1953125/14649 µs (P7b, 0014 amendment); soak on `679d12f`: 2 503 331
  executions, 0 timing faults, 0 FIFO overflows. IMU read deadline D = 14532 µs from a FIFO depth that is INFERRED
  (109 samples): measure it on the board.
- Pins (Luis): receiver SER0 GPIO 0/1 (CRSF 420000, used); NEO-M9N GPS SER1 GPIO 4/5, time pulse GPIO 40; BMM350
  compass I2C1 GPIO 2/3, INT GPIO 41 (not used yet).
- Motors: `motor_pad` [3, 0, 1, 2]; AM32 v2.20 on all four (settings in `m1_bench_2026-10-16/am32/`); the ESC cuts
  the motor ~0.4 s after DShot goes silent. ESC access without a linker: Betaflight 2026.6.2 MADFLIGHT_FC3 over SWD,
  am32.ca, reflash MARV. Hardware watchdog deferred until after M4 (0039). Console motor test at most 20 s (0038).

## M2 (decisions 0040, 0041)
- CRSF receiver, Betaflight Actual rates (70/670 °/s, expo 0), arming on a ch5 rising edge with throttle below
  `arm_throttle_min` (Betaflight min_check), failsafe exactly Betaflight 4.5.1 (100 ms / 300 ms / 1.5 s Drop; 0.5 s of
  link plus a switch cycle to re-arm). Radio (Luis): RadioMaster Pocket, ch5 +100 armed, ch3 throttle.
- Bench: sticks, arm, throttle-up refusal, failsafe timeline (disarm 1500.4 ms), stale-IMU panic, motors from the
  radio all pass. Props off, the motors wind up while armed: the integrator restarts only at arming.

## Next
1. M2 merge gate: `MARV_CI_BASE_REF=origin/master ci/local_ci.sh` (the clone has no local `master`), then merge.
2. M3, attitude estimator (quad §4.0). First fix: `marv_plant` zero specific force at rest (`marv_plant.cpp:255-264`);
   the attitude law still halts on spacing (`attitude_law.hpp:56-82`).
3. Before props-on (M4): hold the rate integrator at zero while the throttle is at minimum, as Betaflight does without
   airmode (check the 4.5.1 source first; Luis: fine as long as the reason is known, add it if needed). Airframe card:
   motor positions from the 335 mm wheelbase, 1700KV, measured all-up mass, Luis's prop model; thrust stand and
   bifilar pendulum (M4 characterises the motor + prop). The card still carries UZH 5-inch physics.

## Noticed, not changed
- ESC "Motor KV" reads 820, motors are 1700KV. Console line buffer keeps a cancelling keystroke. A panic message
  repeats ~2500 lines/s on the console. One rate timing fault on a reboot after a panic (0 on a fresh flash).
  Receiver error counters grow at some boots (0 on a clean one). ESC 2's settings differ from the others.
- `ci/dev_check.sh` skips the full-mode nightly tools tests (e.g. L01 `test_cmake_product_set`): run that test by hand
  whenever a parameter is added (`tests/regression/quad/*/param_ids` must list it).

## Known defects (fix inside the milestone named)
- M4: rotor inertia absent from plant and design model; rate crossover ~8–12 rad/s from the ±50 % / ±30 % bands.
- M5: four motors are compile-time (`actuator.hpp:16`, `mixer.hpp:28`).

## Git and host
- Worktrees in the session scratchpad, never inside the checkout (they break `test_world_sensors.py`). One stash,
  `stash@{0}`: not ours, leave it. No `gh` CLI. Docker `marv-ci` regenerates the product table (README recipe).

## Rules the next lead keeps
- Standing permission (Luis, 2026-10-15): commit and push each unit that passed review and `ci/dev_check.sh`; merge
  into `master` at a milestone end after `ci/local_ci.sh` passes. Loosening a test, a spec change and tags stay Luis's.
  Fix forward; never rewrite pushed history. Flashing an externally built image needs Luis's explicit permission.
- Before every push: read the log's own exit line and run `tools/ci/check_regression_changes.py`.
- At most two development checks at once. No ESC power or motor spin without Luis present.
