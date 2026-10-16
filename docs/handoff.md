# Handoff: M0 accepted; M1 bench passed; P7b in progress, then the P12b soak and the M1 merge

2026-10-16. A state file, not a history: at most 80 lines, rewritten (not appended) at every milestone boundary.
History is in git and `docs/decisions/`; the previous handoff is `git show b641f6f:docs/handoff.md`.

## Read first
- `CLAUDE.md`, then quad spec §4.0 (the MVP build order), then the milestone you execute. Decisions only when touched.

## Where things are
| Item | State |
| --- | --- |
| L0–L5, L11 | passed, tagged up to `quad-L11-pass` |
| Known failing item | L4 acro cross-axis coupling (linear segments and recovery), strict xfail, owned by the deferred L6 FF (0030) |
| M0 | merged into `master` at `e8b6c52` (decisions 0028–0033); accepted on the local gate (Luis 2026-10-16, "5. yes"); Actions warm run 23.1 vs 20.8 min, runner noise 15.9–29.2 |
| M1 | branch `m1-sdk`, pushed; P1–P11, P12a and the powered bench done; P7b (branch `m1-p7b`) and P12b open |
| M2–M6 | not started |

## M1 on the board (madflight FC3v2 Rev-C; decisions 0027, 0034–0038)
- RP2354B rev 3, 2048 KiB W25Q16JV in package, clk_sys 150 MHz. SWD via a Pico 2 debugprobe (serial
  900678EF479C3A0E; GP2 SWCLK, GP3 SWDIO) with `tools/target/flash.sh` (reboots the whole chip so the USB console
  re-enumerates); console `tools/target/readcon.py` / `sendcon.py` on `/dev/ttyACM1` or `ACM2` (`/dev/serial/by-id`).
- Image: `quad_fc` (gyro chain → rate loop → mixer) behind a motor gate that writes DShot stop except during a motor
  test; LSM6DSV IRQ tick at 7497 Hz (INTERNAL_FREQ_FINE −18; nominal 7680); PIO DShot600 (waveform captured, 0037);
  a panic streams "PANIC: <msg>" over USB and keeps sending stop frames (Luis: "take option A"); console
  `motor_test <1-4> <dshot> <ms> PROPS-OFF`, cap 500, at most 20 s (Luis: "1. 500 …"; "20s is good."; 0038 and its
  amendment). `motor_pad` = [3, 0, 1, 2] (measured at the bench).
- Runs from SRAM (copy_to_ram): EMB-3 passes, U = 0.539, R = 72.6 µs < 133.4 µs; from XIP flash it failed (U = 1.52).
  Evidence: `tests/regression/quad/L09/results/p12_bench/report.txt`.
- Every rate execution is a counted timing fault today: the board's tick period is still the 625/4 µs PLACEHOLDER
  (P7b). Nothing halts (0034); the gate writes stop anyway.

## Bench and ESC (2026-10-16, `tests/regression/quad/L09/results/m1-bench-card.md`)
- Pass: ESC arming on stop frames, motor survey (all four directions match quad-X, AM32 reverse on ESC 1 and 4), the
  map, cancel, disconnect, SWD halt (the AM32 ESC cuts the motor ~0.4 s after the DShot line goes silent, then resets).
  Panic not run. Hardware watchdog deferred until after M4 as a recovery feature (decision 0039, branch `m1-wdt`).
- AM32 v2.20 (FLYWOO_GOKU_F421) on all four; settings and EEPROM backups in `m1_bench_2026-10-16/am32/`. ESC access
  without a linker: flash Betaflight 2026.6.2 MADFLIGHT_FC3 (built from source; 4-way passthrough is in the RP2350
  port) over SWD, use am32.ca, reflash MARV. The build is not in the repo; rebuild it with `make CONFIG=MADFLIGHT_FC3`.
- Airframe (Luis): Emax ECO II 2807-1700KV, HGLRC MY7 7-inch (335 mm wheelbase). The card still carries the UZH
  5-inch physics as placeholders; `rotors.speed_range` = [0, 2991] rad/s (Luis: no cap, DShot's own range). The ESCs'
  "Motor KV" setting reads 820, not 1700 (noticed, not changed).

## Rulings 2026-10-16 (Luis: "1. yes … 3. yes 4. yes 5. yes")
1. P7b: the board's rate design is guarded at the chip's mean tick period (from FREQ_FINE), SIL unchanged (0014
   amendment, in progress on `m1-p7b`: register period 3906250/29298 µs, start-up panic on a different trim).
2. Rotor speed range: done on the card. 3. Hard-bound the motor-test cap at 500 in code: after P7b lands.
4. Gyro noise: both kept, UNVERIFIED (board profile). 5. M0 accepted on the local gate. 6. AM32: read (above).

## Next
1. Review and merge P7b into `m1-sdk`, flash, then P12b: soak with 0 timing faults, δ measured (stamp entry jitter
   0.76 µs in SRAM). Then the 500 cap constant, the M1 merge gate (`ci/local_ci.sh`) and the merge.
2. Carried fixes: gyro saturation threshold (P9 review); guard a stuck IMU interrupt line (M2 stale-IMU failsafe);
   the attitude law still halts on spacing (`attitude_law.hpp:56-82`, before M3 runs it on the board).
3. The console line buffer keeps a cancelling keystroke, so the next command is refused (fails safe; noticed).

## Known defects (fix inside the milestone named)
- M3: `marv_plant` reports zero specific force at rest (`marv_plant.cpp:255-264`).
- M4: rotor inertia absent from plant and design model; rate crossover ~8–12 rad/s from the ±50 % / ±30 % bands.
- M5: four motors are compile-time (`actuator.hpp:16`, `mixer.hpp:28`).

## Git and host
- `master` = M0 (`b641f6f`). `m1-sdk` = M1 with `master` merged; worktrees in the session scratchpad (`wt-m1*`),
  never inside the checkout (they break `test_world_sensors.py`). One stash, `stash@{0}`: not ours, leave it.
- No `gh` CLI (read the Actions API with `curl`). Docker `marv-ci` regenerates the product table (README recipe; add
  `-e CCACHE_DISABLE=1` when the container cannot write the ccache directory).

## Rules the next lead keeps
- Standing permission (Luis, 2026-10-15, "yes you may commit and push"): commit and push each unit that passed review
  and `ci/dev_check.sh`; merge into `master` at a milestone end after `ci/local_ci.sh` passes. Loosening a test, a spec
  change and tags stay Luis's. Fix forward; never rewrite pushed history.
- Before every push: read the log's own exit line (not a wrapper's) and run `tools/ci/check_regression_changes.py`.
- At most two development checks at once (a timing test timed out under three).
- No ESC power or motor spin without Luis present.
