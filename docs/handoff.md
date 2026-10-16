# Handoff: M0 merged; M1 on the board, waiting for the powered bench session and three rulings

2026-10-16. A state file, not a history: at most 80 lines, rewritten (not appended) at every milestone boundary.
History is in git and `docs/decisions/`; the previous handoff is `git show b641f6f:docs/handoff.md`.

## Read first
- `CLAUDE.md`, then quad spec §4.0 (the MVP build order), then the milestone you execute. Decisions only when touched.

## Where things are
| Item | State |
| --- | --- |
| L0–L5, L11 | passed, tagged up to `quad-L11-pass` |
| Known failing item | L4 acro cross-axis coupling (linear segments and recovery), strict xfail, owned by the deferred L6 FF (0030) |
| M0 | merged into `master` at `e8b6c52` (decisions 0028–0033); done-when not met on Actions (Next 3) |
| M1 | branch `m1-sdk`, pushed; P1–P11 and P12a done and on the board; P7b, the powered bench and P12b open |
| M2–M6 | not started |

## M1 on the board (madflight FC3v2 Rev-C; decisions 0027, 0034–0038)
- RP2354B rev 3, 2048 KiB W25Q16JV in package, clk_sys 150 MHz. SWD via a Pico 2 debugprobe (serial
  900678EF479C3A0E; GP2 SWCLK, GP3 SWDIO) with `tools/target/flash.sh` (reboots the whole chip so the USB console
  re-enumerates); console `tools/target/readcon.py` / `sendcon.py` on `/dev/ttyACM1` or `ACM2` (`/dev/serial/by-id`).
- Image: `quad_fc` (gyro chain → rate loop → mixer) behind a motor gate that writes DShot stop except during a motor
  test; LSM6DSV IRQ tick at 7497 Hz (INTERNAL_FREQ_FINE −18; nominal 7680); PIO DShot600 (waveform captured, 0037);
  a panic streams "PANIC: <msg>" over USB and keeps sending stop frames (Luis: "take option A"); console
  `motor_test <1-4> <dshot> <ms> PROPS-OFF`, cap 500, at most 3 s (Luis: "1. 500 2. 3 seconds 3. yes"; 0038).
- Runs from SRAM (copy_to_ram): EMB-3 passes, U = 0.539, R = 72.6 µs < 133.4 µs; from XIP flash it failed (U = 1.52).
  Evidence: `tests/regression/quad/L09/results/p12_bench/report.txt`.
- Every rate execution is a counted timing fault today: the board's tick period is still the 625/4 µs PLACEHOLDER
  (P7b). Nothing halts (0034); the gate writes stop anyway.

## Open for Luis (each one word)
1. P7b, amends decision 0014: guard the board's rate design at the mean period instead of the exact SIL stamp pattern
   (refused for patterns longer than 2), SIL guard unchanged, timing tolerance rounded up to whole µs; the period from
   the chip's FREQ_FINE at start-up (7500.3 Hz here, not 7680). Recommended: yes.
2. The airframe card's `rotors.speed_range` (motor limit; feeds rotor_speed_min/max and idle). UZH's [150, 2800] rad/s
   was not copied. Needed before the airframe card feeds the board's parameter set.
3. Hard-bound the motor-test cap at 500 in code (today a parameter with your provenance; init accepts up to 2047)?
   Recommended: yes.
4. Gyro still-bench noise x/y 120/114 mdps vs the datasheet's typical 64.9 (z matches if LPF2 is first order): keep both
   as UNVERIFIED? Recommended: yes.
5. M0's Actions criterion: runner noise (gz-l5 15.9–29.2 min across runs) hides the change. Accept M0 on the local gate
   and the per-unit checks (P1: 3 min vs 24), or a later M0b on the slow core steps? Recommended: accept.
6. AM32 version and settings: unread (no passthrough); needed by M4.

## Next
1. Powered bench with Luis, props off: `tests/regression/quad/L09/results/m1-bench-card.md` (ESC arming on stop
   frames, motor survey → `motor_pad_m1..m4` and AM32 direction, cancel/disconnect/SWD-halt stops). Commit the log.
2. P7b after ruling 1, then P12b: soak with 0 timing faults, δ measured (stamp entry jitter 0.76 µs in SRAM). Then the
   M1 merge gate (`ci/local_ci.sh`) and merge.
3. Carried fixes: gyro saturation threshold (P9 review); guard a stuck IMU interrupt line (M2 stale-IMU failsafe);
   the attitude law still halts on spacing (`attitude_law.hpp:56-82`, before M3 runs it on the board).

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
