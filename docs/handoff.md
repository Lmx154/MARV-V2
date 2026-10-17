# Handoff: M2 merged; M3 built and bench-passed on `m3`, merge gate next; then M4 software

2026-10-17. A state file, not a history: at most 80 lines, rewritten (not appended) at every milestone boundary.
History is in git and `docs/decisions/`; the previous handoff is `git show 4d3db06:docs/handoff.md`.

## Read first
- `CLAUDE.md`, then quad spec §4.0 (the MVP build order), then the milestone you execute. Decisions only when touched.

## Where things are
| Item | State |
| --- | --- |
| M0, M1 | merged (`e8b6c52`, `5365665`); not tagged (Luis's) |
| M2 | merged into `master` at `4d3db06` (local_ci on `c0cdf3c`: all five PASS); bench card `L09/results/m2-bench-card.md` |
| M3 | branch `m3`, pushed; P1-P7 reviewed; dev check PASS on `6160bae`; bench card `L09/results/m3-bench-card.md` |
| M4-M6 | not started |

## M3 (decision 0042 and its amendments; Luis's rulings in its Approval section)
- `fw/estimator`: Mahony (Betaflight 4.5.1 gains Kp 0.25, Ki 0), init from one healthy accel sample, valid flag;
  `GyroCal` boot calibration (1.25 s, movement limit 48 LSB = 3.36 dps on this board). quad_fc subtracts the bias before
  the estimator and the rate loop. Arming adds EstimatorInvalid, Calibrating, NotUpright (25°).
- Tilt budget `attitude_tilt_error_max` 0.0209 rad (1.2°, Luis). T2: `L07/replay` replays the committed hand-rotation
  log (`L07/results`, tar.xz extracted at configure): 2.4e-4 / 4.8e-4 rad. L7 Gazebo hover (`gz-l7`, preset
  `host-gz-l7`) with the estimator in authority; control: att_kp and roll/pitch rate ki 0. Plant: opt-in contact force.
- Bench IMU log variant (`MARV_FC3V2_IMU_LOG`) + `tools/target/imu_log.py`; SWD arming trace `tools/target/arming_trace.py`.
- EMB-3 with the estimator: U = 0.572 (margin 42.8 %).

## The board (madflight FC3v2 Rev-C)
- SWD probe 900678EF479C3A0E; `tools/target/flash.sh` (needs Luis's permission each session unless granted for the
  milestone; granted for M3); `--probe-only` reboots without writing. Console `tools/target/readcon.py`.
- Receiver SER0 (CRSF 420000). GPS SER1 GPIO 4/5 + PPS 40, compass I2C1 GPIO 2/3 + INT 41: not used, breadboard removed.
- Motors `motor_pad` [3, 0, 1, 2]; AM32 v2.20 on a Flywoo Goku G55M 4-in-1; the ESC stops ~0.4 s after DShot goes silent.

## Airframe (Luis, 2026-10-16/17; for M4's card)
- HGLRC MY7 7-inch, wheelbase 335 mm; motor spacing measured 224 mm (front-rear) x 255 mm (left-right), so motors at
  x ±0.112, y ±0.1275 m (not a square X); battery on top. Mass 732 g all-up with the pack (Ovonic 4S 1600 mAh 120C,
  172 g) and props (HQProp 7x3.5x3 PC, 36 g for four). Motors Emax ECO II 2807-1700KV.
- Sourced estimates (analyst, URLs in the session's research; to be put on the card with status INFERRED/UNVERIFIED):
  k_T ≈ 2.8e-6 N/(rad/s)² (2.5-3.4e-6, scaled from an Emax Avia 7040 table on a 1300KV 2807), torque ratio ≈ 0.012 m,
  inertia ≈ (4.2, 4.6, 8.5)e-3 kg m² (point-mass model, ±20 %), motor tau 30-80 ms (no source). Hover ≈ 800 rad/s.
  No thrust stand or pendulum yet; Luis has no safety gear yet, so no props-on test.

## Next
1. M3 merge gate: `MARV_CI_BASE_REF=origin/master ci/local_ci.sh` (six jobs incl. gz-l7), then merge.
2. M4 software, no hardware needed (Luis, 2026-10-16, order proposed, not yet confirmed): angle mode on the board
   (attitude law in quad_fc, sticks to angle setpoints, Betaflight angle limit); the real 7-inch on the card and the
   gains regenerated; the rate integrator held at zero at minimum throttle (Betaflight without airmode); the boot timing
   fault below. Then HIL (L10, on the deferred list: Luis must agree to pull it forward).

## Noticed, not changed
- One rate timing fault at boot when the receiver streams (0 without it); INFERRED: UART traffic at start-up.
- ESC "Motor KV" reads 820 (motors 1700KV). Console line buffer keeps a cancelling keystroke. A panic message repeats
  ~2500 lines/s. After a GyroValid-clear gap, the next sample integrates over the whole gap (arming refuses meanwhile).
- `ci/dev_check.sh` skips the nightly tools tests: run `MARV_CI_MODE=full uv run pytest -q tests/regression/quad/L01/tools
  -k "product_set or budget"` when a parameter or budget entry changes; a budget change also moves the ff_eval golden's
  hash line (regenerate in the marv-ci image).

## Known defects (fix inside the milestone named)
- M4: rotor inertia absent from plant and design model; the plant has no drag (an accelerometer never sees tilt in SIL).
- M5: four motors are compile-time (`actuator.hpp:16`, `mixer.hpp:28`).

## Rules the next lead keeps
- Standing permission (Luis, 2026-10-15): commit and push each unit that passed review and `ci/dev_check.sh`; merge
  into `master` at a milestone end after `ci/local_ci.sh` passes. Loosening a test, a spec change and tags stay Luis's.
  Fix forward; never rewrite pushed history. Before every push run `tools/ci/check_regression_changes.py`.
- At most two development checks at once. No ESC power or motor spin without Luis present. Worktrees live in the
  session scratchpad, never inside the checkout.
