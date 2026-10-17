# M3 bench card — attitude estimator, boot gyro calibration, upright check (quad spec §4.0 M3 TB), Luis present

M3 done when: "T1 known-answer tests (at rest, constant rotation) pass; a committed bench log (still, then rotated by
hand) replays (T2) within the attitude budget share `attitude_tilt_error_max`, 1.2° (Luis, 2026-10-16); the L7 Gazebo
hover passes on the estimate ..." (decision 0042).

Board: madflight FC3v2 Rev-C, ELRS receiver on SER0, RadioMaster Pocket (ch5 arm, ch3 throttle). Props off, battery
unplugged throughout (USB power). Logs: `m3_bench/` here and `tests/regression/quad/L07/results/` (the T2 logs).

## 1. Full-rate IMU log and the T2 replay (2026-10-16, image `quad-L2-pass-188-g9a40ec7`, the bench IMU-log variant)

- The log path: 30 s still capture `bench_still_30s_2026-10-16.bin` (L07 results): 284.8 kB/s, 224 840 samples at 7494.9 /s,
  0 CRC failures, 0 sequence gaps, 0 ring overflows (high water 10 of 4096), spacing 132-135 µs, 0 stamp gaps, 0 deadline
  misses, 0 FIFO overflows, 0 timing faults: LOG VALID.
- Still gyro bias, before any calibration: (−0.175, 0.306, −0.078) deg/s, 0.351 deg/s perpendicular to gravity, so
  b / Kp = 1.41° against the 1.2° share (`still_bias.py`, L07 results). This measurement decided Luis's ruling for boot
  gyro calibration (decision 0042 Approval).
- T2 log `t2_hand_rotation_2026-10-16.bin` (L07 results): 47 s still level, a hand rotation of 138° in 7.3 s, propped
  still 24 s (T_s required 19.0 s). Replayed by `tests/regression/quad/L07/replay` (the firmware estimator and the
  firmware boot calibration on the host): calibration completed 1.250 s after the first sample, bias
  (−0.1757, 0.3086, −0.1109) deg/s; check (a) err1 2.37e-4 rad, err2 4.78e-4 rad against 0.0209: **pass**; check (c')
  (gyro path, accelerometer correction off) 1.64e-2 rad against θ_rot/2 = 1.20: **pass**; control (a) 1.72 rad and
  control (c') 1.46 rad: both fail, as required. Without the calibration the same log gives (a) 0.0243 / 0.0249 rad:
  the measured bias, a correct FAIL at the 1.2° share.
- A first attempt was rejected by the harness (θ_rot 0.0028 rad: the board was laid back flat, not propped); not
  committed, its numbers are in the L07 results README.

## 2. The M3 flight image (2026-10-17, image `quad-L2-pass-216-g6160bae`, sha256 `ff633bc47c73…`)

1. Radio on, level (`step1_switch_off.log`): `switch_off`, estimate valid, tilt 0.032 rad (the frame's lean on the
   bench), link 125 frames/s: **pass**.
2. Boot gyro calibration needs stillness (`step2_calibration_trace.txt`, SWD trace at 344 µs; `step2_after.log`,
   `step2_down.log`): rebooted while Luis rocked the quad by hand: `no_link` until the link had been valid 0.5 s, then
   `calibrating` for the whole 25 s trace and after it (console: gyro up to 0.77 rad/s, tilt 0.33-0.55 rad); set down
   flat: `switch_off`, tilt 0.020 rad: **pass**.
3. Upright check (`step3_*.log`): tilted 0.61 rad (35°): `not_upright`; ch5 flipped to armed while tilted: still
   `not_upright` (refused); set level with ch5 on: `switch_not_cycled` (the refused edge does not arm later); ch5
   cycled: `armed`: **pass**.
4. EMB-3 with the estimator and the calibration in the rate path (`emb3_bench_stats_g6160bae.log.gz`, `emb3_report.txt`,
   the bench-stats variant of the same commit, 120 s): C = 76.34 µs including start-up, T = 133.403 µs, U = 0.572 vs
   1.0: **pass**, margin 42.8 % of T (M1: U = 0.539); RTA R = 77.13 µs vs D = T. One sample per interrupt, 0 FIFO
   overflows.

## Noticed, not changed

- One rate-loop timing fault at boot (`timing_faults=1`, disarmed) on every boot with the receiver streaming
  (2026-10-16 M2 reboot, today's `--probe-only` reboot, the bench-stats flash, the flight-image re-flash); 0 on boots
  with the receiver off or absent (the M1 soak, the first flash of `g6160bae` with the radio off). INFERRED cause:
  receiver UART traffic at start-up delays one early tick. To investigate in M4 before props-on.
- The still-bench accelerometer x/y means read outside the profile's ±12 mg offset band (−13.2, +15.3 mg): the frame's
  ~1.2° lean on the bench, not a sensor offset (the T2 check compares against the measured mean, so it is unaffected).
- Gyro still noise sd 104 / 103 / 80 mdps against the profile's 65 mdps (ratio 1.6 / 1.6 / 1.2), as in M1.
