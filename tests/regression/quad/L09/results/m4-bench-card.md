# M4 bench card (FC3v2, Luis present, battery unplugged, props off)

Quad spec §4.0 M4; decision 0048 item 5. Image: the bench-stats variant (`MARV_FC3V2_BENCH_STATS=ON`) of `527a8ca`
(`quad-L2-pass-275-g527a8ca`), which sets `rc_channel_angle` = 8. Tools: `tools/target/bench_stats.py`,
`tools/target/arming_trace.py`.

## 2026-10-17: EMB-3 with angle mode flying

Procedure: board on USB, battery unplugged; radio on with ch5 low (disarmed), ch8 high (angle), throttle low; quad level
and still; console recorded 150 s (`bench_stats.py capture /dev/ttyACM0 150`); Luis armed with ch5 a few seconds in and held
the sticks centred until the recording ended; armed state confirmed over SWD during the run (`emb3_angle_armcheck_swd.txt`:
`armed`).

Result (`m4_bench/emb3_angle_g527a8ca.log.gz`, `m4_bench/emb3_angle_report.txt`): console lines 3 × `switch_off`, then
147 × `armed`; rate timing_faults 0, attitude faults 0 and attitude timing_faults 0, IMU ovr/spi/order 0, link 125 frames/s;
C = 93.20 µs (irq + account maximum, including start-up), T = 133.41 µs, **U = 0.699 vs 1.0: PASS** (margin 30.1 % of T);
RTA R = 93.98 µs vs D = T: PASS. M3 (acro, the estimator in the path): C = 76.34 µs, U = 0.572.

Angle mode selected: ch8 high was set by Luis; the console does not print the mode, and the 16.9 µs rise in C over M3 is
consistent with the attitude law executing (INFERRED). The props-off radio check of angle mode (motor response to tilt with
ESC power) is still to do with Luis present; it checks the switch directly.
