# HIL S3b: open-loop SIL-to-board comparison (decision 0046 stage S3)

Tool: `tools/target/hil_compare.py` (bench only, standard library only; `python3 tools/target/hil_compare.py --help`).
Pass bar (0046): a recorded SIL sample stream replayed to the board gives DShot outputs equal to the SIL's within float
tolerance; EMB-3 reported. Board results: added by the lead below ("Board run"); none yet.

## Commands

```sh
cmake --preset host-debug && cmake --build --preset host-debug -j 8 --target marv_sil_quad_fc
uv sync --frozen
# reference: the host SIL library (build/host-debug/sim/quad_fc/libmarv_sil_quad_fc.so), one scenario per process
uv run python tools/target/hil_compare.py reference sticks.npz --scenario sticks
uv run python tools/target/hil_compare.py reference motion.npz --scenario motion
# per scenario: RESET THE BOARD (power cycle), then replay and compare
uv run python tools/target/hil_compare.py replay /dev/ttyACM0 sticks.npz sticks.bin
uv run python tools/target/hil_compare.py compare sticks.npz sticks.bin        # writes sticks.bin.compare.txt
# the tool's own checks, no board
python3 tools/target/hil_compare.py selftest
```

`replay` reads the hello and refuses (exit 2) if the schema hash, parameter count, values CRC, tick period or rate divisor
differ from the reference's. The host values CRC is computed in Python from the generated fc3v2 table
(`build/host-debug/generated/marv_params_fc3v2/marv/params/param_defaults.cpp`, the board's rule: CRC-32 over the values in
ParamId order, 4 bytes little-endian) and equals the smoke run's board value 0x97ed61ec (the selftest checks it). Samples go
out 4 per record, each record when its last sample's stamp is due (real time, as `hil_fc.py`); a ManualControl record goes out
just before the sample record that starts at its tick. Raw uplink: `OUT.bin`; counts of the run: `OUT.bin.sent.json`.

A fresh board per replay: the image keeps the composition state and the counters from boot, and the stream ends with one IMU
deadline miss (no record for D_HIL) that latches a disarm. `compare` refuses a board whose counters are not those of one run
("was the board reset?").

## Scenarios (stamps = sample time)

Same sticks in both (ManualControl frames every 60 ticks from tick 0, about 125 Hz; the IMU stream is the same bytes on both
sides): wait 0 s (arm switch low, throttle 0), idle 3.0 s (switch high), throttle 3.5 s (0.4), roll 4.5 s (0.5), pitch 5.5 s
(0.5), yaw 6.5 s (0.5), centred 7.5 s (sticks 0, throttle 0.4), disarm 8.0 s (switch low), end 8.5 s (63 720 ticks,
31 860 rate ticks, 1 062 frames). The calibration (1.25 s) and the link recovery (0.5 s) end before the arm edge.

- `sticks`: still level IMU (gyro exactly 0, accel (0, 0, -g0) FRD, gyro and accel valid). Open loop: the rate error and the
  integrator grow while a stick is held, so P, I, the mixer, its desaturation and DShot quantisation are exercised.
- `motion`: the same from a still start; from 2.0 s a smooth roll/pitch tilt motion (roll 0.15 rad at 1.7 Hz, pitch 0.10 rad
  at 1.1 Hz, smoothstep ramp 2.0 to 2.5 s; gyro = its Euler-rate body rates, accel = the attitude's specific force), so the
  estimator, the filters and D see motion.

## Reference outputs (host-debug library, DShot of the armed rate ticks per phase)

| phase (s) | sticks | motion |
| --- | --- | --- |
| wait 0-3 | all 0 | all 0 |
| idle 3-3.5 | 48 on all | m1 48..256, m2 48..328, m3 48..316, m4 48..306 |
| throttle 3.5-4.5 | 48..1313 on all | m1 257..1332, m2 172..1333, m3 48..1323, m4 293..1324 |
| roll 4.5-5.5 | m1,m4 1279..1313, m2,m3 1312..1345 | m1 1274..1319, m2 1311..1358, m3 1304..1346, m4 1269..1319 |
| pitch 5.5-6.5 | m1 1279..1315, m2 1309..1345, m3 1344..1367, m4 1255..1280 | m1 1272..1328, m2 1306..1357, m3 1335..1367, m4 1249..1278 |
| yaw 6.5-7.5 | m1 1314..1820, m2 1309..1824, m3 551..1367, m4 48..1256 | m1 1325..1837, m2 1306..1836, m3 544..1357, m4 48..1259 |
| centred 7.5-8 | m1 1734..1822, m2 1739..1827, m3 550..735, m4 48..516 | m1 1726..1840, m2 1742..1838, m3 526..744, m4 48..517 |
| disarm 8-8.5 | 28 armed rate ticks (one frame period), then 0 | same |

Armed at rate tick 22 560 (3.0079 s), stopped at 60 060 (8.0077 s) in both. m4 reaches the 48 floor and m3 551 in the yaw phase:
the mixer's desaturation is exercised. Deterministic: two runs of the same library give 0 counts (refdiff, below).

## Floating-point facts (what each build uses)

- Host (the SIL library): `cmake/flags.cmake` lines 18-20 add `-ffp-contract=off` to every host target
  (`MARV_TARGET` host). The reference is therefore the unfused one.
- M33 (the HIL and flight images): the same file adds no `-ffp-contract` for `MARV_TARGET` m33. The compile command of
  the HIL image's tree has none (`-mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mfloat-abi=hard -O3 -std=c++20`,
  `build/m33-hil/compile_commands.json`: 0 occurrences) and `arm-none-eabi-g++ 13.2.1 -Q --help=common` with those flags
  reports `-ffp-contract=[off|on|fast]  fast` (GCC's default; `-std=c++20` without extensions only turns it off for C). The
  linked HIL ELF (`marv_fc3v2.elf` built at the S3a commit) contains 295 VFMA/VFMS/VFNMA.F32 instructions, in
  `RateGroupStep::filter` (52), `Mahony::step` (43), `composition::tick` (42), `mixer::allocate` (36), `execute_bypass`
  (20), `execute` (16), `RateGroupStep::init` (16), `finish` (12), `Quat::operator*` (12) and others.
- So the "-ffp-contract=off both" condition of stage S3 does not hold: the M33 image fuses. Not changed here (no build-flag
  change in this unit); the lead decides whether to add the flag to the m33 build or to accept a tolerance.
- libm: the M33 links the Pico SDK's `__wrap_sincosf`, `__wrap_tanf`, `__wrap_hypotf` (in the ELF), the host glibc's; they
  may differ by an ulp (filter coefficients at init, tilt).

## Tolerance

`compare` accepts the board when every rate tick's four DShot values are within `--tolerance` (default 1 count) of the
reference's, the arming transitions (the DShot going from 0 to non-zero and back, per rate tick) are on the same ticks, the
board's counters show no timing, attitude, link or queue fault and exactly one deadline miss after the stream. Mechanism: a
fused a*b+c or a one-ulp libm difference changes a float by a few 2^-24 relative; the DShot count is that float scaled and
rounded, so the rounded count changes by one when the value is within that error of a rounding boundary, and not otherwise.
The error does not grow with time (stable filters; open-loop terms are sums of bounded terms). The bound is empirical, from
a host proxy of the M33's fusing: the SIL library built in a scratch copy of the tree with `-ffp-contract=off` replaced by
`-ffp-contract=fast -mfma` in `cmake/flags.cmake` (host-release, -O3), references made with both libraries and compared with
`refdiff`: `fma_proxy.txt` (raw output). Result: at most 1 count apart (sticks: 1 737 of 127 440 values differ, 1.36 %;
motion: 2 197, 1.72 %), the arming transitions on the same ticks. An unfused host against the fused M33 with a different
libm can differ more than this proxy: a board maximum of 2 counts is a finding to report, not a pass.

```sh
git archive HEAD | tar -x -C scratch && cd scratch
sed -i 's/-ffp-contract=off/-ffp-contract=fast -mfma/' cmake/flags.cmake
cmake --preset host-release && cmake --build --preset host-release -j 8 --target marv_sil_quad_fc
python3 tools/target/hil_compare.py reference fma_sticks.npz --scenario sticks --lib build/host-release/sim/quad_fc/libmarv_sil_quad_fc.so
python3 tools/target/hil_compare.py refdiff sticks.npz fma_sticks.npz
```

## What was checked without the board

The selftest (compare() on a match, a 1-count difference, a 2-count difference, an arming-tick mismatch, missing/extra
reports; the npz round trip, also read by numpy; the values CRC; the scenario's determinism; the record encoder). A loopback
through a pty to a stand-in board that parses the replay's records with the board's rules, runs the same library and answers
in the board's record formats gave HIL MATCH with 0 counts over both scenarios' streams (a stand-in: it shows the record
encoding, pacing, hello check and compare chain agree, nothing about the board).

## Board run

(The lead adds the image, the commands, `compare` output, and EMB-3 numbers here.)
