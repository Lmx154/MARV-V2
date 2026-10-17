# L07 results: the T2 bench IMU logs (M3 P7)

Evidence for quad spec 4.0 M3 done-when "a committed bench log (still, then rotated by hand) replays (T2) within the attitude
budget share". Raw captures of the FC3v2 IMU log; the replay is `tests/regression/quad/L07/replay` (`unit_l07_replay`,
case `L07Replay.CommittedBenchLogs`). `hover_kick/` is the separate L07 gz measurement record.

## Files

Each log is stored as `<name>.tar.xz` (a tar holding the one raw capture, byte-identical after extraction). `SHA256SUMS` holds
the sha256 of each extracted log (`sha256sum -c`). `tests/regression/quad/L07/replay/CMakeLists.txt` extracts every log listed
in `SHA256SUMS` into `<build>/tests/regression/quad/L07/replay/results/` at configure time (`file(ARCHIVE_EXTRACT)`) and fails
the configure if a sha256 differs; the test reads them from there (`MARV_L07_RESULTS_DIR`). By hand: `tar -xJf <name>.tar.xz`.

| log | raw bytes | archive | what |
| --- | --- | --- | --- |
| `t2_hand_rotation_2026-10-16.bin` | 22790320 | 4.6 MB | T2 evidence: 30 s still level, a hand rotation of about 138 deg, propped still (80 s capture). Replayed by the test. |
| `bench_still_30s_2026-10-16.bin` | 8545728 | 1.6 MB | 30 s still capture; the still gyro bias measurement. Not a T2 log (no rotation); not replayed. |

`still_bias.py` is the script of the still bias measurement below.

## How they were recorded

All on the FC3v2 board, 2026-10-16, with the bench IMU-log image `quad-L2-pass-188-g9a40ec7` (built with `MARV_FC3V2_IMU_LOG`),
USB CDC capture, battery unplugged, Luis present (he rotated the board by hand). Command, for the rotation log:

    python3 tools/target/imu_log.py capture <dev> 80 dryrun2_rotate.bin

(`dryrun2_rotate.bin` is `t2_hand_rotation_2026-10-16.bin`; `usb_still_30s.bin` was captured the same way for 30 s;
`dryrun_rotate.bin`, the first attempt, the same way for about 75 s, the exact argument not recorded.) Protocol of the rotation log: 30 s
still level, a hand rotation of about 138 deg, then the board propped still. The names above are the recorded names with the
date and purpose; contents are unchanged.

Decode and verdict of any log: `python3 tools/target/imu_log.py decode <file>` (add `--csv out.csv` for samples). All three:
CRC failures 0, sequence gaps 0, stamp gaps 0, logged-counter cross-check 0 mismatches, ring_overflows 0, `LOG VALID`; mean rate
7495 samples/s; durations 79.999 s (rotation), 29.999 s (still), 74.998 s (first attempt).

## Replay numbers

`ctest --preset host-debug -L L07` or `unit_l07_replay --gtest_filter=*CommittedBenchLogs`; printed (share = the attitude
budget share, kp from the FC3v2 parameter set):

    t2_hand_rotation_2026-10-16.bin: calibration completed 1.250 s after the first sample, bias (-0.1757, 0.3086, -0.1109) deg/s
    t2_hand_rotation_2026-10-16.bin: theta_rot 2.407222 rad, T_s required 18.986 s (kp 0.2500, share 0.02090 rad), still1 47.139 s, still2 23.572 s, rotation 7.288 s
      (a) err1 2.372e-04 err2 4.780e-04 rad (<= 0.02090): pass;  (c') err 1.640e-02 rad (<= theta_rot/2 = 1.20361): pass
    control (a): ...  (a) err1 2.372e-04 err2 1.721e+00 rad (<= 0.02090): FAIL;  (c') err 1.640e-02 rad (<= theta_rot/2 = 1.20361): pass
    control (c'): ... (a) err1 2.372e-04 err2 4.780e-04 rad (<= 0.02090): pass;  (c') err 1.462e+00 rad (<= theta_rot/2 = 1.20361): FAIL

The first attempt (not committed, to keep the repository small; its capture stayed in the session scratchpad), run once through the same code: rejected, `theta_rot is below pi/2`:

    rejected_first_attempt_2026-10-16.bin: theta_rot 0.002815 rad, T_s required -8.019 s (kp 0.2500, share 0.02090 rad), still1 32.657 s, still2 25.858 s, rotation 14.483 s

## Still gyro bias

    python3 tools/target/imu_log.py decode bench_still_30s_2026-10-16.bin --csv still.csv
    python3 still_bias.py still.csv

    samples 224840
    gyro mean (deg/s): x -0.1747 y 0.3060 z -0.0775
    mean specific force (m/s^2): x -0.1983 y 0.0347 z -9.7793, |a| 9.7814
    gyro mean along gravity 0.0821 deg/s, perpendicular to gravity 0.3513 deg/s

The 0.35 deg/s perpendicular to gravity is the bias the replay's `kBoardGyroBiasDegPerS` case is sized on; the boot calibration
of the rotation log measured (-0.1757, 0.3086, -0.1109) deg/s over its first still seconds.
