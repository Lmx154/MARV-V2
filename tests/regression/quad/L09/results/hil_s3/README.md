# HIL S3a smoke on the FC3v2 (decision 0046 stage S3), 2026-10-17

Image: bench-only `MARV_FC3V2_HIL` variant `quad-L2-pass-267-g3ac0d4b` (sha256
`28cd9f195979cef7a70b0539683c3078e60cd58218cceb0edf099b6115197b08`): quad_fc with the unchanged fc3v2 table runs from
USB-delivered IMU samples and ManualControl frames in a software interrupt at the IMU priority; no IMU, receiver or
DShot program is started (no motor pad driven); battery unplugged; Luis not present.

Command: `python3 tools/target/hil_fc.py smoke /dev/ttyACM0 smoke.bin` (still level IMU at the fc3v2 tick, 4 samples per
record; arm switch high from 3 s to 6 s of stamp time; stream ends at 7 s). Output `smoke.out`, raw uplink
`smoke.bin.xz` (`SHA256SUMS` = uncompressed).

Result: SMOKE OK. Hello: schema hash 0x2ff36d1da5fc3596, 136 parameters, values CRC 0x97ed61ec, tick 1953125/14649 µs,
D_HIL 14532 µs. 52 500 ticks, 26 250 rate executions, 0 rate timing faults, 0 attitude faults, 0 link errors either way;
arming calibrating → switch_off → armed (DShot 48 on all four motors) → switch_off; one IMU deadline miss after the stream
stopped. Longest record of 4 ticks 96.1 µs against 533.3 µs; longest tick 34.5 µs.
