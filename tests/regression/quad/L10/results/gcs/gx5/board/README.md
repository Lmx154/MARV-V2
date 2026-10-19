# A board ground log for the GX-5 live view (S-1), flight image, every telemetry stream on

Bench evidence, not a frozen test. Board: FC3v2 Rev-C running the `m4` flight image (`marv_fc3v2`, banner
`quad-L2-pass-532-g4c5470c`, image hash `17bb03aa722ebdc49ce895465a57d1c2e995754914bb34b606d56300ca5acfae`, ELF sha256
`907fd4b7cc665a827d4686bfe8d05b848eea29a9f2c6a8963265f8760879592c`), disarmed (`arming=no_link`, no receiver bound), ESC unpowered,
board at rest on the bench (no rotation), no motor command, never armed. Date 2026-10-19. Firmware tree clean at `4c5470c`.

## Command

```
uv run python tests/regression/quad/L10/results/gcs/gx5/board/capture_ground_log.py <dir>/board
```

`capture_ground_log.py` starts the helper on the board's link port (`tools/gs/helper.py --serial <if02>`; the helper records the
ground log), connects to its WebSocket as the GCS does, reads the heartbeat, `MARV_IDENTITY` and the full parameter list (155 records), then
asks every one of the ten streams at an interval of 1 us (each is granted its cap of 198 Hz) and only reads for 60 s, stops the
streams and checks the heartbeat is still disarmed. It never arms or commands a motor.

## Files

- `board.ground.log.xz`: the helper's ground log (`tools/gs/groundlog.py` format, magic `MARVGL01`), xz -9e, 1797044 B; the raw
  log is 7580775 B (`xz -dc board.ground.log.xz > board.ground.log`; round trip compared with `cmp`).
- `board.ground.log.info.txt`: `uv run python tools/gs/groundlog.py board.ground.log --info` on the decompressed log.
- `board.json`: the script's results (identity, the stream statistics of the 60 s, the helper's start-up lines);
  `board.console.log`: the console (if00) during the capture; `capture_ground_log.py`: the script.

## Contents (`--info`)

```
rx: 71100 records, 6654431 bytes
tx: 32 records, 1378 bytes
event: 4 records, 174 bytes
```

Decoded with the MARV dialect (pymavlink, the m33 build's `marv.xml`), the received bytes hold: HEARTBEAT 68, COMMAND_ACK 31,
PARAM_VALUE 155, MARV_PARAM_RECORD 155, MESSAGE_INTERVAL 10, and about 11883 of each of the ten stream messages (SYS_STATUS 11888,
ATTITUDE 11885, ATTITUDE_QUATERNION 11884, RC_CHANNELS 11884, ATTITUDE_TARGET 11884, HIGHRES_IMU 11883, ACTUATOR_OUTPUT_STATUS 11883,
MARV_FC_STATUS 11883, MARV_ESTIMATOR 11882, MARV_IDENTITY 11883). The 60 s of streams: 118849 frames, 6629688 B (110.5 kB/s), 0
sequence gaps, every stream granted 5051 us (197.98 Hz) and received at 197.98 Hz by the board's stamps (`board.json`).
The vehicle shows `est_valid=1`, `arming=no_link`; `MARV_IDENTITY` config source 0 (factory), state 0 (OK), refusal 0 (none), validation report id all zero.

What the values are (last message of each stream): a board at rest, disarmed, no receiver. ATTITUDE roll -0.026 rad, pitch -0.003 rad, yaw
-0.004 rad (the board lies slightly tilted; tilt 0.0263 rad on the console); HIGHRES_IMU accel (0.005, 0.258, -9.796) m/s^2, temperature
28.63 degC; RC_CHANNELS `chancount` 0, every channel 65535 and `rssi` 255 (no receiver); HEARTBEAT `system_status` 3 (STANDBY), not
armed; SYS_STATUS battery fields 65535 / -1 (no battery sensor); ACTUATOR_OUTPUT_STATUS all zero (motors off). Panels that depend on a
receiver, a battery or motion show these at-rest or unavailable values.

## How the GX-5 vitest could load it (branch `gx5-live`, not edited)

`gcs/src/lib/state/live_state.test.ts` ("the same panels from a ground log") builds a log in memory with `buildGroundLog` and feeds
`readGroundLog(log)` chunks to a `RecordedLink` behind a `LinkSession` and a `TelemetryClient`, setting the clock to each chunk's `atMs`.
The same test body can read this file instead: decompress it once (`xz -dc` into a temp file, or `execFileSync('xz', ['-dc', path])`
in the test), pass the bytes as a `Uint8Array` to `readGroundLog`, and loop `clock.t = c.atMs; link.push(c.bytes)` as the existing test
does; then `computeLive(telemetry.snapshot(), end)` gives the panels, and the assertions become predicates on a real vehicle
(every panel `heard` and not stale at the end; stale a timeout later; `attitude` stamp count about 11880; `arming` shows disarmed,
`imu` temperature about 28.6 degC from `HIGHRES_IMU`). `readGroundLog` returns only the received chunks (kind 1) with `atMs` on the log's
monotonic clock (about 6e11 ms here, not zero-based), so the clock origin differs from the test's `T0`; the log's 60 s of streams
also carries a parameter listing and the request replies at the start. The stale-control check needs the `end + STALE + 1` read, as in the
existing test.
