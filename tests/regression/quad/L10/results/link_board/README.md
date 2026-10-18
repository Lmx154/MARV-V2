# The ground link on the FC3v2 board, ground-segment unit 12b (decision 0053)

Bench evidence, not a frozen test. Board: FC3v2 Rev-C, SD card in, ESC unpowered, never armed, no motor command. Date 2026-10-18.
Code: branch `gs-link-board`, firmware commit `a4f5338` (banner `quad-L2-pass-395-ga4f5338`). Flight image
`build/m33/fw/hal/target/marv_fc3v2.elf` sha256 `2d6cc936...835f`; bench-stats variant (`-DMARV_FC3V2_BENCH_STATS=ON`,
`build/m33-bs`) sha256 `0ef4603e...f5aa`. Flashed with `tools/target/flash.sh` through the Debug Probe, 12 s wait after each reboot.

## Files

- `link_common.py`: the PC side (dialect from the m33 build's `link/marv.xml` by pymavlink's mavgen; the helper
  `tools/gs/helper.py --serial <if02>` and its WebSocket, or the serial port directly; console reader from `../usb/usb_common.py`).
- `bench_link.py`: done-when (a) to (d) through the helper, PASS/FAIL per step. `bench_load.py`: (e), capture and analyze.
  `armed_runbook.py`: the PC side of the armed step (below).
- `raw/bench_link.{json,console.log,ground.log}` (flight image; `ground.log` is the helper's ground log of the session),
  `raw/load_idle_disarmed.{console.log,phases.json,analysis.txt}` (bench-stats image), `raw/flight_final.log` (flight image
  flashed back, 15 s of console).

## Results (disarmed)

| Done-when | Result |
| --- | --- |
| link | Heartbeat 1 Hz, not armed (system_status STANDBY); AUTOPILOT_VERSION capabilities 0x2010 (byte-wise parameter encoding, MAVLink 2); MARV_IDENTITY: factory, OK, refusal none, schema 0x3ae00eb50938acba, config hash `625f509b...003d` = the factory .mcfg's, image hash all zero (unknown until unit 14), staged_differs 0. |
| (a) list | Through the helper: 137 PARAM_VALUE + 137 MARV_PARAM_RECORD (manifest count 137); every link id, type, value bits, active/staged bits and lock equal to the manifest and `marv_params_fc3v2.mcfg`; 0 sequence gaps; 274 frames in 0.038 s. |
| (b) stage | `att_kp` one ulp up (0x4011145e -> 0x4011145f): echo and read by index show 0x4011145f staged, record active 0x4011145e (unchanged), origin Manual, method Scenario, sigma kind Choice; identity staged_differs 1. `rotor_speed_max` (locked): status text "the parameter is locked: it keeps its compiled value", echo unchanged. Staging writes RAM only: `store::stage_set` (fw/store/src/staging.cpp) changes the staged copy; the pending file is written only by `store::activate`, so nothing touched the SD card. |
| (c) activate | Zero report id: DENIED_REPORT ("no validation report id ... (G7)"). A nonzero report id with the staged config's hash: FAILED, "the config is not bound to this image's hash and a validation report" (the board's image hash is all zero until unit 14, so `bound_to` refuses before the SD is written). Revert: DENIED_UNAVAILABLE ("there is no last good stored config"). Factory reset: FAILED ("this board cannot keep a boot request": `hal_config_request_set` is the unit-14 stub). Afterwards 3 heartbeats in 3 s, console `t` advancing, est_valid=1, identity factory and last activation none. None of these asks for the maintenance grant (the store asks only after the boot request is kept). |
| (d) reset | `picotool reboot -f` without the grant: picotool rc 0 ("asked to reboot into application mode"), board kept running (heartbeats, `t` advancing), `usb_reset_refused` 0 -> 1. **Maintenance request: not done** — the link has no maintenance-request message (see "Open"). |
| (e) timing | Bench-stats image, disarmed (no radio: arming `no_link`), `raw/load_idle_disarmed.analysis.txt`. Link load = parameter listings without pause (the service's largest output; no telemetry stream exists, see "Open"): 437.7 kB/s, 6438 frames/s for 60 s, 0 sequence gaps, heartbeats 60. Interrupt-entry spacing window max: port closed 134.12 us, open (heartbeat only) 134.11 us, load 134.19 us, after 134.05 us; window min 132.63..132.77 us. IMU interrupt (`irq`) window max 74.84 / 74.84 / 74.85 / 74.86 us. USBCTRL chain window max 78.6 us idle, 82.15 us under load. EMB-3 over the whole capture: C = 79.69 us, T = 133.41 us, U = 0.597 (PASS, margin 40.3 % of T); RTA R = 80.46 us vs D = T: PASS. Rate and attitude timing_faults 0, IMU ovr/spi/partial/clamp/order/empty 0, est_valid=1. |
| console | `../usb/fields_diff.py raw/flight_final.log ../usb/raw/flight_final.log`: 46 fields each, none added or removed, same order, no other line template differs: PASS. Flight image left on the board: est_valid=1, rate timing_faults 0, attitude timing_faults 0, usb_reset_refused 0. |
| (f) GUI | Not rendered in a browser. The GUI's own path was exercised instead: `bench_link.py` is a WebSocket client of the helper on the board's link port, exactly the bytes the GCS would get (`raw/bench_link.ground.log`, `raw/bench_link.json` "list"). |

Rate caps in force (stated, none added): the heartbeat at 1 Hz (`kHeartbeatPeriodUs`, the protocol convention); a parameter
listing sends one parameter (two frames) per service step, and on the board only while the send queue has room for both
(`Transport::writable`, new); everything else is an answer to a request. No stream-rate parameter exists.

## Armed step (lead with Luis; props off, ESC unpowered)

`uv run python tests/regression/quad/L10/results/link_board/armed_runbook.py <dir>/armed` on the bench-stats image, then
`bench_load.py analyze <dir>/armed` for the EMB-3 numbers. The script asks for each precondition, tells Luis when to arm and
disarm, and prints per step: disarmed baseline; arm; armed refusals (parameter write refused "only while disarmed", activation
DENIED_ARMED); maintenance while armed (**BLOCKED** until the link has a maintenance request); `picotool reboot -f` refused
(usb_reset_refused +1, still armed); the link at its largest output for 60 s armed; disarm; maintenance while disarmed
(**BLOCKED** likewise; then granted, arming=maintenance, `picotool reboot -f` honoured).

## Open (for the lead)

- No maintenance request exists on the link (not in the SIL either): `composition::request_maintenance` is reached only from
  the store's commands once a boot request is kept, which the board cannot do until unit 14. Done-when (d)'s grant and the
  runbook's maintenance steps need a message chosen by the lead (for example COMMAND_LONG MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
  which is in the scope already, or a dialect action).
- No telemetry stream exists: `link/scope.txt` carries no attitude, rate, motor or RC message, and the SIL streams only the
  heartbeat. The tick-path seqlock snapshot was not added (it would have no reader). EMB-3 above is under the largest link
  output that exists.

## Notes

- `motor_test refused: unknown command (console)` appears once, right after the console host first opens if00 with
  `usb_common.open_raw` (both `bench_link.console.log` and `load_idle_disarmed.console.log`), and not with
  `tools/target/bench_stats.py capture` (`flight_final.log`). INFERRED: the host tty echoes the banner back before raw mode
  is set. Console code is unchanged by this unit.
- RAM: linking the service pulls the store's staging into the image: `store::g_sources` 131072 B (kConfigSlotBytes, a
  PLACEHOLDER) and the service state 65800 B (its source buffer holds the longest text the link carries, 65535 B). Flight
  image text 218652 B, bss 223364 B; bss ends at 0x2006ac84, 87 KB below the heap limit 0x20080000.
