# GX-10 runbook: S4b launched from the GCS (Luis flies; not run yet)

Luis's rulings of 2026-10-18: "HIL preflight + bench images by SWD, motor-capable bench image GCS button only after its props-off TB".
This is the bench run that closes GX-10's "S4b launched from the GCS with Luis flying, report and verdict stored". Everything the
helper can check without the board is tested (tests/tools/test_gs_hil.py, tests/regression/quad/L10/gz/test_hil_port_lock.py, the light
e2e in `light/`); nothing here has touched a board. S4b is HIL stage S4 with the radio on (`--expect none`): evidence of the closed loop
on the real card's INFERRED/PLACEHOLDER values, not a flight result (tests/regression/quad/L10/results/s4/README.md).

Needs: the board on USB with the Debug Probe on SWD (probe serial `900678EF479C3A0E`), Luis at the bench with the RadioMaster (its
signal must reach the board's receiver), **the ESC battery unplugged**, no other heavy job on the host (the run is real time; check
`uptime`, load below about 4), gz-sim 8, Node and `cd gcs && npm ci` done once.

## Before the GCS (the helper never builds)

1. Build the HIL receiver image and the plugin, under the heavy lock, from the repository root:
   `ci/heavy.sh --wait 10800 -- bash -c "cmake --preset m33 -B build/m33-hil -DMARV_FC3V2_HIL=ON -DMARV_FC3V2_HIL_RECEIVER=ON && cmake --build build/m33-hil -j 8 --target marv_fc3v2 && cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc && (cd gcs && npm run build)"`
   The GCS flashes only `build/m33-hil/fw/hal/target/marv_fc3v2.elf`, and only when `build/m33-hil/CMakeCache.txt` has
   `MARV_FC3V2_HIL=ON` and `MARV_FC3V2_HIL_RECEIVER=ON`.
2. Start the helper (it prints the GCS address with the session token):
   `MARV_PROBE_SERIAL=900678EF479C3A0E uv run python tools/gs/helper.py`
   and open that address in the browser. `MARV_PROBE_SERIAL` is what the preflight's probe check and `flash.sh` / `run_hil_s4.py` use.

## On the Bench page

3. Flash the HIL image. If the helper has the board's link open (Connection page, or autoconnect did), press **Hold the board** first: it
   closes the helper's serial port and keeps autoconnect off the board. Then Image kind **hil**, **Flash over SWD**. Wait for the
   SWD flash panel to read `done`; its output ends with `** Verified OK **` and it shows the ELF's SHA-256 (record it). Selecting
   bench-log or bench-profile and pressing Flash must be refused with "only after its props-off TB on the exact image and Luis's
   presence acknowledgement": that is the expected answer in this unit.
4. **Unplug the ESC battery.** Tick **ESC battery unplugged**. The HIL image drives no motor pad, but the acknowledgement is asked for on
   every launch.
5. Radio on, ch5 (arm) down, ch8 (angle) up, throttle low.
6. Press **Hold the board** (if it is not already held), then **Preflight**. All six checks must read `ok`: Debug probe, Board, HIL
   image, ESC battery unplugged, Host idle, Board ports closed and autoconnect suspended. A `refused` check names its reason; fix that
   and press Preflight again. (The board takes about 12 s to show its USB port after the flash: ModemManager.)
7. **Launch S4b.** It runs `tools/sim/run_hil_s4.py --port <the board's port> --out-dir <run dir> --seconds 300 --expect none --gui`
   (300 s is the S4b runbook's length; the Seconds field changes it). The runner reboots the board through the probe, waits about 12 s,
   and opens the gz window. Watch the live line and the SIM page (board arming state, receiver frames, DShot, height, tilt).
8. Wait for the gyro calibration (sticks still), then arm on ch5, raise the throttle past hover (about 35 % stick) to lift off the pad,
   fly gently in **ANGLE**, keep it low. To end: disarm on ch5, then press **Stop** (or let the 300 s run out).
9. In the **Verdict** panel choose the flight mode flown (ANGLE or ACRO), write what happened, press **Save verdict**. It is written
   once and is not overwritten.

## Where things land

Everything of the run is under the helper's run directory, printed by the helper at start as `run dir:` (default
`build/gs-runs/<UTC stamp>/`), in `runs/<run id>/`:

- `manifest.json`: the argv, options, start and end, exit status;
- `out/<world>_hil_n25.hil_report.txt`: the run's report (also printed in the run's output), with the world, the gz log, the board's uplink
  bytes (`.uplink.bin`), the frame and report timing, and gz's stderr beside it;
- `out/verdict.json`: the verdict, next to the report: `{schema, run_id, mode, text, saved_utc, report}`;
- `launcher.log` in `out/`: the run's output (the live status lines).

The Bench page shows the same files' directory under Run. Commit as evidence (a TB log counts only with its script and raw output):
the report, `verdict.json`, `manifest.json`, the ELF's SHA-256 from step 3, and a line for the card update (the plugin's SHA-256 of
the gz log goes in the README, the 225 MB log itself is not committed).

## Afterwards

10. Nothing of the run is left: the Run indicator reads `hil stopped` or `hil exited` and `pgrep -af 'gz sim|run_hil_s4'` prints nothing.
    The hold ended with the run and autoconnect is back (the Hold button reads "Hold the board" again).
11. Flash the flight image back (the HIL image has no link, so the GCS flash page cannot see the board):
    `MARV_PROBE_SERIAL=900678EF479C3A0E tools/target/flash.sh build/m33/fw/hal/target/marv_fc3v2.elf` (after
    `cmake --preset m33 && cmake --build --preset m33` under `ci/heavy.sh`), and check `est_valid=1` on the console.
