# Handoff: M4 on `m4` (7-inch card); flight logger ON (gaps OK), GCS tuning, replay, derive, build, console, logs

2026-10-20 (evening). A state file, not a history: at most 80 lines, rewritten at every milestone boundary and before a
session ends. History is in git and `docs/decisions/`. The full session handoff is Serena memory
**`session-handoff/2026-10-19`** (local, not tracked: worktree paths, every Luis ruling verbatim, rules).

## Read first
- `CLAUDE.md`, quad spec §4.0 (M4), ground spec (ACTIVE, GCS-first scope, decisions 0051 and 0056).
- Serena designs with Luis's rulings at their top: `ground-segment/architect-gcs-first-class-2026-10-18` (GX list),
  `quad-m4/architect-logger-u4-2026-10-20` (U4), `quad-m4/architect-rate-response-2026-10-18`.
- The M4 flight test card DRAFT: `tests/flight/quad/M4/card.md` (section 6 = Luis's decisions U-1..U-14).

## Where things are
| Item | State |
| --- | --- |
| M0-M3 | merged into `master`; tags `quad-M1-pass`..`quad-M3-pass` |
| `origin/m4` | see `git log origin/m4`; dev check PASS on the pushed sha |
| Board | m4 flight image 4c5470c (old); flash the current flight image before GCS work (schema changed, below) |
| Luis's GCS | `~/marv-gcs` (detached), `~/marv-gcs-start.sh` (prints the link; port 8765 or a free one), `~/marv-gcs-update.sh` |
| Bench images | `~/marv-bench-images/` u3-c358806 (bench profile), u2-log-fbdad56 (log). Never `--delete-on-failure` there |

## On m4 now (this session, 2026-10-19/20)
- Logger: U4-1 (link source in place, -64 KB), U4-2 (disarmed arena + lease, one FatFs owner), U4-3 (v2 16-B IMU),
  U4-4 (flight wiring), U4-5 (file reuse, never deletes), and decision 0060: `MARV_FC3V2_FLIGHT_LOG` ON by default,
  L-5 on the flight image is a WARN (ring 135 KB = 0.70 s of the v2 stream; a longer SD stall leaves Gap records).
- One FC3v2 parameter set now includes the log parameters: schema 0x0fd34ce78b31e33f -> 0xd1eb1dc8320b077d. Configs
  bound to the old schema are refused. The bench-profile image builds only with FLIGHT_LOG=OFF (CI builds it so).
- GCS: GX-11 replay (Logs page), GX-13 derive, GX-14 build, GX-15 logs, GX-16 console, and the Tuning page (rate PID
  + FF, angle, motor/mixer with explicit confirm, filters; refused while armed).

## How Luis runs the sim from the UI
1. `~/marv-gcs-update.sh` (once per update; builds the UI and the SIL), then `~/marv-gcs-start.sh`; open the link.
2. Simulation page: pick a card (or the default), optional SIL-only parameter overrides, tick GUI to see Gazebo,
   tick Pilot to fly with the RadioMaster (USB joystick; open it in the Pilot panel), press Launch.
3. Autoconnect opens the run's link: Vehicle, Radio, Params and Tuning pages show the SIL vehicle; SIM page shows truth.
4. Stop on the Simulation page. Logs page -> Replay plays back the run's ground log.

## Next (in order)
1. Luis (manual): the UI walk-through above; CH8 mix then S4b in ANGLE from the Bench page (gx10 RUNBOOK);
   GX-9 RadioMaster bench check (`.../gcs/gx9/BENCH.md`); GX-7 radio TB.
2. Luis at the bench: armed EMB-3 on the current flight image (logger hooks + telemetry; decision 0060's gate),
   CG and weighing, props-off bench-profile TB, idle speed then regenerate tau_max_hover (gates props-on ACRO).
3. Luis rules the flight card's section 6 (tether, battery limits, abort thresholds, pass bars).
4. After first flight: U4-6 soaks and U4-7 budget (restore L-5), U4-X (XIP move) only if needed; GX-12, GX-17, GX-18.
- Flight gates: S4b ANGLE, CG, armed EMB-3 on the exact image, the card's gates G-1..G-13.

## Noticed, not changed
- No battery-voltage reading in the firmware (no low-battery failsafe); armed zero throttle idles at DShot 48 (the kill
  is the ch5 disarm). Both are in the card for Luis.
- The GCS manifest has no min/max/enum, so the Params/Tuning pages leave range checks to the vehicle's refusal.
- Params/Radio pages and derive keep their own gating copies (Tuning uses `gating.ts`); dev_check never builds the
  log-bench or HIL images; G1 does not cover target-only files; TelemetryClient start/stop not refcounted.
- `ci/blast_radius.yaml` and any `CMakeLists.txt` are full roots: such a change runs the full ~92-step check.

## Rules the next lead keeps
- Workers run only targeted checks (the tests and images their change touches); the full `ci/dev_check.sh` runs once
  before a push, on a batch (Luis, 2026-10-20). Haiku runners wait on scripts; the lead does not poll.
- Heavy jobs via the scratchpad `par.sh` (RAM check; dev_check on the default lock, others niced on 2 slots).
- No manual UI testing by agents; automated e2e only (Luis). Tuning is the user's job in the UI, not the agents'.
- No ESC power or motor spin without Luis present. Loosening tests, spec changes, tags and safety numbers are Luis's.
- Never `pkill -f` broad patterns; check running workers before removing a worktree.
