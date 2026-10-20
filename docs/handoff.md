# Handoff: M4 on `m4` (7-inch card); flight logger ON (gaps OK), GCS tuning, replay, derive, build, console, logs

2026-10-20 (evening). A state file, not a history: at most 80 lines, rewritten at every milestone boundary and before a
session ends. History is in git and `docs/decisions/`. Full session handoff: Serena memory **`session-handoff/2026-10-19`**
(local, not tracked: worktree paths, every Luis ruling verbatim).

## Read first
- `CLAUDE.md`, quad spec §4.0 (M4), ground spec (ACTIVE, GCS-first scope, decisions 0051, 0056).
- Serena designs with Luis's rulings on top: `ground-segment/architect-gcs-first-class-2026-10-18` (GX list),
  `quad-m4/architect-logger-u4-2026-10-20` (U4), `quad-m4/architect-rate-response-2026-10-18`.
- M4 flight test card DRAFT `tests/flight/quad/M4/card.md`: gates G-1..G-13; section 6 = Luis's decisions U-1..U-14.

## Where things are
| Item | State |
| --- | --- |
| M0-M3 | merged into `master`; tags `quad-M1-pass`..`quad-M3-pass` |
| `origin/m4` | code at bd5c39c (full dev check PASS, 92 steps) + docs commits |
| Board | old flight image 4c5470c; flash the current one before GCS work (parameter schema changed) |
| Luis's GCS | `~/marv-gcs` (at origin/m4, built), `~/marv-gcs-start.sh` (prints the link), `~/marv-gcs-update.sh` |
| Bench images | `~/marv-bench-images/` u3-c358806 (bench profile), u2-log-fbdad56 (log). Never `--delete-on-failure` there |

## On m4 (2026-10-19/20)
- Logger U4-1..U4-5 and decision 0060: `MARV_FC3V2_FLIGHT_LOG` ON by default; L-5 on the flight image is a WARN (ring
  135 KB = 0.70 s of the v2 stream; a longer SD stall leaves Gap records); files reused, never deleted.
- One FC3v2 parameter set incl. log params: schema 0x0fd34ce78b31e33f -> 0xd1eb1dc8320b077d (old configs refused).
  Bench-profile image builds only with FLIGHT_LOG=OFF (CI builds it so; its hash changed -> new props-off TB to use it).
- GCS: Tuning page (rate PID+FF, angle, motor/mixer with confirm, filters; refused while armed), GX-11 replay (Logs
  page), GX-13 derive, GX-14 build, GX-15 logs, GX-16 console. Pilot bridge (GX-9): RadioMaster as USB joystick ->
  helper -> CRSF-format frames on UDP to the SIL plugin -> the board's own CRSF decoder (SIL only).

## Next (in order)
1. Luis (manual, from the UI): sim walk-through (Simulation page: card, overrides, GUI, Pilot, Launch; stop; Logs ->
   Replay); a Tuning change; CH8 mix then S4b in ANGLE from the Bench page (gx10 RUNBOOK); GX-9 BENCH.md; GX-7 radio TB.
2. Luis at the bench, ESC unpowered: armed EMB-3 on the current flight image (decision 0060's gate); CG and weighing;
   props-off bench-profile TB; idle speed then regenerate tau_max_hover (gates props-on ACRO).
3. Luis rules the card's section 6; then tethered hover in ANGLE, free hover, ACRO, each log + card update committed.
4. After first flight: U4-6 soaks + U4-7 budget (restore L-5); U4-X (XIP move) only if needed; GX-12, GX-17, GX-18.

## Testing methodology (Luis, 2026-10-20; keep it)
- **Workers run targeted checks only**: the unit tests, the one image and the one e2e spec their change touches, plus
  the gate scripts it reaches (G1, S-1 `tools/ci/check_s1.py`, GS-3 `tools/ci/check_gs3.py`). No `ci/dev_check.sh`, no
  full builds: "there is no reason to build everything" unless pushing.
- **One full `ci/dev_check.sh` per push**, on a batch of reviewed units merged together; read its exit code, the
  `^PASS` count (92) and `FAIL:` lines as a separate step, then `git push origin <sha>:m4`. A docs-only commit on a
  checked tree needs no new check. `ci/local_ci.sh` (fresh clone, frozen suites) is the gate for a merge into `master`.
- **Review against the spec**: every code unit goes to a `reviewer` with the original acceptance criteria (not the
  coder's summary); the lead accepts. Thresholds and CI gates carry negative controls; new logic carries mutants that
  must fail a test; a found defect gets a test that fails before the fix.
- **UI**: no manual UI testing by agents. Automated only: vitest, `npm run check`, Playwright via `tools/gs/e2e.sh`
  (light = fake vehicle; `--live` = real gz SIL, heavy). Luis does the hands-on UI and bench testing.
- **Board**: agents may flash and run disarmed, ESC unpowered; anything with ESC power or motors waits for Luis.
  Bench and flight runs commit script, raw log and card update.
- **Load**: never two heavy jobs in one worktree. A dev check runs on the default lock at normal priority
  (`ci/heavy.sh --wait 10800 -- ci/dev_check.sh`); other heavy jobs may run beside it only with >= 8 GiB free RAM, as
  `MARV_HEAVY_LOCK=/tmp/marv-heavy-slot2.lock nice -n 15 ci/heavy.sh --wait 10800 -- <cmd>` (slot2/slot3). Un-niced
  parallel jobs made ctest timeouts and gz real-time tests flake (load 25-40 on 16 cores).
- **Waiting**: a Haiku `runner` waits on long scripts and reports exit/PASS/FAIL; the lead never polls.
- **Cost driver**: `ci/blast_radius.yaml` and any `CMakeLists.txt` are full roots, so such a change runs all 92 steps.

## Noticed, not changed
- No battery-voltage read in the firmware (no low-battery failsafe); armed zero throttle idles at DShot 48 (the kill
  is ch5 disarm). Both are in the card for Luis.
- The GCS manifest has no min/max/enum (Params/Tuning leave range checks to the vehicle); Params/Radio/derive keep
  their own gating copies; dev_check never builds the log-bench or HIL images; G1 skips target-only files.

## Rules the next lead keeps
- Loosening tests, spec changes, tags and safety numbers are Luis's. Never rewrite pushed history.
- Never `pkill -f` broad patterns (the scratchpad is shared); check running workers before removing a worktree.
- Tuning is the user's job in the UI; agents build the UI and the safety around it, not the gains.
