# Handoff: M4 on `m4` (7-inch card); logger U1-U2b, feed-forward, telemetry and GCS GX-0..GX-10 merged

2026-10-19 (evening). A state file, not a history: at most 80 lines, rewritten at every milestone boundary and before a
session ends. History is in git and `docs/decisions/`. The full session handoff is Serena memory
**`session-handoff/2026-10-19`** (local, not tracked: worktree paths, every Luis ruling verbatim, rules).

## Read first
- `CLAUDE.md`, quad spec §4.0 (M4), ground spec (ACTIVE, GCS-first scope, decisions 0051 and 0056).
- Designs with Luis's rulings at their top (Serena): `ground-segment/architect-gcs-first-class-2026-10-18` (the GX list
  and its order), `quad-m4/architect-onboard-logger-2026-10-18`, `quad-m4/architect-rate-response-2026-10-18`.

## Where things are
| Item | State |
| --- | --- |
| M0-M3 | merged into `master`; tags `quad-M1-pass`..`quad-M3-pass` |
| `origin/m4` | `03390bb (+ this docs/docstring commit)`, dev check PASS on that sha |
| Board | m4 flight image 4c5470c (banner quad-L2-pass-532-g4c5470c); free for agents (Luis: "it's all yours") |
| Luis's GCS | `~/marv-gcs` (detached worktree), `~/marv-gcs-start.sh`, `~/marv-gcs-update.sh` (fetch + rebuild under the lock) |
| Bench images | `~/marv-bench-images/` u3-c358806 (bench profile), u2-log-fbdad56 (log), u2b-*. Never `--delete-on-failure` there |

## On m4 now (this session)
- Feed-forward (0058): model following, |u_ff| <= k·tau_max_hover, k = 0.5 (Luis). tau_max_hover uses the PLACEHOLDER
  idle speed (f_min = 0): props-on ACRO blocked until idle speed is measured.
- Logger U1, U2a (bench log image, SD storage, tick hooks), U2b (soaks, power cut 12/12, EMB-3 with hooks C 81.09 us).
- U3 bench profile: fixed ceilings 450 / 40 s refused by validate, plus a clamp at the motor write (Luis: "Fixed + clamp").
- Telemetry 11a (GX-4): EMB-3 disarmed PASS at every cap (C 86.33 us, 110.5 of 150 kB/s). Armed: UNKNOWN.
- GCS: GX-0 e2e, GX-1 helper runs, GX-2 shell, GX-3 Simulation page, GX-5 live view, GX-6 R-1 (stick/arm/angle distinct),
  GX-7 radio page (PLACEHOLDERs CENTRE_TOLERANCE, MOVED_US), autoconnect (SIL wins over board), GX-8 sim channel (SC-1
  lockstep SHA equal; S-1 import check), GX-10 HIL from the GCS (Bench page: preflight, launch, verdict, SWD flash of
  the hil kind only), GX-9 pilot bridge (RadioMaster as USB joystick in SIL; R-2 T1 bit-identical).

## Next (in order)
1. Luis: CH8 mix on the RadioMaster (angle switch), then S4b in ANGLE launched from the Bench page
   (`tests/regression/quad/L10/results/gcs/gx10/RUNBOOK.md`); confirms the INFERRED "Board CDC" HIL-image check.
2. Luis: GX-9 bench check (`.../gcs/gx9/BENCH.md`: EdgeTX axis layout, joydev permissions) and GX-7 radio TB
   (replaces the two PLACEHOLDERs). Telemetry EMB-3 armed (radio arm, ESC unpowered, ~2 min).
3. Remaining GX units: GX-11 replay, GX-15 logs, GX-13 derive, GX-14 build, GX-12 lock editor, GX-16 console, GX-17/18.
4. Logger U4 (flight logger): EMB-6 open for the flight image; hooks + telemetry not measured together; unused 1 GiB
   files never truncated; HAL-pull kinds deferred.
5. Luis physical: bifilar pendulum (u2-log image, battery unplugged, TX off); props-off bench-profile TB (u3-c358806
   runbook, ch6 console check first); CG check; idle-speed measurement (then regenerate tau_max_hover).
- Flight gates: first hover needs S4b ANGLE + CG; no m4 flight image flies before an armed EMB-3.
- Pending Luis decisions from the old handoff: "check-4 stricter form", "ESKF board use" (ESKF tick may overrun EMB-3).

## Noticed, not changed
- `gcs/e2e/connection.spec.ts` test 2 fails live: test 1 never stops its SIL run, so "Start SIL run" is disabled.
- G1 clang-tidy does not cover target-only files; boot timing fault with the receiver streaming at boot.
- dev_check does not detect stale `gcs/node_modules` (run `npm ci`); TelemetryClient start/stop not refcounted;
  duplicate gating code in the Params and Radio pages; device.py's refusal text does not say a HIL run holds the port.
- Pilot (GX-9) flights leave no lockstep log (wall-clock frames): not replayable.

## Rules the next lead keeps
- Commit and push each unit that passed review and `ci/dev_check.sh` (read the result as a separate step first);
  merge into `master` after `ci/local_ci.sh`. Loosening tests, spec changes, tags and safety numbers stay Luis's.
- No ESC power or motor spin without Luis present. Flashing permitted. Real-time HIL only on an idle host.
- Every build/test under `ci/heavy.sh --wait 10800 -- <cmd>` in the background. Never `pkill -f` broad patterns (the
  scratchpad is shared). Check running workers before removing a worktree. Workers hit turn limits: resume them.
- Motor-capable bench images (bench-profile, bench-log) are refused from the GCS until their props-off TB on the exact
  image and Luis's presence acknowledgement.
