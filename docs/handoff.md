# Handoff: M4 bench gates on `m4`; stopped before the AM32 step (Luis, 2026-10-27)

2026-10-27. A state file, not a history: at most 80 lines, rewritten at every milestone boundary and before a session
ends. History: git and `docs/decisions/`. Older notes: Serena `session-handoff/2026-10-19` (its /tmp worktrees are gone).

## Owner redirects (verbatim)
- Luis, 2026-10-26: "don't get stuck testing like crazy and banging your head on the same problem. this has been happening
  for a while we need fast progress even if things are broken as long as we flag them we can check them later on."
  Lead's reading: a failure that does not gate the next hardware step is flagged (here or in the card) and work moves on;
  one correction attempt per failure, then flag. Safety numbers and props-on gates stay Luis's.
- Luis, 2026-10-26: "i really don't like how this is a script first process. a gui would be nice because having to type
  huge amounts of text is not fun". Practice: agents run every command that needs no hands; Luis gets GCS clicks.

## Read first
- `CLAUDE.md`, quad spec §4.0 (M4), ground spec (ACTIVE, GCS-first). The M4 card `tests/flight/quad/M4/card.md`:
  section 1 gates, section 6 Luis's rulings. Serena designs: `quad-m4/architect-rate-response-2026-10-18`, `-logger-u4-`.

## Where things are
| Item | State |
| --- | --- |
| M0-M3 | merged into `master`; tags `quad-M1-pass`..`quad-M3-pass` |
| `m4` | main checkout is on `m4` = `origin/m4` (no longer `m0-regen-line`); code as bd5c39c (dev check PASS, 92 steps) + docs, card and bench-evidence commits |
| Board | flight image built in `~/marv-gcs` (f412791; same firmware as 3531c3f): banner `quad-L2-pass-662-gf412791`, elf 221050b8…110a, image 52b78dac…d9bb, logger ON, timing_faults 0. Config slot A seq 1 (37a48a20…), validated + activated from the GCS: `yaw_deadband` 0.07 |
| GCS | `~/marv-gcs` (f412791). App-menu entry "MARV Ground Station" (`~/marv-gcs-open.sh`: probe serial set, opens the browser). The helper started 2026-10-27 may still run (terminal of that name). Its image library sees only images built in `~/marv-gcs` |
| Built trees | `~/marv-gcs/build`: m33 (on the board), m33-hil (HIL S4b, elf af1c1d3f…a7c4), host-gz-quadfc, GCS bundle. Main repo `build/m33` (3531c3f, superseded), `build/m33-bs` (bench stats, unused) |
| Branches | `m1-wdt`, `u4-6a-measure` pushed to origin as backup; `l6-c6a-wip` parked; stale /tmp worktree records pruned |

## Card gates (section 1) on 2026-10-27
- DONE: G-1 CH8 angle switch; G-3 CG (Luis, by hand); G-4 weighing (the card's 0.732 kg stands); G-12 image above; G-11 not
  needed on a factory or GCS-validated config (U-3). Stick play measured: `tests/flight/quad/M4/bench/stickplay_2026-10-27/`.
- OPEN, Luis: G-10 AM32 KV 1700 / 14 poles; G-2 HIL S4b in ANGLE; G-5 armed fault-free run (decision 0060 amendment: GCS
  streams at cap, logger on, about 2 min, ESC unpowered; pass = stays armed, timing_faults/motor_drops/ovr unchanged); G-6
  props-off angle motor-direction check + failsafe re-walk (TX off: idle at about 300 ms, stop at 1.5 s); G-13 tether.
- Before (c) ACRO only: G-7 idle speed -> `tau_max_hover`, G-8 pendulum/tau/thrust map, full bench-profile TB, G-9.
- Rulings: U-1..U-14 "all yes" (2026-10-26); "1. yes 2. recommended" (failsafe re-walk; lift-off tip is a visual call);
  "A" (EMB-3 as the armed fault-free run). Still to ask before (b): free-hover area size and clearance.

## Next (in order; stopped before 1)
1. Luis: close the GCS window; flash Betaflight (MADFLIGHT_FC3; 2026.6.2 in M1) from the Betaflight app; props off,
   battery in; esc-configurator passthrough: KV 1700, poles 14 on all four, motor direction untouched (ESC 1 and 4 are
   reversed on purpose, `tests/regression/quad/L09/results/m1-bench-card.md:63-71`); screenshots; battery out.
2. Luis: reopen the GCS; S4b from the Bench page (`tests/regression/quad/L10/results/gcs/gx10/RUNBOOK.md` steps 3-9). The
   HIL image boots the same config store (`fw/hal/target/src/main.cpp:715,732`), so a config bound to the flight image
   refuses arming there (INFERRED, untested; Betaflight probably erased it). If refused: Params, Reset to factory.
3. Agent: SWD flash `~/marv-gcs/build/m33/fw/hal/target/marv_fc3v2.elf` (`MARV_PROBE_SERIAL=900678EF479C3A0E
   tools/target/flash.sh <elf>`), banner by `tools/target/readcon.py <board if00> 3`. Luis: Params, stage `yaw_deadband`
   0.07, Validate (about 2 min: records, composition, ctest, Gazebo SIL), Activate; confirm `config: source slot` on console.
4. G-5, G-6, then (a) tethered hover in ANGLE; log + card update committed per run (card section 5); then (b).
5. Right after (a), Luis 2026-10-27 "A": parameter changes apply like other firmware (Activate without a report; boot
   checks type/finite/lock/composition stay; full validation optional). Changes core G7 and ground GS-11: decision
   record quoting him, firmware (arming NotBound) and GCS change, tests updated with his approval.
6. After first flight: U4-6 (incl. the measured combined EMB-3) and U4-7; ACRO yaw deadband (`fw/stick_rates` has none);
   U4-X only if needed; GX-12, GX-17, GX-18.

## Testing methodology (Luis, 2026-10-20; keep it)
- Workers run targeted checks only (the unit tests, image and e2e spec their change touches, plus gate scripts G1,
  `tools/ci/check_s1.py`, `tools/ci/check_gs3.py`); no full builds unless pushing.
- One full `ci/dev_check.sh` per push of a reviewed batch: read exit, `^PASS` count (92) and `FAIL:` as a separate step,
  then `git push origin <sha>:m4`. Docs-only commits on a checked tree need no check. `ci/local_ci.sh` gates `master`.
- Every code unit goes to a `reviewer` with the original acceptance criteria; thresholds and gates carry negative
  controls; new logic carries mutants; a found defect gets a test that fails before the fix.
- UI: agents test only automatically (vitest, `npm run check`, `tools/gs/e2e.sh`); Luis does hands-on UI and bench.
- Board: agents may flash and run disarmed, ESC unpowered; ESC power or motors wait for Luis. Commit script, raw log, card.
- Load: one heavy job per worktree; dev check `ci/heavy.sh --wait 10800 -- ci/dev_check.sh`; others beside it only with
  >= 8 GiB free, `MARV_HEAVY_LOCK=/tmp/marv-heavy-slot2.lock nice -n 15 ci/heavy.sh --wait 10800 -- <cmd>`.
- A Haiku `runner` waits on long scripts; `ci/blast_radius.yaml` and any `CMakeLists.txt` run all 92 steps.

## Noticed, not changed (flagged)
- GCS Activate says "the vehicle did not answer the config command" although the board applied it (it restarts first).
- The first 3531c3f image counted rate timing_faults=1 once after boot; 0 on every later boot. Receiver power-up leaves
  about 400 CRSF/UART errors that do not grow afterwards.
- Bench stats cannot combine with the flight logger (`fw/hal/target/CMakeLists.txt:184-189`): measured combined EMB-3 owed.
- No battery-voltage read (no low-battery failsafe); armed zero throttle idles at DShot 48 (the kill is ch5).
- The GCS library refuses images outside `~/marv-gcs` (`tools/gs/artifacts.py:146-147`); the dev venv lacks pyserial.

## Rules the next lead keeps
- Loosening tests, spec changes, tags and safety numbers are Luis's. Never rewrite pushed history.
- Never act on a pipe's exit status: `git switch m4 | tail` hid a failure on 2026-10-26 and fast-forwarded the wrong
  branch (restored, nothing lost). Push the exact checked sha as its own step.
- Never `pkill -f` broad patterns; check running workers before removing a worktree. Tuning is Luis's, in the UI.
