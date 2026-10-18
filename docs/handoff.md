# Handoff: M4 on `m4` (real card, drag, airmode latch, Betaflight throttle, HIL S4a passed); ground segment params-first in progress

2026-10-18. A state file, not a history: at most 80 lines, rewritten at every milestone boundary and before a session ends.
History is in git and `docs/decisions/`; the previous handoff is `git show be6b91b:docs/handoff.md`.

## Read first
- `CLAUDE.md`, quad spec §4.0 (M4), ground spec §5 (now ACTIVE, params-first scope, decision 0051).
- Serena memories: `ground-segment/params-first-ruling-full` (the signed design), `ground-segment/progress`,
  `ground-segment/gui-followups`.

## Where things are
| Item | State |
| --- | --- |
| M0-M3 | merged into `master`; tags `quad-M1-pass`..`quad-M3-pass` |
| `m4` | pushed at `7fbb8f6` (dev check PASS at `d46f7f6`); local `m4` is 8 commits ahead (GS units 7 and 8a, reviewed, not yet dev-checked) — run `ci/dev_check.sh` alone, then push |
| Board | flight image `gd46f7f6` (angle on ch8, airmode latch, Betaflight throttle; factory parameter table), est_valid=1 |
| In flight | GS unit 4b (parameter store): store code at `999c72f` on `gs-store`; two workers were killed by systemd-oomd (2026-10-17 18:17) with UNCOMMITTED work in `wt-gs-store-link` (link-hazard fix + wiring) and `wt-gs-store-tests` (store_test.cpp) under the session scratchpad |

## M4 since the last handoff (decisions 0048-0054)
- 0048 Luis's rulings (plant omega_min 0, angle ch8 at 100, HIL spec text, FMA tolerance 1 count, tags). 0049 plant drag
  (real card D = 0.356, 0.214, 0 kg/s, PLACEHOLDER). 0050 Betaflight airmode latch (1288 µs). 0052 loaded max rotor speed
  2297 rad/s and stick → rotor speed (hover 0.349). 0054 HIL S4 bridge (delay in the HIL source, receiver records).
- Bench: EMB-3 armed in angle mode U = 0.699. HIL S4a on the board: 60 s closed loop VERDICT OK, late-command control OK
  (`tests/regression/quad/L10/results/s4/`, with the S4b runbook). Gazebo evidence in `L10/results/s2e*`, `ihold*`.

## Ground segment (decisions 0051, 0053; Luis signed every item 2026-10-17)
- Done and merged into `m4`: unit 2 GEN (link ids, locks, .mcfg), 3 VALIDATE, 4a ConfigRefused/Maintenance, 5 CODEC
  (MARV dialect 15200-15204, GS-3 check), 7 HELPER (tools/gs), 8a GUI (gcs/: connection + parameters pages).
- Next, in order: finish 4b (see "In flight"); 6 LINK-SIL (the SIL speaks MAVLink over UDP; carry param_value as raw
  bits in the C++ codec); 8b GUI follow-ups; 9 REPORT; 11 PLOTS; 12 USB, 13 SD, 14 FLASH-SLOTS (bench); 15; 16.
- **Link hazard (found in 4b):** `marv::generated::kParamDefaults` is defined in several parameter-set archives; the
  board image gets FC3v2's 137-record table only by archive order. Today's images are correct (0xefc, checked). Fix
  before wiring anything new into the image (the killed engineer's packet: per-set symbols or one set per image, plus a
  post-link gate with a negative control).

## Waiting on Luis
- S4b: fly the Gazebo sim through the board with the RadioMaster (runbook in `L10/results/s4/README.md`; host idle).
- Props-off radio check of angle mode (ESC power, Luis present).
- SD card size and format (FAT32 / exFAT) for unit 13.

## Noticed, not changed
- CI image has no Node (the TS conformance test registers only when gcs/node_modules exists): a CI-image unit before
  the merge gate. Boot timing fault with the receiver streaming at boot (INFERRED UART). ESC "Motor KV" 820 vs 1700.

## Rules the next lead keeps
- Standing permission (Luis, 2026-10-15): commit and push each unit that passed review and `ci/dev_check.sh`; merge
  into `master` after `ci/local_ci.sh` passes. Loosening a test, spec changes and tags stay Luis's. Fix forward.
- Never run a dev check while workers build. Read the check's result before pushing. Flashing during M4: permitted.
  No ESC power or motor spin without Luis present. Real-time HIL runs only on an idle host.
- **Memory (2026-10-17 oomd kill):** at most ONE build-heavy worker at a time (C++ builds, ctest, gz); check `free -g`
  before launching one; rewrite this handoff before starting long parallel work, not only at milestone boundaries.
