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
| `m4` | pushed at `52b4625` (dev check PASS at `0632319`, incl. GS units 4b, 7, 8a) |
| Board | flight image `gd46f7f6` (angle on ch8, airmode latch, Betaflight throttle; factory parameter table), est_valid=1 |
| In flight (night of 2026-10-18) | GS unit 6 LINK-SIL (branch `gs-link-sil`, engineer) and unit 8b GUI polish (`gs-gui-polish`, coder, npm only); next the ESKF in shadow (decision 0055), then units 9 and 11 |

## M4 since the last handoff (decisions 0048-0054)
- 0048 Luis's rulings (plant omega_min 0, angle ch8 at 100, HIL spec text, FMA tolerance 1 count, tags). 0049 plant drag
  (real card D = 0.356, 0.214, 0 kg/s, PLACEHOLDER). 0050 Betaflight airmode latch (1288 µs). 0052 loaded max rotor speed
  2297 rad/s and stick → rotor speed (hover 0.349). 0054 HIL S4 bridge (delay in the HIL source, receiver records).
- Bench: EMB-3 armed in angle mode U = 0.699. HIL S4a on the board: 60 s closed loop VERDICT OK, late-command control OK
  (`tests/regression/quad/L10/results/s4/`, with the S4b runbook). Gazebo evidence in `L10/results/s2e*`, `ihold*`.

## Ground segment (decisions 0051, 0053; Luis signed every item 2026-10-17)
- Done and merged into `m4`: unit 2 GEN (link ids, locks, .mcfg), 3 VALIDATE, 4a ConfigRefused/Maintenance, 5 CODEC
  (MARV dialect 15200-15204, GS-3 check), 7 HELPER (tools/gs), 8a GUI (gcs/: connection + parameters pages).
- Unit 4b STORE merged (store, boot policy, 84 tests; one parameter set per link + post-link gate).
- Next, in order: 6 LINK-SIL (the SIL speaks MAVLink over UDP; carry param_value as raw
  bits in the C++ codec); 8b GUI follow-ups; 9 REPORT; 11 PLOTS; 12 USB, 13 SD, 14 FLASH-SLOTS (bench); 15; 16.
- ESKF (decision 0055, Luis 2026-10-18): attitude + gyro bias from the IMU, in shadow beside Mahony; board use is Luis's call.

## Waiting on Luis
- S4b: fly the Gazebo sim through the board with the RadioMaster (runbook in `L10/results/s4/README.md`; host idle).
- Props-off radio check of angle mode (ESC power, Luis present).
- SD card: 32 GB FAT32 (Luis 2026-10-18). Bench items wait for Luis's morning (2026-10-18).

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
