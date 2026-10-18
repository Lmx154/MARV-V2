# Handoff: M4 on `m4` (real card, drag, airmode latch, Betaflight throttle, HIL S4a passed); ground segment params-first in progress

2026-10-18. A state file, not a history: at most 80 lines, rewritten at every milestone boundary and before a session ends.
History is in git and `docs/decisions/`; the previous handoff is `git show be6b91b:docs/handoff.md`.

## Read first
- `CLAUDE.md`, quad spec §4.0 (M4), ground spec §5 (now ACTIVE, params-first scope, decision 0051).
- Serena memories: `ground-segment/params-first-ruling-full` (the signed design), `ground-segment/progress`,
  `ground-segment/gui-followups`, `hardware/barometer-timing`.

## Where things are
| Item | State |
| --- | --- |
| M0-M3 | merged into `master`; tags `quad-M1-pass`..`quad-M3-pass` |
| `m4` | pushed at `4495b9c` (dev check PASS; GS units 6, 8b, 9, 12, 13, 14 and the ESKF; 0c307dd was pushed with a failing check and fixed forward at 4495b9c) |
| Board | flight image `gd46f7f6` (angle on ch8, airmode latch, Betaflight throttle; factory parameter table), est_valid=1 |
| In flight (2026-10-18) | GS unit 15 FLASH-PANEL (branch `gs-flash`, coder, bench on the board); then 11 PLOTS (telemetry + seqlock), 10 DERIVE, 16 |

## M4 since the last handoff (decisions 0048-0054)
- 0048 Luis's rulings (plant omega_min 0, angle ch8 at 100, HIL spec text, FMA tolerance 1 count, tags). 0049 plant drag
  (real card D = 0.356, 0.214, 0 kg/s, PLACEHOLDER). 0050 Betaflight airmode latch (1288 µs). 0052 loaded max rotor speed
  2297 rad/s and stick → rotor speed (hover 0.349). 0054 HIL S4 bridge (delay in the HIL source, receiver records).
- Bench: EMB-3 armed in angle mode U = 0.699. HIL S4a on the board: 60 s closed loop VERDICT OK, late-command control OK
  (`tests/regression/quad/L10/results/s4/`, with the S4b runbook). Gazebo evidence in `L10/results/s2e*`, `ihold*`.

## Ground segment (decisions 0051, 0053; Luis signed every item 2026-10-17)
- Done and merged into `m4`: unit 2 GEN (link ids, locks, .mcfg), 3 VALIDATE, 4a ConfigRefused/Maintenance, 5 CODEC
  (MARV dialect 15200-15204, GS-3 check), 7 HELPER (tools/gs), 8a GUI (gcs/: connection + parameters pages).
- Unit 4b STORE, 6 LINK-SIL, 8b GUI polish, 9 REPORT merged. 13 SD (FatFs in third_party/fatfs, `L10/results/sd/`), 12a USB
  (two CDC, resets gated on the maintenance grant, `L10/results/usb/`), 12b link on the board (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
  1/3 for the grant; armed step with Luis 9/9, `L10/results/link_board/`) merged. 14 FLASH-SLOTS (slots at 0x1d0000/0x1e8000, POWMAN boot request, image hash at boot, `L10/results/slots/`)
  merged. Next: 15, 11, 10, 16.
- ESKF merged (decision 0055): shadow beside Mahony, T2 passes with controls failing (`L07/results/eskf_shadow/`), host-only
  SIL library `marv_sil_quad_fc_eskf`. Not done: the Gazebo comparison, a board cost measurement; board use is Luis's call.

## Waiting on Luis
- S4b: fly the Gazebo sim through the board with the RadioMaster (runbook in `L10/results/s4/README.md`; host idle).
- Props-off radio check of angle mode (ESC power, Luis present).
- Decide: check-4 form, ESKF board use (exFAT dropped, Luis 2026-10-18). Board: SD 64 GB FAT32; BAR_INT jumper (GPIO 23) soldered
  (mem hardware/barometer-timing); udev rule /etc/udev/rules.d/60-marv-picotool.rules (picotool without sudo).

## Noticed, not changed
- CI image has no Node (the TS conformance test registers only when gcs/node_modules exists): a CI-image unit before
  the merge gate. Boot timing fault with the receiver streaming at boot (INFERRED UART); rate timing_faults=1 after ~60% of probe reboots (L10/results/usb); G1 clang-tidy does not cover target-only files. ESC "Motor KV" 820 vs 1700.

## Rules the next lead keeps
- Standing permission (Luis, 2026-10-15): commit and push each unit that passed review and `ci/dev_check.sh`; merge
  into `master` after `ci/local_ci.sh` passes. Loosening a test, spec changes and tags stay Luis's. Fix forward.
- Never run a dev check while workers build. Read the check's result before pushing. Flashing during M4: permitted.
  No ESC power or motor spin without Luis present. Real-time HIL runs only on an idle host.
- **Memory (2026-10-17 oomd kill):** at most ONE build-heavy worker at a time (C++ builds, ctest, gz); check `free -g`
  before launching one; rewrite this handoff before starting long parallel work, not only at milestone boundaries.
