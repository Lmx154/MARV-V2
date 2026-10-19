# Handoff: M4 on `m4` (7-inch card, GS units 2-15 done, GCS-first scope signed); logger, feed-forward, telemetry, GCS shell in flight

2026-10-19. A state file, not a history: at most 80 lines, rewritten at every milestone boundary and before a session ends.
History is in git and `docs/decisions/`. The full session handoff is Serena memory **`session-handoff/2026-10-19`** (read it
first: worktree paths, every branch's state and next step, Luis's day plan, rules).

## Read first
- `CLAUDE.md`, quad spec §4.0 (M4), ground spec (ACTIVE, GCS-first scope, decisions 0051 and 0056).
- Serena memories: `session-handoff/2026-10-19`; designs with Luis's rulings at their top:
  `ground-segment/architect-gcs-first-class-2026-10-18`, `quad-m4/architect-onboard-logger-2026-10-18`,
  `quad-m4/architect-rate-response-2026-10-18`; facts: `hardware/vehicle-build`, `hardware/barometer-timing`;
  ground segment: `ground-segment/params-first-ruling-full`, `ground-segment/progress`, `ground-segment/gui-followups`.

## Where things are
| Item | State |
| --- | --- |
| M0-M3 | merged into `master`; tags `quad-M1-pass`..`quad-M3-pass` |
| `origin/m4` | `e5003d4`, dev check PASS (GS 12-14, card `vehicles/marv_7in_fc3v2.yaml` k_T 3.274e-6, sim `<ground/>`) |
| local `m4` | worktree `<session scratchpad>/wt-m4` (path in the memory), ahead of origin: decisions 0056/0057/0059, GX-0, logger U1, GX-1. NOT dev-checked as a whole: run `ci/dev_check.sh`, read PASS, then push |
| Board | m4-era flight image, factory config; logger U2 worker was using it (check the console banner before use) |
| Branches in flight | gs-telemetry (11a), log-target (U2), log-bench-profile (U3), rate-ff (done, in review, record 0058), gx2-shell (GX-2) — each one's state and next step in the memory |

## Done this session (2026-10-17..19)
- GS units 2-15: params over MAVLink, store + SD (FatFs, exFAT off), flash A/B slots, USB two-CDC with gated resets, link
  on the board (armed step with Luis 9/9), validation reports, flash panel with the G7 gate. ESKF in shadow (0055).
- 7-inch card rename + k_T (Luis: "yes switch and rename"; 0052 item 5). Sim ground, HIL `--gui` (paused start).
- S4b flown by Luis (ACRO: his angle switch drives ch14). Handling analysis → feed-forward (approved, branch rate-ff).
- Decisions: 0056 GCS-first scope, 0057 eRPM pulled forward, 0058 (draft, rate-ff) model following, 0059 onboard logger.

## Luis's bench plan (2026-10-19)
- Bifilar pendulum (inertia). Props-off bench-profile TB (U3 runbook; first check ch6 up/mid/down on the console).
- ESC: KV 1700, poles 14. Radio: CH8 mix (or rc_channel_angle 14; it is not locked); then S4b in ANGLE with `--gui`.
- CG check of the flight-ready quad. Props-on motor test only after the TB passes on the exact image (tau needs eRPM).
- Pending his yes: the QF-2 sentence in record 0058; check-4 stricter form; ESKF board use.

## Noticed, not changed
- Boot timing fault with the receiver streaming at boot (INFERRED UART); rate timing_faults=1 after ~60 % of probe reboots
  (L10/results/usb); G1 clang-tidy does not cover target-only files; R-1 (stick/arm/angle channels distinct) not enforced
  by validate yet (GX-6); ESKF config set ~330 B under half a slot (telemetry branch recomputes the slot).

## Rules the next lead keeps
- Standing permission (Luis, 2026-10-15): commit and push each unit that passed review and `ci/dev_check.sh`; merge
  into `master` after `ci/local_ci.sh` passes. Loosening a test, spec changes and tags stay Luis's. Fix forward.
- Read the check's result before pushing, as a separate step (0c307dd was pushed after a failing check once).
- No ESC power or motor spin without Luis present. Flashing during M4: permitted. Real-time HIL only on an idle host.
- **Memory:** one build-heavy job at a time: wait until `pgrep -x ninja` is empty, `free -g`, -j4; `ci/heavy.sh` (local m4)
  is the host-wide lock. Workers hit turn limits: have them commit in small steps; resume them with SendMessage.
