# Handoff: M3 merged and tagged; M4 on `m4` (real card, angle mode, HIL S1-S3, drag, real-card Gazebo flight); airmode latch in progress

2026-10-17. A state file, not a history: at most 80 lines, rewritten (not appended) at every milestone boundary.
History is in git and `docs/decisions/`; the previous handoff is `git show 68d2214:docs/handoff.md`.

## Read first
- `CLAUDE.md`, then quad spec §4.0 (M4 as amended: Luis's order before the stand; L10 HIL and plant drag pulled forward).

## Where things are
| Item | State |
| --- | --- |
| M0-M3 | merged into `master`; tags `quad-M1-pass` 5365665, `quad-M2-pass` 4d3db06, `quad-M3-pass` 55120ad (M0 untagged) |
| M4 | branch `m4`, pushed at `c8418d0` (dev check PASS on it); board holds the bench-stats image `g527a8ca` (angle on ch8) |
| In progress | branch `m4-airmode` (worktree `wt-airmode`): Betaflight's default airmode latch replacing 0045, decision 0050 |

## M4 so far (decisions 0043-0050)
- Real airframe card `vehicles/marv_5in_fc3v2.yaml` (7-inch, 732 g with pack; thrust, torque, inertia INFERRED until stand and
  pendulum); gains from it (QF-3 rate worst PM 45.0°, Ms 2.0). Angle mode in quad_fc on **ch8 at 100** (0048).
- Luis's rulings 2026-10-17 (0048): plant accepts omega_min = 0; HIL spec text; FMA: flight build keeps contraction,
  L10 comparison within 1 DShot count; tags; airmode first kept (0045), then **B: Betaflight latch** after the sim demo.
- HIL (0046/0047): S1 link, S2 SIL + delay line + launch pad, S3 board DShot = SIL within 1 count. S2e: the real card flies
  quad_fc in Gazebo in angle mode at N = 10 (`L10/results/s2e`, `s2e_drag`). Tool: `tools/sim/run_quad_fc.py`.
- Plant drag (0049): opt-in −R D Rᵀ v in dynamics and specific force; real card D = (0.356, 0.214, 0) kg/s from Faessler et
  al. 2018 scaled by mass (PLACEHOLDER until a flight fit). Without drag the estimator drifted to level (truth 13-19° tilted
  after a step); with drag 3-6° and decaying. UZH-card logs byte-identical.
- I-hold demo (`L10/results/ihold`, 5 mm CG offset via shifted rotor positions in the world): a dip to zero throttle gives a
  26.8° lurch after the throttle returns vs 2.3° when the integrator is kept: why Luis chose the latch.
- Bench 2026-10-17 (`L09/results/m4-bench-card.md`): EMB-3 armed in angle mode U = 0.699 (C 93.2 µs, T 133.4 µs), 0 faults.

## Next
1. Airmode latch: review, Gazebo comparison (`L10/results/ihold_latch`), dev check, push; rebuild + flash the flight image.
2. Boot timing fault with the receiver streaming at boot (INFERRED UART traffic): explain before props-on. Flight test
   card draft (pre-flight, tethered hover, free hover, angle, acro, abort criteria) for Luis's approval.
3. HIL S4 (Luis near the board: the radio must reach its receiver), then S5 (S4's sticks replayed in SIL).
4. Luis present: props-off radio check of angle mode (ESC power). Then stand/pendulum, then tethered hover.

## Noticed, not changed
- `t3_test.cpp` builds the rate loop's config by hand without its timing fields. sensor_replay refuses logs with records 9-13
  (so a drag world's HIL-style log cannot be replayed). ESC "Motor KV" 820 vs 1700. Speed range max = no-load speed.
  Card filename says 5in for a 7-inch airframe. Drag-world replay through sensor_replay: UNKNOWN (code-read only).

## Rules the next lead keeps
- Standing permission (Luis, 2026-10-15): commit and push each unit that passed review and `ci/dev_check.sh`; merge
  into `master` after `ci/local_ci.sh` passes. Loosening a test, spec changes and tags stay Luis's. Fix forward.
- Never run a dev check while coders build (a timing test times out under load). Read the check's result before
  pushing. Flashing during M4: permitted by Luis (2026-10-17). No ESC power or motor spin without Luis present.
