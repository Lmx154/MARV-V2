# 0026: MVP-first realignment

## What changed

No file under `tests/regression/` changes in this record's commit. It changes the specs and the rules:

- `docs/spec/00-core-contracts.md`: §2 adds the `PLACEHOLDER` provenance and splits rule 1 (safety numbers stop to
  ask, others may be carried as placeholders); §7.1 adds the TB bench level, moves T4 to the merge and defers T6;
  §7.2 item 3 (negative controls only for T3 thresholds, statistical checks and CI gates; T4 tests are behaviour
  predicates with the loop-disabled control) and a new item 6 (freeze requirements, not outputs); §7.3 (a record only
  to loosen, delete or change a spec; a regeneration needs the `golden regenerated:` commit line); §7.4.
- `docs/spec/10-quad-flight-software.md`: §0 (the MVP path is what is active), §1 (bench and flight test card gate
  the first flights; HIL deferred), new §4.0 (build order M0–M6 and the deferred list), §4.1 (the existing layers,
  with MVP-subset notes on L6–L10), §6.1 (minimal arming), §6.3 (B1 starts after M4).
- `CLAUDE.md`: a "Direction: MVP first" section; the UNKNOWN rule, the number rule and the tests section follow the
  spec changes.
- `docs/handoff.md`: rewritten as a state file of at most 80 lines.
- `ci/blast_radius.yaml`: `.claude/**` is ignored.

## Why

Luis, 2026-10-15, verbatim: "last time i got a fully working firmware in a few hours, this time it's taking 1 month.
it was working well, so there is something happening here that is stalling us there is something in the roadmap that
is pinning us down into testing and development hell".

Four read-only advisor audits (prior art, velocity, adaptability, physics; 2026-10-15) agreed on the cause:

1. The T4 pass bar required nonlinear Gazebo runs to stay inside linear design-model envelopes tight enough that
   gains × 1.1 break them (decision 0006). Every unmodelled effect became a red test needing new maths and rulings:
   DShot quantisation (stage (d), 0017), gyroscopic coupling (R2, 0025), noise (a Monte Carlo estimated at 3.1 h;
   its reference run was killed after 47 min, then no commit from 2026-10-05 to 2026-10-13).
2. Those runs are perfect-model (`tools/card/report.py:33`): truth and firmware share one card with no dispersion,
   and the plant leaves out rotor inertia, battery and ground contact. They re-confirm the design, not the hardware.
3. The spine put the board (L9) after L6 (e), L7 and L8, and froze bit-exact controller outputs while the controller
   still changed: 51 of 172 commits modified or deleted frozen files, each needing a record (556 KB of records).
4. Decisions 0021–0024 cut run time, not the cause.

For comparison, PX4 and Betaflight carry about 0.25–0.4 test lines per source line and gate per commit on unit tests
and simple simulation; MARV carries 10.6 (66.6k test lines against 6.3k in `fw/`).

## Evidence

- The advisor reports (in the 2026-10-15 session; summarised above). Spot-checked by the lead: the 1 µs halting
  tolerance (`fw/rate/include/marv/rate/rate_loop.hpp:52,230-241`), the stamp-equality halt
  (`fw/compositions/l5_attitude_scripted/src/l5_attitude_scripted.cpp:194`), the square-mixer assert
  (`fw/mixer/include/marv/mixer/mixer.hpp:28`), zero specific force at rest (`sim/plant/src/marv_plant.cpp:255-264`),
  the perfect-model line (`tools/card/report.py:33`), the 8.3 rad/s PI crossover
  (`marv_params_rate_report.txt`), the `m33` preset labelled compile-only (`CMakePresets.json:80`), 51 commits
  modifying or deleting files under `tests/regression/`, and the gap between `b48e202` and `afab9ae`.
- No threshold, tolerance or predicate is loosened by this record. The T4 envelope suites stay as they are until M0
  replaces each with predicates under its own record, listing what still covers each removed check.

## Approval

Luis, 2026-10-15, verbatim, on the advisors' recommendations: "very well, let's accept the recommended changes".
And: "i need you to overwrite the handoff and the roadmap aswell as everything needed to redirect our efforts. if you
have to remove old stuff then do so. we need a hard realignment because obviously this is not fast enough. i believe
we should have just started simpler instead of tryign to implement everything from the get go, by getting the MVP
working then adding more complex features gradually".
