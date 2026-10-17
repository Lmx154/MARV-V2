# MARV V2

C++20 GNC firmware for the MARV V2 board (RP2354B), its SIL simulation and its test tooling.

Current state and next step: `docs/handoff.md`. Build order: quad spec §4.0.

## Direction: MVP first

Luis, 2026-10-15: "we need a hard realignment because obviously this is not fast enough. i believe we should have just
started simpler instead of tryign to implement everything from the get go, by getting the MVP working then adding more
complex features gradually" (decision 0026).

- The goal is a quad flying on the MARV V2 board under onboard receiver control (quad §4.0, M0–M4), then a second
  airframe by configuration (M5). Build the least that meets the current milestone's "done when"; everything else
  goes on §4.0's deferred list.
- **A refinement never blocks hardware contact.** Simulation polish, extra fidelity, Monte Carlo campaigns and
  envelope tuning wait until after first flight unless a milestone cannot pass without them.
- A session reads `docs/handoff.md`, this file and the milestone it executes. Other spec sections and decision records
  are read only when the task touches them.

## ACTIVE-spec rule

- Work only from ACTIVE specs in `docs/spec/`: `00-core-contracts.md` and `10-quad-flight-software.md`.
  `20-ground-segment.md` and `30-rocket.md` are PARKED: read them for context, never implement from them. Only Luis
  changes a spec's status (core §0).
- The four specs are the only planning documents; quad §4.0 is the roadmap. Do not create other roadmap, plan,
  milestone or ledger files. A task names the milestone it executes and adds no scope.
- If a product spec contradicts the core, the core wins; report the contradiction.
- Draft 0 is retired and archived outside this repository. Do not read it, and do not read or reuse earlier projects
  (`~/Projects/Rust/MARV-FC`, `~/Projects/MARV-Gazebo-HIL`, anything in `~/Documents/Notes/marv-archive/`). Reading
  public reference projects (PX4, Betaflight, ArduPilot) for comparison is allowed; copy nothing GPL into MARV.
- The reference repository is the toolbox, `~/Projects/Web/avionics-toolbox` (core Appendix A says what to take,
  fix and leave).

## Number rule

Every number is exactly one of: a cited constant, a sourced parameter (with its uncertainty), a derived value (with
its rule), a design-budget entry (`design/budget.yaml`), a labelled scenario value, or a labelled `PLACEHOLDER` (core
§2).

- No numeric literals in `fw/` except 0, 1, 2 and ½. Cited constants go in the constants header with their citation
  (G1).
- Tunables are parameters with provenance, read through `param_get`; none are compiled in except the declared
  bootstrap exception (core §4, G2).
- A `PLACEHOLDER` carries its reason and what will replace it. It may be committed; it blocks the flight that depends
  on it, not the commit.
- Conflicting sources are both kept and marked `UNVERIFIED`. Claims about third-party behaviour are `INFERRED` until a
  committed test shows them. A measurement counts only if its script, inputs and raw output are committed.
- Troubleshooting never edits numbers in code: inject values through the parameter interface or the test harness.

## UNKNOWN rule

An agent that needs a number it cannot source tags it `UNKNOWN` and never guesses. For a safety number (arming,
failsafe, pyro, motor and current limits, safety thresholds) it stops and asks Luis. For any other number it may carry
a labelled `PLACEHOLDER` parameter so the work continues, and lists it in its report.

## CI gates

CI enforces these whether or not anyone reads this file (core §7.4):

| Gate | Rule |
| --- | --- |
| G1 | No unexplained numeric literals under `fw/` (clang-tidy magic-number checks). |
| G2 | Every tunable is a parameter with provenance. |
| G3 | No flight composition contains a `marv::truth` / `marv_truth_` symbol. |
| G4 | Nothing is tuned to one simulated vehicle (active with the vehicle zoo). |
| G5 | "It flies" is not a result: every derived requirement is checked with its margin. |
| G6 | The simulator is replayed against published flight data (active with branch B1). |
| G7 | Only validated (image, config) pairs fly (active with the ground segment). |
| G8 | This file keeps these four sections; CI fails if one is missing. |

## Tests and decisions

- Frozen suites live in `tests/regression/quad/Lnn/`. They run in full only on the merge into `master` (core §7.6).
  Development is checked by `ci/dev_check.sh` (smoke plus what the change can reach). A path no rule in
  `ci/blast_radius.yaml` maps selects the full suite; add the rule in the same change.
- Freeze requirements, not outputs: margins, predicates and contracts (core §7.2). Gazebo (T4) tests are behaviour
  predicates with one control (the loop disabled), never design-model envelopes.
- Negative controls only where a threshold decides: T3 thresholds, statistical checks and CI gates.
- Before adding a test, apply core §7.7: it traces to a pass-bar line, a contract, a guardrail or a found defect; name
  the existing test that would already catch the break (if one does, add nothing); extend before adding a file.
- A decision record (`docs/decisions/NNNN-<slug>.md`, one page at most) is needed only to loosen a threshold or
  predicate, delete a frozen test, or change a spec. Regenerating a golden with unchanged thresholds needs only the
  commit line `golden regenerated: <path or glob>: <cause>` (core §7.3, decision 0028). Never loosen a test to
  make it pass.
- The bench (TB) and flight logs are evidence: commit the script, the raw log and the card update.

## Commands

- `uv sync --frozen` first (the parameter generator runs from `.venv`).
- Build and test: `cmake --preset host-debug && cmake --build --preset host-debug && ctest --preset host-debug`
  (also `host-release`); target build: `cmake --preset m33 && cmake --build --preset m33`.
- Development check, the definition of verified for a commit: `ci/dev_check.sh` (`ci/smoke.sh`, then what
  `uv run python ci/affected.py` selects from the diff against `master`, in your own incremental build trees).
- Merge gate, the definition of verified for a merge into `master`: `ci/local_ci.sh [--commit <rev>] [core|gz-l2|gz-l4|gz-l5|gz-l7|gz-l11 ...]`
  (fresh clone, the Actions containers, the runner's limits; at most `--parallel 2`). Goldens are regenerated only
  inside these images until M0 replaces bit-exact goldens with tolerance compares.
- Gazebo hover demonstration (gz-sim 8 on PATH; quad §4.1 L2 and L5; run reports land in `<dir>`):

  ```sh
  cmake --preset host-gz && cmake --build --preset host-gz
  uv run python tools/sim/run_scenario.py --card vehicles/uzh_neurobem_5in.yaml \
      --scenario scenarios/quad/L02/hover.yaml --sequence --mode test --hover lo --out-dir <dir>
  cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5
  uv run python tools/refdata/refdata.py ensure quad/L05/t3 --procs 4
  uv run python tools/sim/run_l5.py --card vehicles/uzh_neurobem_5in.yaml \
      --scenario scenarios/quad/L05/step_roll.yaml --out-dir <dir> --m 1
  ```

## Code conventions (core §3)

- Frames NED / FRD. Quaternion Hamilton, body → NED, stored `[w, x, y, z]`, canonical sign w ≥ 0.
- SI units at every interface. Time is `uint64_t` µs through `hal_time_us()`; a tick is one primary-IMU sample.
- Motors use logical quad-X numbers 1–4; array index = logical number − 1.
- Scalar-templated numerics; flight builds instantiate `float`. Never `-fsingle-precision-constant`.
- `fw/` builds with `-fno-exceptions -fno-rtti`, static allocation only, no Pico SDK headers above the HAL.
