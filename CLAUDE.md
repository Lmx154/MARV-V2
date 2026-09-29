# MARV V2

C++20 GNC firmware for the MARV V2 board (RP2354B), its SIL simulation and its test tooling.

Current state and next step: `docs/handoff.md`.

## ACTIVE-spec rule

- Work only from ACTIVE specs in `docs/spec/`: `00-core-contracts.md` and `10-quad-flight-software.md`.
  `20-ground-segment.md` and `30-rocket.md` are PARKED: read them for context, never implement from them. Only Luis
  changes a spec's status (core §0).
- The four specs are the only planning documents. Do not create roadmaps, plans, milestone or ledger files. A task
  names the spine layer it executes (quad §4) and adds no scope.
- If a product spec contradicts the core, the core wins; report the contradiction.
- Draft 0 is retired and archived outside this repository. Do not read it, and do not read or reuse earlier projects
  (`~/Projects/Rust/MARV-FC`, `~/Projects/MARV-Gazebo-HIL`, anything in `~/Documents/Notes/marv-archive/`).
- The reference repository is the toolbox, `~/Projects/Web/avionics-toolbox` (core Appendix A says what to take,
  fix and leave).

## Number rule

Every number is exactly one of: a cited constant, a sourced parameter (with its uncertainty), a derived value (with
its rule), a design-budget entry (`design/budget.yaml`) or a labelled scenario value (core §2).

- No numeric literals in `fw/` except 0, 1, 2 and ½. Cited constants go in the constants header with their citation
  (G1).
- Tunables are parameters with provenance, read through `param_get`; none are compiled in except the declared
  bootstrap exception (core §4, G2).
- Conflicting sources are both kept and marked `UNVERIFIED`. Claims about third-party behaviour are `INFERRED` until a
  committed test shows them. A measurement counts only if its script, inputs and raw output are committed.
- Troubleshooting never edits numbers in code: inject values through the parameter interface or the test harness.

## UNKNOWN rule

An agent that needs a number it cannot source tags it `UNKNOWN` and stops to ask. It never guesses.

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

## Frozen tests and decisions

- Frozen suites live in `tests/regression/quad/Lnn/`. All of them run on every change to `master` (a pull request
  or a push; CI checks the whole pushed range).
- Modifying or deleting anything under `tests/regression/` needs a decision record, `docs/decisions/NNNN-<slug>.md`,
  in the same change (core §7.3). Never loosen a test to make it pass.
- Every metric test has a negative control that must break it (core §7.2).

## Commands

- `uv sync --frozen` first (the parameter generator runs from `.venv`).
- Build and test: `cmake --preset host-debug && cmake --build --preset host-debug && ctest --preset host-debug`
  (also `host-release`); target compile check: `cmake --preset m33 && cmake --build --preset m33`.
- Full CI, the definition of verified: `docker build -t marv-ci -f ci/Dockerfile .` then
  `docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci ci/run_ci.sh` (run on a clean
  copy if a host `build/` exists). Goldens are regenerated only inside this image.

## Code conventions (core §3)

- Frames NED / FRD. Quaternion Hamilton, body → NED, stored `[w, x, y, z]`, canonical sign w ≥ 0.
- SI units at every interface. Time is `uint64_t` µs through `hal_time_us()`; a tick is one primary-IMU sample.
- Motors use logical quad-X numbers 1–4; array index = logical number − 1.
- Scalar-templated numerics; flight builds instantiate `float`. Never `-fsingle-precision-constant`.
- `fw/` builds with `-fno-exceptions -fno-rtti`, static allocation only, no Pico SDK headers above the HAL.
