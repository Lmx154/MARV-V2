# Handoff: quad L0 passed, next is quad L1

2026-09-29. Written for the next agent working in this repository. This file records state; it adds no scope. The
specs remain the only planning documents (core §0 rule 5). Replace this file at the next handoff.

## Read first, in this order

1. `CLAUDE.md`: the rules CI enforces (ACTIVE-spec rule, number rule, UNKNOWN rule, gates G1–G8), commands,
   conventions.
2. `docs/spec/00-core-contracts.md` (ACTIVE): shared contracts. §2 numbers and provenance, §3 conventions, §4 firmware
   boundary and parameters, §7 testing and freezing, §10 decisions.
3. `docs/spec/10-quad-flight-software.md` (ACTIVE): §4 is the build order (the spine, L0–L10). **§4 "L1 — Vehicle card
   and `marv_plant` v0" is your next step.**
4. `docs/spec/20-ground-segment.md` and `30-rocket.md` are PARKED: read for context, never implement from them.

Luis (the owner) makes the final calls. Numbers you cannot source are tagged `UNKNOWN` and you stop to ask.

## Current state

- `master` at `6cca7ec`, tagged `quad-L0-pass`, pushed to `github.com:Lmx154/MARV-V2`. GitHub Actions CI green on it.
- Branch `quad-l0` equals `master`; it can be deleted. Local `.claude/worktrees/` and `worktree-agent-*` branches are
  leftovers from L0 workers (git-ignored, local only).
- Owner decisions after the specs were written (already recorded in the specs): keep branch `master`; no branch
  protection; a change is a pull request into `master` or a push to `master`, and CI checks the whole pushed range.
- Draft 0 is archived at `~/Documents/Notes/marv-archive/` (not a source; don't read it). Earlier projects
  (`~/Projects/Rust/MARV-FC`, `~/Projects/MARV-Gazebo-HIL`) are off-limits (core §1).

## What L0 built (quad spec §4 L0, all pass-bar items met)

| Area | Where | Notes |
| --- | --- | --- |
| Primitives | `fw/prim/` | `Vec`, `Mat`, Hamilton `Quat` stored `[w,x,y,z]`, templated on scalar; float instantiated for the M33. `constants.hpp` is the ONLY place for cited constants (only file exempt from G1). |
| Types | `fw/types/` | `TimeUs`/`Tick` (uint64), `ImuSample` (FRD rates rad/s, specific force m/s², temp K, flag bits), `Motor` (logical 1–4, index = n−1), `DshotValue` (type can only hold 0 or 48–2047), `ActuatorOutput<M,S>`. |
| HAL | `fw/hal/` (+ `fw/hal/sim/`) | `hal_time_us`, `hal_actuators_write`, `hal_panic`; `composition::init/tick` contract. `hal_sim`: exact virtual clock (period num/den µs, stamp = ⌊n·num/den⌋, time frozen inside a tick), actuator latch. |
| Scheduler | `fw/sched/` | `RateGroups<N>`: stateless `due(n)` bitmask, divisors from parameters, lowest index first. |
| Parameters | `fw/params/`, `tools/gen/params_gen.py` | YAML sources → generated ids/defaults/manifest; generator refuses entries without method, source or σ (G2). `param_get`/`param_value<Id>`/`params_init`. `marv_add_param_set(<name> CARD … REGISTER …)` builds independent sets; product set is `marv_params` (cache vars `MARV_PARAM_CARD_FILES`/`MARV_PARAM_REGISTER_FILES`, currently pointing at the L0 fixtures). |
| SIL entry | `fw/sil/` | `marv_sil.h`, pure C ABI (info_get/init/tick/shutdown). One init per process; validate-before-act; k ticks per host step; exports only `marv_sil_*` (hidden visibility + version script). `marv_add_sil_library(<name> COMPOSITION <lib> [PARAMS <set>])`. Harness-only parameter override. |
| L0 composition, null plant | `fw/compositions/l0/`, `sim/null_plant/` | Fixed test composition (host + m33); runner with SplitMix64 IMU stream and FNV-1a trace hash. |
| Frozen suite | `tests/regression/quad/L00/` | 157 ctest tests labelled `frozen`, golden `replay/golden_l0_trace.txt` (hash `96ab3c6663da1c63`), CI-gate controls, tools tests, L0 fixtures (own param set `marv_params_l0`, independent of the product card). |
| CI | `ci/Dockerfile`, `ci/run_ci.sh`, `.github/workflows/ci.yml` | Pinned image (the only place goldens are regenerated). Gates: G1 `tools/ci/lint_g1.py` + `.clang-tidy`; G3 `tools/ci/check_g3.py` + `cmake/flight_targets.cmake` (flight targets found automatically under `fw/`); G8 `tools/ci/check_claude_md.py`; frozen-test rule `tools/ci/check_regression_changes.py`; PB2 double-promotion control. Every gate has a negative control that must fail. |

The L0 interface decisions are frozen: the declarations in the headers above are the contract. Changing one needs a
decision record in `docs/decisions/` (template `0000-template.md`), and so does modifying or deleting anything under
`tests/regression/`.

## How to verify

```
uv sync --frozen
cmake --preset host-debug && cmake --build --preset host-debug && ctest --preset host-debug
cmake --preset m33 && cmake --build --preset m33
docker build -t marv-ci -f ci/Dockerfile .
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci ci/run_ci.sh   # the definition of verified
```

Run the Docker command on a clean copy if a host `build/` exists, and always with `-u` (otherwise it leaves root-owned
files). clang-tidy exists only in the image. After CMake changes to parameter sets, use a fresh `build/`.

## Next step: quad L1

Execute quad spec §4 **L1 — Vehicle card and `marv_plant` v0**: its Builds, Opening, Pass bar and Freezes are the
scope. Open items the spec and L0 leave for L1 (check them before building; ask Luis where the spec leaves a choice):

- **Card and register.** The spec's example card fails its own rules (core §2.1 note, §2.3: σ missing on most entries;
  fill or tag before L1 closes). The NeuroBEM mass conflict (752 vs 772 g) stays `UNVERIFIED`. Spin directions per
  logical motor have no default and must be in the card (core §3). `design/budget.yaml` does not exist yet; its values
  are filled when a layer that tests against them is specified (core §2.2, C-6).
- **Generator gaps found at L0.** `params_gen.py` handles scalar f32/i32 only (card entries like `inertia_diag` are
  vectors); the `locked` flag has no source field; σ = 0 is accepted for measured/datasheet values (decide the policy
  in the card linter); CMake checks that `MARV_PYTHON` exists but not that it can import PyYAML.
- **Switching the product parameter set** to the real card and register is done through the cache variables; the L0
  frozen suite uses its own set and must stay green.
- **SDF and plugin-config generators** are in the L1 pass bar although the Gazebo plugin is L2. Gazebo gz-sim 8
  (Harmonic, the spec's host) and gz-sim 10 are both installed; `gz sim` may pick 10, so pin the version when L2 starts.
- **Linters and generators run in CI**, with negative controls, like every L0 gate.

## Carried forward (not L1 unless the spec says so)

- L9: on the M33, the scheduler's 64-bit `%` calls `__aeabi_uldivmod` and `init` calls `memcpy` (libgcc/newlib,
  flash-resident by default); core 1 must run from SRAM (EMB-1), as must `hal_panic`. Target `hal_panic` behaviour
  (what the motors do) is an open flight-safety decision. One SIL init per process means a Gazebo GUI reset needs a
  gz restart.
- Spec gaps outside L0, not yet fixed: L7's attitude-error budget "derived from QF-3" has no rule; Q-D6's criterion
  does not match the L6 Allan check; L10's "within float tolerance" needs a horizon and metric.
- Known, accepted gate limits: G1 does not flag character literals used as numbers or files with unusual suffixes; G3
  does not parse `-Wp,-include` or `@file` response files; G8 checks section presence, not substance.

## Working notes

- Every numeric literal under `fw/` other than 0, 1, 2 and ½ fails CI; don't dodge the lint with arithmetic
  (`2 + 1`); put cited constants in `constants.hpp` with their citation.
- Goldens are regenerated only inside the `marv-ci` image, with the explicit command in
  `tests/regression/quad/L00/replay/CMakeLists.txt`; CI never regenerates.
- Keep changes scoped to the layer being executed; report anything noticed but not changed.
