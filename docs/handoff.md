# Handoff: quad L1 passed, next is quad L2

2026-09-29. Written for the next agent working in this repository. This file records state; it adds no scope. The
specs remain the only planning documents (core §0 rule 5). Replace this file at the next handoff.

## Read first, in this order

1. `CLAUDE.md`: the rules CI enforces (ACTIVE-spec rule, number rule, UNKNOWN rule, gates G1–G8), commands,
   conventions.
2. `docs/spec/00-core-contracts.md` (ACTIVE): shared contracts. The sections that matter most here:
   - §2 numbers and provenance;
   - §3 conventions (host frames convert once, in the adapter);
   - §4 firmware boundary;
   - §6 physics ownership and hosts;
   - §7 testing, freezing and the §7.5 convergence rule.
3. `docs/spec/10-quad-flight-software.md` (ACTIVE): §3 (Gazebo only: the lockstep plugin, the two modes, the SIM
   requirements) and §4, the build order (the spine, L0–L10). **§4 "L2 — Gazebo host" is your next step.** Q-D4
   (§7) is decided at L2.
4. `docs/spec/20-ground-segment.md` and `30-rocket.md` are PARKED: read for context, never implement from them.
5. `docs/decisions/0001-*.md` and `0002-*.md`: the two L1 decision records.

Luis (the owner) makes the final calls. Numbers you cannot source are tagged `UNKNOWN` and you stop to ask.

## Current state

- **Branch and tag.** `master` holds the L1 work plus this handoff; the L1 work is pushed to
  `github.com:Lmx154/MARV-V2`. The tag `quad-L1-pass` is on `b8ed690`, where GitHub Actions run 36609144674 is green
  and the full Docker CI passes 40/40. The tag `quad-L0-pass` is on `6cca7ec`.
- **Branches and worktrees.** Only `master` exists. There are no local worktrees or `worktree-agent-*` branches.
  `.serena/` is git-ignored.
- **Decision records** (both approved by Luis on 2026-09-29, with conditions that are met):
  - **0001:** `SigmaKind` on the parameter record. The kind/σ rule (I1) is enforced at build time (generator refusal,
    and a `static_assert` over the generated table) and at firmware startup.
  - **0002:** the frozen G1 tools test no longer pins the full literal list of `constants.hpp`. In the same change,
    `tools/ci/check_constants.py` now requires a citation and a math, physics or standard kind on every constant.
- **Owner decisions after the specs were written** (recorded in the card, budget and decision records):
  - **σ policy.** Published, measured, identified and datasheet entries need σ > 0 derived by a recorded rule, or
    σ `UNKNOWN`. Design-budget and scenario entries use `sigma: choice`. `sigma: exact` is allowed only for integer
    counts. Every run report lists the σ-`UNKNOWN` entries.
  - **Mass.** 0.772 kg, `UNVERIFIED`, with 0.752 kg recorded as the conflict (paper and agile_flight). B1's log
    replay settles it. σ is 0.010 kg, half the spread.
  - **Spin directions.** m1 and m2 ccw, m3 and m4 cw. Read from NeuroBEM Fig. 3, and they agree with the PX4/ArduPilot
    quad-X directions. Tagged `INFERRED`; B1 replay checks the yaw-torque sign.
  - **ESC map.** A labelled scenario value, linear in ω: DShot 48 → ω_min, 2047 → ω_max, 0 = stop. The pole count is
    14 (Hobbywing XRotor 2306 Race Pro, variant `INFERRED`).
  - **Plant precision.** `marv_plant` runs in double.
  - **RK4 and table interpolation** join the primitives with their first user, not at L1.
  - **Budget values.** Budget-register values are filled only when a layer that tests against them is specified.
- **Draft 0 and earlier projects.** Draft 0 is archived at `~/Documents/Notes/marv-archive/`; it is not a source, so
  don't read it. Earlier projects (`~/Projects/Rust/MARV-FC`, `~/Projects/MARV-Gazebo-HIL`) are off-limits (core §1).

## What L1 built (quad spec §4 L1, all pass-bar items met)

| Area | Where | Notes |
| --- | --- | --- |
| Card / profile / budget schema + linter | `tools/card/schema.py`, `tools/card/lint.py` | The one implementation of the entry schema and the σ policy. It rejects the core §2.1 example card, a card without spin directions, a single σ on a vector entry, and rotor positions outside the core §3 quadrants. |
| Data | `vehicles/uzh_neurobem_5in.yaml`, `sensors/profiles/marv_v2_board_default.yaml`, `design/budget.yaml` | Every number carries its source. The profile keeps datasheet units and is not flattened into firmware until L6. The budget restates the rationales for PM_min (45°, stored in rad) and the χ² quantile (0.999); every other budget entry is `value: UNKNOWN`. |
| Parameter record | `fw/params/include/marv/params/param_types.hpp`, `fw/params/src/param.cpp` | `SigmaKind {Known, Exact, Unknown, Choice}` sits after `locked`; sizeof is unchanged (40 host / 28 M33). One `constexpr sigma_consistent` is shared by `params_init` and the generated `static_assert`. |
| Generator | `tools/gen/params_gen.py` | σ tokens (`UNKNOWN`, `choice`, `exact`), `lock {by, on, via: manual}`, and vector `shape` (frd3, diag3, range, motors) expanded to scalar ids. It checks each record against the kind/σ rule before writing and emits `params_provenance.json`. The L0 `param_ids.hpp` and manifest are byte-identical. |
| Product parameter set | `tools/card/flatten.py`, `marv_add_card_param_set` in `fw/params/CMakeLists.txt` | card → lint → flatten → params_gen, 27 ids including `rotor_yaw_sign_m1..m4` = +1, +1, −1, −1. The cache vars are `MARV_VEHICLE_CARD` and `MARV_DESIGN_BUDGET`. `marv_params_l0` and `marv_params_l1_fixture` are independent test sets. |
| Run report | `tools/card/report.py`, target `marv_params_report` (defined outside `fw/`) | Written to `build/<preset>/generated/marv_params/l1_report.txt`. Contents: the card hash (card plus profile, `8d34b3e8…`), the budget hash, the commit, and the UNVERIFIED / INFERRED / σ UNKNOWN / value UNKNOWN / design-budget / locked lists. It is labelled perfect-model. |
| Gravity | `fw/prim/include/marv/prim/gravity.hpp` | WGS 84, NIMA TR8350.2 3rd ed. Amdt 1, eq. (4-1) and (4-3), with the constants in `constants.hpp`. Golden vectors come from the toolbox's `gravity.ts` (`tools/golden/gravity_vectors.mjs`). |
| `marv_plant` v0 | `sim/plant/` (`marv_plant.h`) | C ABI, opaque handle, validate-before-act. DShot → ω map; exact zero-order-hold first-order motor over fixed sub-steps plus one partial step. Returns the NED wrench about the CM with gravity included, plus rotor speeds and eRPM. The wrench is evaluated from the end-of-step motor state. Double precision, host only. |
| Generators | `tools/card/gen_plant_config.py`, `tools/card/gen_sdf.py`, `cmake/marv_card_gen.cmake` | Write the plant-config C header (hex floats), the plugin `<plugin>` XML and SDFormat 1.11 (sdformat14 = gz-sim 8), into `build/<preset>/generated/plant_card/`. Link origin at the CM. The consumer supplies the scenario fields (site latitude and height, sub-step h, seed). |
| Frozen suite | `tests/regression/quad/L01/` | ctest: `unit/prim` (gravity), `unit/params`, `unit/plant` (motor, wrench, ABI, KAT), `unit/plant_card` (hover against the card). pytest: `tools/`. Plus `fixtures/`, `controls/`, and the plant known-answer reference `unit/plant/reference/plant_ref.py` with its committed inputs and outputs. |
| CI | `ci/run_ci.sh`, `tools/ci/check_constants.py`, `tools/ci/check_g3.py` | New steps: L01 pytest, the card lint, the product build from the card, the constants citation check, G3 plant patterns (`marv_plant_`, `marv::plant::`), plant-reference reproduction, and printing the report. Each has a negative control that must fail: the §2.1 card, a σ = 0 card, a Python without PyYAML, a mismatched record table (`static_assert`), an uncited constant, a vehicle number in `constants.hpp`, a planted plant symbol, and perturbed reference inputs. |

The L0 and L1 interface decisions are frozen: the declarations in the headers above are the contract. Changing one
needs a decision record in `docs/decisions/` (template `0000-template.md`, next number 0003). So does modifying or
deleting anything under `tests/regression/`, including `L01/`.

## How to verify

```
uv sync --frozen
cmake --preset host-debug && cmake --build --preset host-debug && ctest --preset host-debug
cmake --preset m33 && cmake --build --preset m33
docker build -t marv-ci -f ci/Dockerfile .
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci ci/run_ci.sh   # the definition of verified
```

Run the Docker command on a clean copy if a host `build/` exists, and always with `-u` (otherwise it leaves root-owned
files). Set `-e MARV_CI_BASE_REF=origin/master` to check a branch's range. clang-tidy exists only in the image. After
CMake changes to parameter sets, use a fresh `build/`. Today the full run is 40 steps, about 10 minutes.

## Next step: quad L2

Execute quad spec §4 **L2 — Gazebo host**. Its Builds, Opening, Pass bar and Freezes are the scope. Check these open
items before building, and ask Luis where the spec leaves a choice:

- **Gazebo version and CI.**
  - Pin gz-sim 8 (Harmonic, the spec's host). gz-sim 8 and 10 are both installed, `gz sim` may pick 10, and `gz sdf`
    on this host is sdformat16.
  - The CI image has no Gazebo. L2's T4 tests need it headless in CI, which is a change to `ci/Dockerfile`. That is a
    toolchain decision: take it to Luis.
- **Plugin.**
  - The lockstep plugin reads the generated `<plugin>` element. Its `filename` / `name` (`marv_gz_lockstep` /
    `marv::gz::Lockstep`) are provisional, so rename them in `tools/card/gen_plant_config.py` if needed.
  - World gravity is zero (the plant applies it).
  - Wrench application: `AddWorldWrench` acts at the link origin, and the generated models put that origin at the CM
    (quad §3.1).
- **Frames.** `marv_plant` returns the wrench in NED. The adapter converts ENU/FLU ↔ NED/FRD once, and the adapter
  test checks it against a direct `marv_plant` call.
- **Sensor bytes.** The L2 adapter test mentions "every sensor byte", but `marv_plant` v0 has no sensor models (they
  arrive with L6). Settle what the test compares at L2 before building it.
- **Step sizes.** Gazebo's step, `marv_plant`'s motor sub-step and firmware ticks per host step are all chosen by the
  §7.5 convergence rule (SIM-7), with the halving sequence recorded. `marv_plant` holds each command over one step
  call, so one plant call per firmware tick keeps the zero-order hold exact. Nothing bounds dt/h yet.
- **SIM-3 throughput.** It needs the budget register's `batch_plan`, which is `value: UNKNOWN`. That is an UNKNOWN to
  raise with Luis, not to guess.
- **Q-D4.** How each mode sets its real-time factor (generated world variants, or `set_physics`) is decided at L2.
- **Firmware state.** One SIL init per process, so a Gazebo GUI reset needs a gz restart.
- **Scenario runner.** The runner (card, scenario, seed) → logged run should call `tools/card/report.py`'s
  `build_report`. Truth is not dispersed yet (core §6), so every closed-loop result is labelled perfect-model.

## Carried forward (not L2 unless the spec says so)

- **L9.**
  - On the M33, the scheduler's 64-bit `%` calls `__aeabi_uldivmod`, and `init` calls `memcpy`. Both come from
    libgcc/newlib and are flash-resident by default.
  - Core 1 must run from SRAM (EMB-1), and so must `hal_panic`.
  - What the motors do on a target `hal_panic` is an open flight-safety decision.
- **B1.**
  - Mass conflict (0.772 vs 0.752 kg).
  - Spin directions and yaw-torque sign.
  - The thrust_map reading (`INFERRED`).
  - blackbird.yaml's `thrust_max` of 8.5 N per motor vs k·2800² = 12.25 N. The card uses only the speed range.
  - σ for every card entry is `UNKNOWN` except mass. A layer whose pass bar consumes an `UNKNOWN` σ fails until the σ
    is sourced.
- **Spec gaps outside L2, not yet fixed** (for Luis):
  - L7's attitude-error budget "derived from QF-3" has no rule.
  - Q-D6's criterion does not match the L6 Allan check.
  - L10's "within float tolerance" needs a horizon and a metric.
  - Core §2.1's example card uses field shapes the schema rejects (`tau_s`, string `conflict` and `status`).
  - Core §9 lists RK4 and interpolation at L1, although they join with their first user.
  - Core §3's "double only for the shadow" rule does not mention simulator code (the plant runs double by owner
    decision).
- **Known, accepted gate limits.**
  - G1 covers `fw/` only; `sim/plant` has structural literals such as array sizes and the quaternion index.
  - G1 does not flag character literals or files with unusual suffixes.
  - G3 does not parse `-Wp,-include` or `@file` response files, and its plant control is host-only.
  - G8 checks section presence, not substance.
  - `check_constants.py`'s vehicle-number guard is a keyword rule (card, profile, budget, datasheet), so review of
    `constants.hpp` changes stays the last line of defence.
- **Goldens.**
  - `plant_ref_expected.txt` reproduces inside the CI image.
  - `gravity_golden.csv` is reference data from the external toolbox, compared within a tolerance. The image has no
    node, so it cannot regenerate it.

## Working notes

- **Numeric literals.** Every numeric literal under `fw/` other than 0, 1, 2 and ½ fails CI; don't dodge the lint with
  arithmetic (`2 + 1`). Cited constants go in `constants.hpp`. Each paragraph there needs a `Citation:` and one
  `Kind: math|physics|standard`. Vehicle and part numbers go in the card or a sensor profile, never there.
- **Card YAML.** Consumers must load card YAML through `tools/card/schema.py`: plain `yaml.safe_load` reads
  `1.562522e-6` as a string. The generators require the card file to be named `<vehicle>.yaml`.
- **Custom targets.** Custom CMake targets must not be defined under `fw/`: G3 treats every target there as a flight
  target, and it refuses the UTILITY type.
- **Goldens.** Goldens are regenerated only inside the `marv-ci` image, with the explicit command recorded next to
  them (see `tests/regression/quad/L00/replay/CMakeLists.txt`); CI never regenerates.
- **Parallel workers.** Worktrees created for parallel workers start from `master`, not from the current branch, so
  fast-forward them to the working branch first.
- **Scope.** Keep changes scoped to the layer being executed; report anything noticed but not changed.
