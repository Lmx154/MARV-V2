# Handoff: quad L3 passed, next is quad L4

2026-09-30. Written for the next agent working in this repository. This file records state; it adds no scope. The
specs remain the only planning documents (core §0 rule 5). Replace this file at the next handoff.

## Read first, in this order

1. `CLAUDE.md`: the rules CI enforces (ACTIVE-spec rule, number rule, UNKNOWN rule, gates G1–G8), commands,
   conventions.
2. `docs/spec/00-core-contracts.md` (ACTIVE): shared contracts. The sections that matter most here:
   - §2 numbers and provenance;
   - §3 conventions (logical motor numbers, spin directions in the card);
   - §4 firmware boundary;
   - §7 testing, freezing and the §7.5 convergence rule;
   - Appendix A (what to take from the toolbox, including `loopshape.ts`).
3. `docs/spec/10-quad-flight-software.md` (ACTIVE): §4, the build order (the spine, L0–L10), and §6.2 (QF-2, QF-3,
   QF-4, QF-8). **§4 "L4 — Rate loop on truth gyro (PID)" is your next step.**
4. `docs/spec/20-ground-segment.md` and `30-rocket.md` are PARKED: read them for context, never implement from them.
5. `docs/decisions/0001`–`0004`. Decision 0004 holds every L3 choice: the idle floor, the identity tolerance rule, the
   desaturation algorithm, the extra opening outputs, and the per-layer parameter-id manifests.

Luis (the owner) makes the final calls. Numbers you cannot source are tagged `UNKNOWN` and you stop to ask.

## Current state

- **Branch and tag.** `master` holds the L3 work plus this handoff, pushed to `github.com:Lmx154/MARV-V2`.
  The tag `quad-L3-pass` is on `dcaf2fb`, where GitHub Actions run 36679221985 is green. Both CI scripts also pass
  locally on a clean copy: `ci/run_ci.sh` in `marv-ci` (40 steps, including the regression-change check with
  `MARV_CI_BASE_REF=master`) and `ci/run_ci_gz.sh` in `marv-ci-gz` (6 steps).
  Earlier tags: `quad-L2-pass` on `320cf18`, `quad-L1-pass` on `b8ed690`, `quad-L0-pass` on `6cca7ec`.
- **Branches and worktrees.** Only `master` exists. There are no local worktrees.
- **Decision 0004** (approved by Luis on 2026-09-30 with the push):
  - **Idle floor.** `idle_speed` is derived from the card's `speed_range.min` (150 rad/s, published, σ `UNKNOWN`). It
    stands in for QF-6's measured minimum stable speed, which does not exist yet; a measured value replaces it with no
    mixer change. A test holds it equal to the plant's `omega_min`.
  - **Mixer storage.** The generator (`tools/card/mixer.py`, through `flatten.py --out-mixer`) computes M = B⁻¹ in
    double. B is in per-motor thrust, column i = [1, −y_i, x_i, s_i·c_q]. M is stored as 16 f32 parameters
    `mixer_m<i>_{thrust,roll,pitch,yaw}`, method `derived`, σ `UNKNOWN`. The generator refuses a singular B, a
    thrust-column entry ≤ 0, or a card where zero torque is infeasible between idle and maximum speed.
  - **Identity tolerance.** Per element, |fl(M̂B̂) − I| ≤ (γ₄ + ε)(|M||B|), with ε = float32 epsilon and
    γ_n = nε/(1−nε) (Higham §3.5). This rule is for L3 only; L10 needs its own.
  - **Desaturation.** Allocation is in per-motor thrust in [f_min, f_max] = [k·ω_idle², k·ω_max²]. The priority is
    roll/pitch > yaw > collective:
    - s scales roll and pitch together; when s < 1, yaw is zero;
    - otherwise t clips yaw only, and the collective shifts for yaw too (air mode at zero and full throttle);
    - the collective c is the thrust request clamped to its feasible interval.
    s and t come from a closed pairwise form, with no iteration. A final comparison clamp sends NaN to f_min.
  - **Output path.** ω = √(f/k), then the inverse of the linear-in-ω ESC map (a labelled scenario value), rounded to
    nearest and clamped to [⌈D(ω_idle)⌉, 2047].
  - **Opening outputs.** Besides DShot per motor: the roll, pitch and yaw saturation flags (achieved < requested), for
    L4's anti-windup, and the achieved collective thrust in N (for B2).
  - **Parameter-id manifests.** The frozen L01 product-set check now asserts that the built `ParamId` set equals the
    union of every `tests/regression/quad/Lnn/param_ids`, with the manifests disjoint. A layer that adds product-set
    parameters commits its own manifest and does not edit L01.
- **Owner decisions carried from L1 and L2** (card, budget, decisions 0001–0003): the σ policy, mass 0.772 kg
  `UNVERIFIED` (0.752 kg conflict), spin directions m1/m2 ccw and m3/m4 cw `INFERRED`, the ESC map linear in ω (a
  labelled scenario value), `marv_plant` in double, the second CI image `marv-ci-gz`, and the T4 halving rule.
- **Draft 0 and earlier projects** are off limits (core §1).

## What L3 built (quad spec §4 L3)

| Area | Where | Notes |
| --- | --- | --- |
| Mixer parameters | `tools/card/mixer.py`, `tools/card/flatten.py` (`--out-mixer`), `fw/params/CMakeLists.txt` | `idle_speed` plus 16 mixer entries join the product set `marv_params` as a third params_gen input. Without `--out-mixer`, flatten's output is byte-identical to L2. The inversion is plain Python (Gauss–Jordan with partial pivoting); numpy is not a dependency. |
| Mixer module | `fw/mixer/` (`marv_mixer`, `marv/mixer/mixer.hpp`) | Scalar-templated, float instantiated. `mix()` is the L3 opening: DShot per motor, flags, achieved thrust. `allocate()` and `thrust_to_dshot()` are its two stages. `load_config()` = `from_params()` + `require_valid()`, which calls `hal_panic` naming the violated rule (`ConfigError`). |
| Frozen suites | `tests/regression/quad/L03/` | ctest (label frozen): `unit/mixer` (`unit_l3_mixer`, 14 tests). pytest: `tools/test_mixer_params.py` (12 tests), run by `ci/run_ci.sh`. Manifest `param_ids`. |
| Frozen L01 edit | `tests/regression/quad/L01/tools/test_flatten_report.py`, `tests/regression/quad/L01/param_ids` | Recorded in 0004. |

The L3 tests: identity within the 0004 bound, with a negative control (one M entry perturbed by 16× its bound); the
sign contract on f and DShot; case a (s against a double bisection oracle, roll:pitch ratio kept, yaw zero); case b
(full roll/pitch at T = 0 and at T = Σf_max); case c (full yaw at zero thrust, yaw clipped alone beyond headroom);
idle (every motor at 48 on the card); the inverse ESC map round-trips every DShot command; a seeded randomized sweep
(20000 samples, NaN and ±inf included) in which every motor stays in [f_min, f_max] and [48, 2047], with a negative
control that plants seven violations; and config validation, including a death test.

The L0–L3 interface decisions are frozen: the declarations in the headers above are the contract. Changing one needs a
decision record in `docs/decisions/` (template `0000-template.md`, next number 0005). So does modifying or deleting
anything under `tests/regression/`.

## How to verify

```
uv sync --frozen
cmake --preset host-debug && cmake --build --preset host-debug && ctest --preset host-debug
cmake --preset m33 && cmake --build --preset m33
cmake --preset host-gz && cmake --build --preset host-gz && uv run pytest tests/regression/quad/L02 -q -rs
docker build -t marv-ci -f ci/Dockerfile . && docker build -t marv-ci-gz -f ci/Dockerfile.gz .
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci ci/run_ci.sh      # verified, part 1
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci-gz ci/run_ci_gz.sh  # verified, part 2
```

Run the Docker commands on clean copies if a host `build/` exists. Use
`rsync -a --exclude build --exclude .venv --exclude __pycache__ ./ <dir>/`, one fresh copy per image, and always
with `-u`. `marv-ci-gz` is built `FROM marv-ci`, so build that first. To exercise the regression-change check
locally, commit inside the copy and pass `-e MARV_CI_BASE_REF=master`.

This host has gz-sim 8 and 10 and libdart 6.13 and 6.16 side by side. Always use `gz sim --force-version 8`; the
runner does. gz prints hundreds of libprotobuf "already exists in database" lines on every start. They are noise.
`clang-tidy-18` is not installed on the host, so the tidy half of G1 runs only in `marv-ci`.

## Next step: quad L4

Execute quad spec §4 **L4 — Rate loop on truth gyro (PID)**. Its Builds, Opening, Pass bar and Freezes are the scope.
Its pass bar has T3 and T4 items, so it is the first layer since L2 to fly in Gazebo. Check these open items before
building, and ask Luis where the spec leaves a choice:

- **Gains.** Gains come from the loop-shaping rule (the toolbox's `loopshape.ts`, core Appendix A) on the card's
  linear design model at the chosen loop rate. No typed gains. The loop rate is QF-8's convergence rule; whether L4
  fixes it or uses the tick rate as a labelled scenario value is Luis's call.
- **Design inputs with σ `UNKNOWN`.** QF-3's "margins still meet `PM_min` with motor τ at the card's ±σ" needs the
  motor τ σ, which is `UNKNOWN` on the card. Ask before inventing one.
- **QF-2's τ_ref rule** needs ω_max and α_max = τ_max/J per axis. τ_max follows from the mixer envelope (roll/pitch:
  thrust margin × arm; yaw: rotor torque), and the inertia σ is `UNKNOWN`.
- **Anti-windup.** It consumes L3's saturation flags. Two behaviours are unspecified:
  - a NaN torque request sets no flag and reports a NaN achieved value, though the motors sit at idle (0004 item 5
    specifies only the motor range);
  - a flag can be set with achieved equal to requested when s rounds just below 1.
  Decide what L4 does with both.
- **Gazebo.** Flying L4 needs a new composition (rate loop + mixer) plus `MARV_GZ_SIL` / `MARV_GZ_PARAMS`. The plugin
  code does not change. The composition's parameter set must include the product set, since the mixer reads
  `marv_params`.
- **Stale reads.** A T4 test that reads state uses the fresh-read rule (0003 item 11). An attitude-dependent torque
  makes the plant's stale input matter; bound it.

## Carried forward (not L4 unless the spec says so)

- **L3 notes.**
  - `thrust_to_dshot` computes the idle bound ⌈D(ω_idle)⌉ in float. If a measured idle speed maps exactly onto an
    integer DShot, rounding could put idle one step high (never below). Today it is exactly 48.
  - The randomized sweep checks range and flag consistency only. Optimality of s and t is checked in the case a–c
    scenarios against the bisection oracle.
  - `tools/card/mixer.py` calls the private helper `gen_plant_config._known`.
  - QF-6 asks for a *measured* minimum stable rotor speed; `idle_speed` is the published stand-in until one exists.
- **SIM-3.** Open until Luis sets `batch_plan` (core C-6). A 10 s run of the determinism scenario trips a DART
  assertion (`Skeleton::computeForwardDynamics`, probably constant torque spinning the body up without bound); the
  committed runs are 1–2 s.
- **CI pinning.** `marv-ci-gz` pins only the gz top-level packages, DART and cppzmq. About 300 dependencies float
  within noble, and the OSRF repository keeps only recent versions: a pinned version that disappears fails the image
  build loudly.
- **The T4 pass rule's slack.** Rule (ii)'s bound includes E, so a reference offset of about 0.5·E can pass.
- **L6.** Sensor bytes join the adapter test (a new decision record). Sensor evaluation before the motor advance
  needs a `marv_plant` ABI change (decision record).
- **QF-4.** The plant has zero actuator latency: the command of tick j acts on tick j. DShot frame and ESC delay
  belong inside `marv_plant`.
- **L9.**
  - `__aeabi_uldivmod` and `memcpy` are flash-resident on the M33.
  - Core 1 and `hal_panic` must run from SRAM (EMB-1).
  - The motor behaviour on a target `hal_panic` is an open flight-safety decision. `marv_mixer`'s `require_valid`
    is a new `hal_panic` caller.
- **B1.**
  - the mass conflict;
  - the spin directions and yaw-torque sign (the mixer's yaw column depends on them);
  - the `thrust_map` reading;
  - blackbird's `thrust_max` of 8.5 N against k·2800²;
  - σ `UNKNOWN` for every card entry except mass.
- **Spec gaps, not yet fixed (for Luis):**
  - L7's attitude-error budget has no rule.
  - Q-D6's criterion does not match the L6 Allan check.
  - L10's "within float tolerance" needs a horizon and a metric (0004 says L10 gets its own rule).
  - Core §2.1's example card uses rejected field shapes.
  - Core §9 lists RK4 and interpolation at L1.
  - Core §3's double rule does not mention simulator code.
  - Quad §4 L2 says the adapter compares "every sensor byte", which v0 cannot do (0003 item 3); and "ENU↔NED
    round-trips are exact" holds only for vectors (0003 item 6).
  - New: quad §4 L3's opening lists only DShot as output; 0004 item 4 adds the flags and achieved thrust. L3 says
    "the card's minimum stable rotor speed" and QF-6 says "measured"; 0004 item 1 bridges them.
- **Known, accepted gate limits.**
  - G1 and G3 do not cover `sim/gz` (outside `fw/`). The number rule there is kept by review.
  - `sim/gz/CMakeLists.txt` sets PIC on `marv_prim` and `marv_types` when `MARV_GZ=ON`.
  - `ci/run_ci.sh` does not run the L02 pytest suites (including `L02/tools`); only `run_ci_gz.sh` runs them.

## Working notes

- **Numeric literals.** Every literal under `fw/` other than 0, 1, 2 and ½ fails CI. Cited constants go in
  `constants.hpp`, with a `Citation:` and a `Kind:`. Vehicle numbers go in the card. Test numbers are derived from
  the card or labelled `scenario test value` with a reason.
- **Card YAML.** Load it only through `tools/card/schema.py`.
- **New product-set parameters.** Add the layer's `tests/regression/quad/Lnn/param_ids` manifest (0004).
- **Custom targets.** Custom CMake targets must not be defined under `fw/`.
- **Goldens.** Regenerated only inside the image, with the command recorded next to them.
- **Gazebo tests.** Run each scenario in its own gz process: one SIL init per process. Use the runner's
  `run_gz_process`, which sets `GZ_PARTITION`, `GZ_IP=127.0.0.1` and the plugin path. A skipped pytest in the gz
  image is a failure.
- **Parallel workers.** Worktrees start from `master`. Fast-forward them to the working branch first.
- **Scope.** Keep changes to the layer being executed. Report anything noticed but not changed.
