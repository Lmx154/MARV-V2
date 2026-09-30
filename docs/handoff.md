# Handoff: quad L2 passed, next is quad L3

2026-09-30. Written for the next agent working in this repository. This file records state; it adds no scope. The
specs remain the only planning documents (core §0 rule 5). Replace this file at the next handoff.

## Read first, in this order

1. `CLAUDE.md`: the rules CI enforces (ACTIVE-spec rule, number rule, UNKNOWN rule, gates G1–G8), commands,
   conventions.
2. `docs/spec/00-core-contracts.md` (ACTIVE): shared contracts. The sections that matter most here:
   - §2 numbers and provenance;
   - §3 conventions (logical motor numbers, spin directions in the card);
   - §4 firmware boundary (actuator values: DShot 0 or 48–2047);
   - §7 testing, freezing and the §7.5 convergence rule.
3. `docs/spec/10-quad-flight-software.md` (ACTIVE): §4, the build order (the spine, L0–L10), and §6.2 (QF-2, QF-3,
   QF-6). **§4 "L3 — Mixer / allocation" is your next step.**
4. `docs/spec/20-ground-segment.md` and `30-rocket.md` are PARKED: read them for context, never implement from them.
5. `docs/decisions/0001`–`0003`. Decision 0003 holds every L2 choice: Luis's (items 1–7), the lead's (items 8–11),
   and the gz-sim 8 behaviour found while building L2.

Luis (the owner) makes the final calls. Numbers you cannot source are tagged `UNKNOWN` and you stop to ask.

## Current state

- **Branch and tag.** `master` holds the L2 work (`320cf18`) plus this handoff, pushed to `github.com:Lmx154/MARV-V2`.
  The tag `quad-L2-pass` is on `320cf18`, where GitHub Actions run 36674899399 is green. Both CI scripts also pass
  locally: `ci/run_ci.sh` in `marv-ci` (38 steps; the 39th, the regression-change check, runs only with
  `MARV_CI_BASE_REF`) and `ci/run_ci_gz.sh` in `marv-ci-gz` (6 steps). Earlier tags: `quad-L1-pass` on `b8ed690`,
  `quad-L0-pass` on `6cca7ec`.
- **Branches and worktrees.** Only `master` exists. There are no local worktrees.
- **Decision 0003** (approved by Luis on 2026-09-30 with the push):
  - **CI.** A second pinned image, `marv-ci-gz` (`ci/Dockerfile.gz`, `FROM marv-ci`). It holds gz-sim 8.15, the
    dartsim physics plugin 7.8, DART 6.13.2, sdformat 14.9 and cppzmq, and runs only the Gazebo steps.
  - **SIM-3** is measured and recorded. Its required value is `UNKNOWN` (`design/budget.yaml` `batch_plan`), so SIM-3
    stays open.
  - **Adapter test scope.** The adapter test compares every `marv_plant` v0 output. Sensor bytes join at L6, by a new
    decision record.
  - **Q-D4.** It is decided as generated world variants: pilot at real-time factor 1, test at 0.
  - **L2 command source.** The `l2_open_loop` composition (DShot per motor from parameters set by the SIL override).
    The plugin always steps firmware in-process.
  - **"Exact" means** bit-exact for vectors and every plant output. The attitude quaternion is held to a derived
    rounding bound.
  - **Hover** is checked as a DShot bracket. It passes if the hover command lies between D_lo and D_hi, and each
    passes against a tick-held reference (end-of-tick thrust, which the L1 plant ABI defines).
  - **Per tick:** one SIL tick, then one plant call. The mean wrench is applied per host step.
  - **The T4 rule.** Halving over m = 4, 2, 1 ticks per host step, with Aitken/Richardson extrapolation and a roundoff
    floor. Each quantity is read at the last fresh step.
  - **Determinism.** Byte-identical plugin logs. The control is the SDF's ixx set one ulp higher.
  - **gz-sim 8 behaviour, and the fixes:**
    - Link velocity commands are reset to zero and kept, so the plugin removes them after step 0.
    - State updates are skipped below a 1e-6 pose change, so reads can be stale.
- **Owner decisions carried from L1** (card, budget, decisions 0001/0002): the σ policy, mass 0.772 kg `UNVERIFIED`
  (0.752 kg conflict), spin directions m1/m2 ccw and m3/m4 cw `INFERRED`, the ESC map linear in ω (a labelled scenario
  value), and `marv_plant` in double.
- **Draft 0 and earlier projects** are off limits (core §1).

## What L2 built (quad spec §4 L2)

| Area | Where | Notes |
| --- | --- | --- |
| Open-loop composition | `fw/compositions/l2_open_loop/`, `sim/l2_open_loop/` (SIL library `marv_sil_l2_open_loop`) | Parameters `ol_dshot_m1..m4` (method `scenario`, σ `choice`, default 0), validated at init. An illegal value calls `hal_panic`. |
| Adapter (host-free) | `sim/gz/adapter/` (`frames.hpp`, `adapter.hpp`, cited constants) | ENU↔NED and FLU↔FRD maps, the attitude map q_nb = ŝ·(−(w+z), −(x+y), y−x, z−w), and ω_frd = R(q)ᵀ·perm(ω_world). The per-tick loop: `SilCommandSource` or `ScriptedCommandSource`, then `marv_plant_step`, then W̄ summed from W_0 and divided once. Static and PIC. |
| Lockstep plugin | `sim/gz/plugin/` (`marv_gz_lockstep` / `marv::gz::Lockstep`), built only with `MARV_GZ=ON` (presets `host-gz`, `host-gz-release`) | The SIL and parameter set are chosen by the cache variables `MARV_GZ_SIL` / `MARV_GZ_PARAMS`, so a composition swap needs no code change. Configure-time refusals are `std::abort` with `marv_gz_lockstep: REFUSED: <reason>`. It checks dt and simTime every step, and removes the velocity commands after step 0. The binary log is little-endian, one flush per host step, with a trailer and no wall-clock or path bytes (`lockstep_log.hpp`, `tools/sim/lockstep_log.py`). The env var `MARV_GZ_TEST_KEEP_VEL_CMD` exists only for a negative control. |
| Worlds | `tools/card/gen_world.py` | (card, scenario, mode, m) → SDFormat 1.11 world: dartsim named, gravity 0, max_step_size = m·tick, the plugin element schema in its docstring, and the initial pose in ENU/FLU. No ground plane or geometry: no L2 scenario needs contact, and a GUI shows nothing yet (Q-D1). |
| Scenarios | `scenarios/quad/L02/{free_fall,rotation,hover,determinism}.yaml`, schema `tools/sim/scenario.py` | Every number is labelled or derived. Tick 625/4 µs (the L0 scenario value), site φ = 0.82 rad, h0 = 500 m, m sequence [4,2,1], durations in ticks. The hover bracket is D_lo = 765 and D_hi = 766 (`tools/sim/hover.py`). |
| Runner | `tools/sim/run_scenario.py` (CLI and module) | `run`, `run_sequence`, `measure_rtf`, run reports: build_report, commit, card and scenario hashes, versions (gz-sim, gz-physics, DART, sdformat), stale-step count, halving section. One gz process per run (one SIL per process), with a unique `GZ_PARTITION`. |
| T4 rule and references | `tools/sim/t4_rule.py`, `tools/sim/reference.py` | The item 9 rule and roundoff floors. Free-fall ODE with the g(h) gradient; continuous and tick-held hover references; rotation invariants. Each reference's F_ref is shown ≤ F by halving. |
| Frozen suites | `tests/regression/quad/L02/` | ctest (label frozen): `unit/composition`, `unit/adapter`. pytest: `tools/` (223 tests), `gz/test_plugin_smoke.py` (17), `gz/test_determinism.py` (3), `gz/test_analytic.py` (24). The pytest suites run only in `ci/run_ci_gz.sh`, which fails on any skip. |
| SIM-3 | `tools/sim/measure_sim3.py`, `tests/regression/quad/L02/results/sim3/` | Release build, free fall, m = 1, 1 s. The marginal RTF per core is about 9–15 (±25%: a 1 s run measures little beyond the ≈ 2.1 s startup), on one core. Required: `UNKNOWN`. |
| CI | `ci/Dockerfile.gz`, `ci/run_ci_gz.sh`, `.github/workflows/ci.yml` | Steps: gz_toolchain, gz_build, gz_plugin_smoke, gz_determinism, gz_runner_tools, gz_analytic. `cmake/flight_targets.cmake` excludes the plugin from the G3 SIL export list (0003 item 11). |

L2 results, for reference:

| Check | Result |
| --- | --- |
| Free fall, p_D | Branch (ii), observed order ≈ 1.0 |
| Free fall, v_D | Branch (i) |
| Rotation, \|L\| and KE | Branch (ii), observed order ≈ 1.0 |
| Hover, p_D | Branch (ii), both D |
| Hover, v_D and a_D | Branch (i), both D |
| Hover logged thrust against the tick-held reference, per tick | 0 violations, worst tick at 0.28 of the floor |
| Determinism | Log SHA-256 `6b124cb9…075a`, the same on host and image |

The L0, L1 and L2 interface decisions are frozen: the declarations in the headers above are the contract. Changing
one needs a decision record in `docs/decisions/` (template `0000-template.md`, next number 0004). So does modifying or
deleting anything under `tests/regression/`, including `L02/`.

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
with `-u`. `marv-ci-gz` is built `FROM marv-ci`, so build that first.

This host has gz-sim 8 and 10 and libdart 6.13 and 6.16 side by side. Always use `gz sim --force-version 8`; the
runner does. gz prints hundreds of libprotobuf "already exists in database" lines on every start. They are noise.

## Next step: quad L3

Execute quad spec §4 **L3 — Mixer / allocation**. Its Builds, Opening, Pass bar and Freezes are the scope. All of its
pass bar is T1. Check these open items before building, and ask Luis where the spec leaves a choice:

- **The idle floor.**
  - L3 says "idle floor at the card's minimum stable rotor speed", and QF-6 says "the measured minimum stable rotor
    speed". The card has only `speed_range` [150, 2800] rad/s (published, blackbird `motor_omega_min`, σ `UNKNOWN`).
    Under the ESC scenario map, that is also the speed at DShot 48.
  - Whether the idle floor is that entry, or a separate measured value (none exists yet, so `UNKNOWN`), is Luis's
    call.
- **"Within float tolerance".** The identity check (mixer ∘ effectiveness) needs a derived bound with its rule, as the
  L2 adapter bounds have. The same gap is open for L10.
- **Output path.** Wrench → rotor-speed request → DShot needs the inverse of the ESC map. That map is a labelled
  scenario value, linear in ω, and L2's `tools/sim/hover.py` already inverts it in Python. The firmware version must
  take k and the map from parameters: no literals under `fw/` (G1).
- **Sign contract.** +yaw raises the ccw motors, per the card: m1 and m2, `INFERRED`, with B1 replay checking the
  sign. The card's `rotor_yaw_sign_m1..m4` parameters are +1, +1, −1, −1.
- **Desaturation.** The priority (roll and pitch over yaw) and the air-mode behaviour (QF-6) must be documented before
  the tests are written.
- **L3 has no T4 item.** Flying the mixer in Gazebo means a new composition plus `MARV_GZ_SIL` / `MARV_GZ_PARAMS`. The
  plugin code does not change.

## Carried forward (not L3 unless the spec says so)

- **SIM-3.** Open until Luis sets `batch_plan` (core C-6). The measurement exists only for a Debug and a Release 1 s
  free-fall run. A 10 s run of the determinism scenario trips a DART assertion (`Skeleton::computeForwardDynamics`,
  probably constant torque spinning the body up without bound); the committed runs are 1–2 s.
- **CI pinning.** `marv-ci-gz` pins only the gz top-level packages, DART and cppzmq. About 300 dependencies they pull
  in float within noble, and the OSRF repository keeps only recent versions: a pinned version that disappears fails
  the image build loudly. So the determinism promise ("pinned tool versions") holds only partly.
- **gz-sim 8 stale reads** (0003 item 11). Any later T4 test that reads state must use the fresh-read rule. The
  plant's own input can also be stale by up to 1e-6. That is harmless at L2, but a layer whose scenario torque depends
  on attitude must bound it.
- **The pass rule's slack.** Rule (ii)'s bound includes E, so a reference offset of about 0.5·E can pass. The negative
  controls are far outside that.
- **L6.** Sensor bytes join the adapter test (a new decision record). Sensor evaluation before the motor advance
  needs a `marv_plant` ABI change (decision record).
- **QF-4.** The plant has zero actuator latency: the command of tick j acts on tick j. DShot frame and ESC delay
  belong inside `marv_plant`.
- **L9.** As before:
  - `__aeabi_uldivmod` and `memcpy` are flash-resident on the M33.
  - Core 1 and `hal_panic` must run from SRAM (EMB-1).
  - The motor behaviour on a target `hal_panic` is an open flight-safety decision.
- **B1.** As before:
  - the mass conflict;
  - the spin directions and yaw-torque sign;
  - the `thrust_map` reading;
  - blackbird's `thrust_max` of 8.5 N against k·2800²;
  - σ `UNKNOWN` for every card entry except mass.
- **Spec gaps, not yet fixed (for Luis):**
  - L7's attitude-error budget has no rule.
  - Q-D6's criterion does not match the L6 Allan check.
  - L10's "within float tolerance" needs a horizon and a metric.
  - Core §2.1's example card uses rejected field shapes.
  - Core §9 lists RK4 and interpolation at L1.
  - Core §3's double rule does not mention simulator code.
  - New: quad §4 L2 says the adapter compares "every sensor byte", which v0 cannot do (0003 item 3); and "ENU↔NED
    round-trips are exact" holds only for vectors (0003 item 6).
- **Known, accepted gate limits.** As before, plus:
  - G1 and G3 do not cover `sim/gz` (outside `fw/`). The number rule there is kept by review.
  - `sim/gz/CMakeLists.txt` sets PIC on `marv_prim` and `marv_types` when `MARV_GZ=ON`.
  - The `run_ci.sh` frozen step does not run the L02 pytest suites; only `run_ci_gz.sh` runs them.

## Working notes

- **Numeric literals.** As before: every literal under `fw/` other than 0, 1, 2 and ½ fails CI. Cited constants go in
  `constants.hpp`, with a `Citation:` and a `Kind:`. Vehicle numbers go in the card.
- **Card YAML.** Load it only through `tools/card/schema.py`.
- **Custom targets.** Custom CMake targets must not be defined under `fw/`.
- **Goldens.** Regenerated only inside the image, with the command recorded next to them.
- **Gazebo tests.** Run each scenario in its own gz process: one SIL init per process. Use the runner's
  `run_gz_process`, which sets `GZ_PARTITION`, `GZ_IP=127.0.0.1` and the plugin path. A skipped pytest in the gz
  image is a failure.
- **Parallel workers.** Worktrees start from `master`. Fast-forward them to the working branch first.
- **Scope.** Keep changes to the layer being executed. Report anything noticed but not changed.
