# Handoff: quad L4 passed (one tracked failing item), next is quad L5

2026-09-30. Written for the next agent working in this repository. This file records state; it adds no scope. The
specs remain the only planning documents (core §0 rule 5). Replace this file at the next handoff.

## Read first, in this order

1. `CLAUDE.md`: the rules CI enforces (ACTIVE-spec rule, number rule, UNKNOWN rule, gates G1–G8), commands,
   conventions.
2. `docs/spec/00-core-contracts.md` (ACTIVE): §2 numbers and provenance, §3 conventions, §4 firmware boundary, §7
   testing, freezing and the §7.5 convergence rule, Appendix A.
3. `docs/spec/10-quad-flight-software.md` (ACTIVE): §4, the spine (L0–L10), and §6.2. **§4 "L5 — Attitude loop on
   truth attitude" is your next step.**
4. `docs/spec/20-ground-segment.md` and `30-rocket.md` are PARKED: read them for context, never implement from them.
5. `docs/decisions/0001`–`0005`. **Decision 0005 holds every L4 choice**, including twelve owner decisions quoted
   verbatim and the known failing item below. Read it whole before touching the rate loop, the gains or any L04 test.

Luis (the owner) makes the final calls. Numbers you cannot source are tagged `UNKNOWN` and you stop to ask.

## Known failing item (safety): read this first

**Gyroscopic coupling in combined full-stick input (0005 owner decision 12).**
- **Behaviour.** At full-stick rates on all three axes (670 °/s), the L4 PI baseline (ω_c 8.3 rad/s) cannot reject
  the gyroscopic coupling ω×Jω:
  - roll falls from its 11.7 rad/s setpoint to 0.09 rad/s;
  - the integrator winds up with no mixer saturation (s = t = 1 throughout), so anti-windup correctly does not
    freeze it;
  - after the stick centres, roll reaches 10.3 rad/s, pitch 4.75 and yaw 1.79.
- **Evidence.** `tests/regression/quad/L04/results/acro_cause/`, from a bit-exact replay of the firmware on the
  logged gyro samples.
- **The test.** `tests/regression/quad/L04/gz/test_t4_acro.py`, the recovery check. It is a strict xfail
  (`raises=AssertionError`) whose condition is "`tests/regression/quad/L06/` does not exist":
  - once any L06 directory exists, it becomes a normal test and must pass;
  - an unexpected pass fails CI.
- **Luis's rule.** L4 may be tagged with this item open. **L6 cannot pass until it passes, and it must pass before
  L8 (pilot in the loop).** At L6, evaluate ω×Jω feed-forward together with the D-term design, then re-run this
  check. Do not re-seed the recovery envelope from the wound-up controller state: Luis rejected that, because it
  defines the uncommanded rate as correct.

## Current state

- **Branch and tag.** `master` holds the L4 work and this handoff, pushed to `github.com:Lmx154/MARV-V2`. The tag `quad-L4-pass` is
  on `daac5d9`, where GitHub Actions run 36722493579 is green. Both CI
  scripts pass on a clean copy with the work on a branch (so the regression-change check sees it against `master`):
  - `ci/run_ci.sh` in `marv-ci`: 42 steps, ALL STEPS PASSED, ctest 314/314, regression-change check: 2 frozen files covered by 0005;
  - `ci/run_ci_gz.sh` in `marv-ci-gz`: 8 steps; the L04 gz suite gives 40 passed, 1 xfailed (the known failing item), 0 skipped.
  Earlier tags: `quad-L3-pass` on `dcaf2fb`, `quad-L2-pass` on `320cf18`, `quad-L1-pass` on `b8ed690`,
  `quad-L0-pass` on `6cca7ec`.
- **Branches and worktrees.** Only `master` exists. There are no local worktrees.
- **Frozen files changed at L4.** Two L01 tests were generalised from snapshot checks to the manifest rule of 0004.
  Luis approved it, and 0005 "What changed" records it.

## What L4 built (quad spec §4 L4)

| Area | Where | Notes |
| --- | --- | --- |
| Registers | `design/budget.yaml`, `design/scenario_values.yaml` | Budget: `tau_robustness_band` (±30 %) and `inertia_robustness_band` (±50 %), Luis's stand-ins for the card's UNKNOWN σ. New product scenario register: the tick 625/4 µs, `rate_loop_divisor` 2 (3.2 kHz), `rate_max_*` 670 °/s (Betaflight 4.5.2). `flatten.py --scenario`. |
| Gain generator | `tools/card/rate.py` (`flatten.py --out-rate`) | Loopshape's PI structure; the crossover is the largest with worst-case PM ≥ `PM_min` over the τ×J band box, on the exact ZOH-discretised loop. Result: ω_c 8.32 rad/s, PM nominal 52.06°, worst 45.000001°. kd = 0 (D design moves to L6). τ_ref = max(τ_cl, ω_max/α_max) = 0.1609 s on every axis, the PI baseline, not a freestyle target. The derivation report is written at build. |
| QF-8 curve | `tools/card/rate_qf8.py`, `tests/regression/quad/L04/results/qf8/` | Crossover against loop rate, 6.4 kHz down to 12.5 Hz, as a fixed-input golden. QF-8 stays open until motor τ σ exists. |
| Rate loop | `fw/rate/` (`marv_rate`) | A first-order setpoint prefilter (τ_ref), parallel PID, forward-Euler integrator deferred one execution, D on measurement. |
| Rate loop: anti-windup | `fw/rate/` | Clamps with Luis's three-condition predicate and the (γ₄+ε) bound. |
| Rate loop: faults | `fw/rate/` | Non-finite or invalid inputs, or a non-finite output, reset the loop and output zero torque; a fault flag latches. |
| Rate loop: period check | `fw/rate/` | dt from stamps, checked exactly to ±1 µs at half-µs resolution. `from_params()` and `load_config()` wire the parameters. |
| T3 | `tests/regression/quad/L04/t3/` | Float loop against the double design plant. A fixed-input golden with a derived rounding tolerance (6.33e-4 rad/s; observed 9.8e-6). Controls: gains ×1.1 fails at 582×, +1 tick at 6.5×. The 17×17 band envelope is recorded. CI reproduces the golden. |
| Truth gyro | `sim/gz/adapter` (`marv::truth::TruthGyroSilCommandSource`), `<gyro_source>truth</gyro_source>` | Optional plugin element. The log records the IMU sample actually passed. L2 behaviour and logs are unchanged. |
| Test composition | `fw/compositions/l4_rate_scripted`, `sim/l4_rate_scripted`, preset `host-gz-l4` | Scripted setpoints (K = 8 segments), a chirp at the plant input and a collective thrust, all from its own scenario register appended to the product set. No SIL ABI change. |
| T4 runner | `tools/sim/l4_scenario.py`, `tools/sim/run_l4.py`, `scenarios/quad/L04/` | Separate from the frozen L2 schema. |
| T4 tests | `tests/regression/quad/L04/gz/` | Steps: inside the live band envelope ± (E + F). Chirp: measured PM 52.15°, 7.10° above `PM_min`. Acro, with a bit-exact DShot replay tool under `tests/regression/quad/L04/replay/`. Every run is labelled "truth-fed, perfect-model". |

## How to verify

```
uv sync --frozen
cmake --preset host-debug && cmake --build --preset host-debug && ctest --preset host-debug
cmake --preset m33 && cmake --build --preset m33
cmake --preset host-gz && cmake --build --preset host-gz && uv run pytest tests/regression/quad/L02 -q -rs
cmake --preset host-gz-l4 && cmake --build --preset host-gz-l4 && uv run pytest tests/regression/quad/L04/gz -q -rs
docker build -t marv-ci -f ci/Dockerfile . && docker build -t marv-ci-gz -f ci/Dockerfile.gz .
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci ci/run_ci.sh      # verified, part 1
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci-gz ci/run_ci_gz.sh  # verified, part 2
```

Run the Docker commands on clean copies if a host `build/` exists. Use
`rsync -a --exclude build --exclude .venv --exclude __pycache__ --exclude .claude ./ <dir>/`, one fresh copy per
image, and always with `-u`. `marv-ci-gz` is built `FROM marv-ci`, so build that first. To exercise the
regression-change check locally, commit inside the copy **on a new branch** (`git checkout -b check`) and pass
`-e MARV_CI_BASE_REF=master`. A commit on `master` itself leaves nothing to compare.

The L04 gz suite expects 1 xfailed (the known failing item) and 0 skipped. It takes about 2.5 minutes.

This host has gz-sim 8 and 10 and libdart 6.13 and 6.16 side by side. Always use `gz sim --force-version 8`; the
runner does. gz prints hundreds of libprotobuf "already exists in database" lines on every start. They are noise.
`clang-tidy-18` is not installed on the host, so the tidy half of G1 runs only in `marv-ci`.

## Next step: quad L5

Execute quad spec §4 **L5 — Attitude loop on truth attitude**. Its Builds, Opening, Pass bar and Freezes are the
scope. Check these before building, and ask Luis where the spec leaves a choice:

- **Design rule.** The attitude controller is "designed on the exact closed rate loop at the same phase margin":
  loopshape's `outerLoop` around T = L/(1+L).
  - Use the L4 design model: the exact sampled-data loop of `tools/card/rate.py`, not loopshape's e^{−sT}.
  - Decide how the τ×J band box applies to the outer loop, as it did for QF-3 at L4.
- **Its rate.** "Chosen by SIM-7", the convergence rule.
- **Truth attitude.** It needs a route into the composition, the counterpart of the L4 truth-gyro source, and no
  SIL ABI widening.
- **Pass bar interaction.** "The L4 suite still green" includes the L4 known failing item, as an expected failure.
  A large-angle recovery scenario may hit the same gyroscopic coupling limit; report it rather than tune around it.
- **New product-set parameters.** Commit an `L05/param_ids` manifest (0004).
- **Luis's rule on tests.** Frozen tests assert rules, never snapshots of shared repo state. Goldens are allowed only
  for a fixed input the test owns (the L4 T3 and QF-8 goldens show the pattern).

## Carried forward (not L5 unless the spec says so)

- **L4 notes.**
  - QF-8 is open until motor τ σ exists. The listed rates being IMU ODRs is INFERRED (the profile records only a
    range).
  - The D term (kd = 0 now), its filter rule and the crossover cap move to L6. So does ω×Jω feed-forward (the known
    failing item).
  - The acro bound (iii) is a constant peak bound per axis: loose for idle-axis crosstalk, not vacuous.
  - The replay fidelity gate is DShot-quantised. A one-ulp change in kp_pitch is not detected.
  - The composition's extended parameter set relies on link order. A mis-order fails loudly. At L9, a flight
    composition uses the product set only.
  - The step-test predicate cannot resolve an envelope shift below 48 executions (15 ms), because F is 98 %
    grid-convergence H. Exact stamp alignment is guarded separately.
  - The QF-8 header mode still reads the card for mixer and rotor entries.
- **Snapshot-style frozen tests in L00–L03 (Luis asked for the list; nothing changed yet).**
  - L01 `test_flatten_report.py`: the UNKNOWN budget set (:31, :134-160), nothing locked (:206, :218, :238, :455),
    inertia σ (:214), batch_plan UNKNOWN (:389), the report's last line (:393), the length tie (:196).
  - L01 `test_card_lint.py:482`: the mass conflict still open.
  - L02 `test_run_scenario.py:358, 385-395`: batch_plan UNKNOWN.
  - L03 `test_mixer_params.py`: the exact mixer name sets (:164, :267) and literal-text card edits (:34-36, :77,
    :241, :256).
  - L00 `test_lint_g1.py:98-104`: the constants literal filter.
  - Borderline: pinned card values in L01, L02 and L03.
- **Spec gaps added at L4** (for Luis; 0005 lists them):
  - QF-8 ignores filtering and aliasing.
  - "rate PID" against the rule's PI.
  - QF-2's "thrust margin × arm" and "achievable time constant" are undefined.
  - The L4 opening gains a fault output and allocation feedback.
  - QF-3 names no gain margin.
  - QF-3 is checked across register bands, not card σ.
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

- **L4 runs.** Use `tools/sim/run_l4.py`. It refuses a world tick that differs from the scenario register, since
  the rate loop's period check would panic. The L4 plugin build is `build/host-gz-l4`. The L2 build is still
  `build/host-gz`, and `test_truth_gyro.py` uses it.
- **Replaying a run.** `tests/regression/quad/L04/replay/l4_acro_replay` re-executes the composition on logged
  samples and must match the logged DShot bit for bit. Use it to recover internals (flags, torques) the log does not
  carry.
