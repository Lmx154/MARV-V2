# Handoff: quad L5 passed (two tracked failing items), next is quad L6

2026-09-30. Written for the next agent working in this repository. This file records state; it adds no scope. The
specs remain the only planning documents (core §0 rule 5). Replace this file at the next handoff.

## Read first, in this order

1. `CLAUDE.md`: the rules CI enforces (ACTIVE-spec rule, number rule, UNKNOWN rule, gates G1–G8), commands,
   conventions.
2. `docs/spec/00-core-contracts.md` (ACTIVE): §2 numbers and provenance, §3 conventions, §4 firmware boundary, §7
   testing, freezing and the §7.5 convergence rule. **§7.5 now has a "Flight rate groups" paragraph** (owner decision
   0006-21): a flight rate group that no product rule sets runs at its parent group's rate unless that fails EMB-3;
   only then does the convergence rule choose a lower rate.
3. `docs/spec/10-quad-flight-software.md` (ACTIVE): §4, the spine (L0–L10), and §6.2. **§4 "L6 — Sensor models and
   the gyro chain" is your next step.**
4. `docs/spec/20-ground-segment.md` and `30-rocket.md` are PARKED: read them for context, never implement from them.
5. `docs/decisions/0001`–`0008`. **Decision 0006 holds every L5 choice** (25 owner decisions verbatim, lead decisions
   A–H, results, spec gaps, carried-forward items). 0007 is the additive L2 change (plant initial rotor speed) and the
   one approved frozen edit. 0008 is the post-merge `safe.directory` fix to the L5 rate-bypass regenerate script. 0005 holds the L4 choices. Read 0005 and 0006 whole before touching the rate loop, the
   attitude loop, their gains or any L04/L05 test.

Luis (the owner) makes the final calls. Numbers you cannot source are tagged `UNKNOWN` and you stop to ask.

## Luis's instructions for L6 (verbatim, 2026-09-30)

> For the handoff / L6:
> 1. The T3 reference data is 8.6 MB and will grow each layer. Since it's regenerable by rule in the pinned image,
>    propose committing the generator plus a content hash instead of the data (or git LFS). Decide before L6 adds
>    more.
> 2. Still owed: T4 suite wall-clock per layer and the smoke-per-push / full-nightly split.
> 3. L6 is the performance layer: D term, gyro chain, DShot error diffusion, and ω×Jω feed-forward evaluation. Both
>    strict xfails (L4 acro, L5 R2) must pass there. Start L6 by bringing me the D-term design choices.

So the first L6 actions, in order: bring Luis the T3-reference storage proposal (item 1) and the D-term design choices
(item 3), and measure the T4 wall-clock per layer (item 2). Measured so far (this session, marv-ci-gz): L02 gz about
1.7 min, `gz_l4` 2 min 21 s, `gz_l5` 7 min 21 s; the L05 T3 oracle about 50 s host, 2 min 20 s in marv-ci.

## Known failing items (safety): read this first

Both are the same cause class and have the same gating. **L6 cannot pass until both pass, and both must pass before L8
(pilot in the loop).** Each is a strict pytest xfail (`raises=AssertionError`, strict) whose condition is
"`tests/regression/quad/L06/XFAIL_GATE_CLOSED` does not exist" (decision 0009; L6 stage (e) creates it): from then
on it is a normal test and must pass;
an unexpected pass fails CI. Do not re-seed envelopes from wound-up state and do not tune bounds (0005 decision 12,
0006 decisions 5 and 15).

1. **L4 acro, combined full-stick segment** (0005 decision 12). `tests/regression/quad/L04/gz/test_t4_acro.py`,
   evidence `tests/regression/quad/L04/results/acro_cause/`. The PI baseline (ω_c 8.3 rad/s) cannot reject ω×Jω;
   the integrator absorbs then releases it, giving uncommanded rates with the stick centred. No L3 flag set.
2. **L5 R2, inverted and tumbling at rate_max on all axes** (0006 decision 15, new). `tests/regression/quad/L05/gz/
   test_t4_recovery.py`, evidence `tests/regression/quad/L05/results/recovery_cause/` (bit-exact replay, 20747
   executions). No L3 flag, s = t = 1 throughout; |ω×Jω| at the first execution is 41/44/33 % of τ_held and larger than
   any torque the loop requests. Counterfactual distance to gz: design 7.96, + ω×Jω 1.46e-2, + DShot 7.9e-3. The
   vehicle recovers (α < 90° at 0.259 s against 0.124 s in the design model), but leaves the envelope by up to
   0.80 rad and 5.3 rad/s.

Luis: at L6, evaluate ω×Jω feed-forward together with the D-term design, then re-run both checks.

## Current state

- **Branch and tag.** `master` holds the L5 work, pushed to `github.com:Lmx154/MARV-V2`. Luis's condition: tag
  `quad-L5-pass` only once GitHub Actions is green on the pushed head. Actions failed on `e9bfaae` (run 36785715345):
  the new `rate_bypass/regenerate.sh` ran `git archive` as root on a checkout owned by another user, and git refused it
  for dubious ownership. Local CI had used `-u` and hidden this. `d7a044c` passes `safe.directory`. Its run
  (36788978797) then failed the regression-change check: L5 was frozen from `e9bfaae`, and that push carried no
  record. Decision 0008 records the change, with Luis's approval. The tag goes on the head that carries 0008 and this
  handoff, once Actions is green there. **Reproduce GitHub's conditions before pushing: run the CI images
  as root (no `-u`) with `MARV_CI_BASE_REF` set to the push range base.**
  Earlier tags: `quad-L4-pass` on `daac5d9`, `quad-L3-pass` on `dcaf2fb`, `quad-L2-pass` on `320cf18`, `quad-L1-pass`
  on `b8ed690`, `quad-L0-pass` on `6cca7ec`.
- **Verified.** Full CI on clean copies of `quad-l5` at `f56c0fb` (the final commit `e9bfaae` changes only 0006 text),
  with `MARV_CI_BASE_REF=master`:
  - `ci/run_ci.sh` in `marv-ci`: 49 steps, ALL STEPS PASSED; ctest 454/454 (debug and release), frozen 453/453;
    regression-change check: 1 frozen file changed, covered by 2 decision records.
  - `ci/run_ci_gz.sh` in `marv-ci-gz`: 10 steps; `gz_l4` 40 passed, 1 xfailed; `gz_l5` 63 passed, 1 xfailed; 0 skipped.
- **Frozen files changed at L5.** One: `tests/regression/quad/L01/tools/test_gen_sdf_plant_config.py`
  (`initial_omega_rad_s` added to `SCENARIO_FIELDS`, plus a negative control), approved by Luis and recorded in 0007.
  Luis: that test stays a pinned set deliberately; it is not one of the snapshot tests to generalise.
- **Branches and worktrees.** `quad-l5` (merged) and the worker branches `worktree-agent-*` and `quad-l5-p9b` remain
  locally, all merged into `master`; worktrees under `.claude/worktrees/` can be removed.

## What L5 built (quad spec §4 L5; details and numbers in 0006)

| Area | Where | Notes |
| --- | --- | --- |
| Attitude law | `fw/attitude/` (`marv_attitude`) | PX4 reduced-attitude (tilt-prioritised) with yaw-weight compensation (0006 C, decisions 9, 12, 13). One gain k on all axes; w = α_max,yaw / min(α_max,roll, α_max,pitch) ≤ 1 shapes large combined errors only. ρ = 0 branch at exactly 180° tilt. Faults, latch, period check as L4. |
| Angle mode | `fw/attitude/` | Tilt = θ_max·s/max(1,‖s‖), θ_max 60° (Betaflight 4.5.2 `pid.c:138`). Yaw stick outside the deadband is a rate command (world-down) with the heading tracking; on release, heading lock at the first world-down yaw-rate zero crossing; guard (a) one lock per release; guard (b) fallback at max(t_cross, ω_r/α_min) (decisions 14, 17, 18). Deadband 0 (Betaflight default) — revisit at L8. |
| Gains | `tools/card/attitude.py` (`flatten.py --out-attitude`) | Exact sampled-data closed rate loop (bypass), 0005's sup rule on the P gain, band box per decision 1. At 3.2 kHz: k = 3.0872879, PM 69.390° nominal, 45.000018° worst at (J+, τ+), crossover 3.477–4.610 rad/s, w = 0.143256, α_min 83.327 rad/s², t_cross 0.25625 s. `att_loop_ratio` = 1 by the core §7.5 flight-rate-group rule (decision 21; EMB-3 met per §5.4; re-check at L9). |
| Rate-loop change | `fw/rate/` | Additive `execute_bypass` (prefilter skipped for the attitude path; acro unchanged, bit-identity golden from `quad-L4-pass`). |
| Truth attitude | `fw/sil` (`marv_truth.h`, `TRUTH_STATE` option), `sim/gz/adapter` (`marv_gz_truth_attitude`), `<attitude_source>truth</attitude_source>` | Test-only SIL entry `marv_truth_state_set`, exported only by `TRUTH_STATE` libraries; G3 export rule extended with controls. q canonicalised to w ≥ 0. Log record type 5. `AttitudeState<T>` in `fw/types` is L7's opening. |
| Plant | `sim/plant`, plugin, scenarios | `marv_plant_config.initial_omega_rad_s` (default 0, bit-identical), `<initial_rotor_speed_rad_s>`, `initial_state.rotor_speed_rad_s` (`hover` resolved by `run_l5.hover_rotor_speeds`) — 0007. |
| Composition | `fw/compositions/l5_attitude_scripted`, `sim/l5_attitude_scripted`, preset `host-gz-l5` | Stick script, attitude chirp (rad/s at the attitude output), constant yaw disturbance, thrust — its own register appended to the product set. |
| T3 | `tests/regression/quad/L05/t3/`, `L05/tools/` | Fixed-input golden with derived tolerance; controls ×1.1 (×3946 roll/pitch, ×192 yaw) and +1 tick (×46.4, ×8.2); 17×17 grid margins; t_cross and fallback properties; envelopes; Q (DShot quantisation term). |
| T4 | `tests/regression/quad/L05/gz/`, `tools/sim/{l5_scenario,run_l5,gen_l5_chirp}.py`, `scenarios/quad/L05/` | Angle steps, truth plumbing, attitude chirp (roll 68.90°, pitch 68.90°, yaw 69.40°; slack ≥ 21°), yaw release and fallback, recovery R1 (π − 0.01, hover rotors) and exact-180° α check, R2 (strict xfail). All predicates envelope ± (E + F + Q). |

## How to verify

```
uv sync --frozen
cmake --preset host-debug && cmake --build --preset host-debug && ctest --preset host-debug
cmake --preset m33 && cmake --build --preset m33
cmake --preset host-gz && cmake --build --preset host-gz && uv run pytest tests/regression/quad/L02 -q -rs
cmake --preset host-gz-l4 && cmake --build --preset host-gz-l4 && uv run pytest tests/regression/quad/L04/gz -q -rs
cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5 && uv run pytest tests/regression/quad/L05/gz -q -rs
docker build -t marv-ci -f ci/Dockerfile . && docker build -t marv-ci-gz -f ci/Dockerfile.gz .
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci ci/run_ci.sh      # verified, part 1
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci-gz ci/run_ci_gz.sh  # verified, part 2
```

Run the Docker commands on clean copies (`rsync -a --exclude build --exclude .venv --exclude __pycache__ --exclude
.claude ./ <dir>/`, one per image), once with `-u` and once as root without it, as GitHub Actions runs them (a
root-owned copy needs a container to delete it). To exercise the regression-change check, commit in the copy on a
new branch and pass `-e MARV_CI_BASE_REF=master`. `test_truth_gyro.py` (L04 gz) needs `build/host-gz` as well as
`build/host-gz-l4`, or it skips. Expected: L04 gz 40 passed, 1 xfailed; L05 gz 63 passed, 1 xfailed; 0 skipped.

## Findings at L5 that shape L6

- **DShot quantisation at hover** (measured, `L05/results/step_cause/`): hover sits at DShot 765 (765.06 rounded); one
  step is 4.56 mN per motor, a request dead band of ±8.0e-4 / ±6.0e-4 / ±1.8e-4 N·m (roll/pitch/yaw), giving a ±0.006
  rad attitude limit cycle. Q absorbs it at T4 (decision 19). Consequence: T4 angle steps catch gross failures and
  attitude-gain errors only; the +1-tick delay and ±10 % rate-gain errors are enforced at T3. Luis: evaluate DShot
  error diffusion at L6 (it changes L3 output: its own decision record).
- **The chirp at roll/pitch** carries about 2.5° of measurement uncertainty from that limit cycle (plateau rule,
  decision 20); yaw is clean (0.085°).
- **T3 reference size**: golden + envelope 8.6 MB (about 2.6 MB compressed in git) at 3.2 kHz — Luis's item 1.
- **Regeneration at L6**: gains, T3 fixture (`--refresh-inputs`, `--refresh-q-inputs`), golden/envelope/Q, step and
  yaw scenario execution counts, and the chirp scenarios (`uv run python tools/sim/gen_l5_chirp.py`) all regenerate by
  rule; tests assert rule properties and fail loudly when a committed value is stale.

## Carried forward (not L6 unless the spec or Luis says so)

- **From L5 (0006 "Carried forward").** L8: yaw deadband (0 today) with real sticks; the guard that only test
  compositions may carry `TRUTH_STATE`. L9: re-check the attitude rate against EMB-3 with measured WCET (decision 21).
  `run_l4` passes `rotor_speed_rad_s` through but `l4_scenario` does not accept it yet (0007). The recovery envelope's
  T3 rounding term is borrowed (INFERRED, about 1e-5 against a 4.1e-2 halving term). The runner fills the L2
  separatrix field with the state's own value for recovery scenarios (the analytic check does not apply there). The
  plugin applies initial rates one host step late (0003 item 11); the recovery test skips rate channels at execution 0.
- **From L4 (0005, handoff of quad-L4-pass).** QF-8 open until motor τ σ exists; D term, its filter rule and the
  crossover cap move to L6; the replay fidelity gate is DShot-quantised; the composition's extended parameter set
  relies on link order (a flight composition uses the product set only, L9); the step predicate cannot resolve an
  envelope shift below about 15 ms.
- **Snapshot-style frozen tests in L00–L03** (Luis asked for the list; nothing changed): L01 `test_flatten_report.py`
  (:31, :134-160, :206, :218, :238, :455, :214, :389, :393, :196); L01 `test_card_lint.py:482`; L02
  `test_run_scenario.py:358, 385-395`; L03 `test_mixer_params.py` (:164, :267, :34-36, :77, :241, :256); L00
  `test_lint_g1.py:98-104`. Not `test_gen_sdf_plant_config.py` (kept pinned by decision, 0007).
- **L3, SIM-3, CI pinning, B1, L9, QF-4, spec gaps**: unchanged from the quad-L4-pass handoff; see 0005 "Spec gaps
  logged" and 0006 "Spec gaps logged". New L5 spec gaps: the attitude chirp joins the L5 pass bar (decision 16);
  angle-mode yaw bypasses QF-2's reference model; 0006 F's first chirp amplitude rule was superseded (decision 20).
- **Known, accepted gate limits.** G1 and G3 do not cover `sim/gz` (review only). `ci/run_ci.sh` does not run the L02
  pytest suites; `run_ci_gz.sh` does.

## Working notes

- **Numeric literals.** Every literal under `fw/` other than 0, 1, 2 and ½ fails CI. Cited constants go in
  `constants.hpp` with `Citation:` and `Kind:`. Test numbers are derived or labelled `scenario test value` with a
  reason.
- **New product-set parameters.** Add the layer's `tests/regression/quad/Lnn/param_ids` manifest (0004).
- **Goldens.** Regenerated only inside the image, with the command recorded next to them; CI reproduces them byte
  for byte with a perturbed-input control.
- **Gazebo tests.** One scenario per gz process (`run_gz_process`), `gz sim --force-version 8`; a skipped pytest in
  the gz image is a failure. Protobuf "already exists" lines are noise.
- **Parallel workers.** Worktrees start from `master`; fast-forward them to the working branch first. Size packets to
  finish inside a worker's turn budget: the large T3 and Gazebo packets overran at L5.
- **Replays.** L4 (`tests/regression/quad/L04/replay/`) and L5 (`results/{step,recovery}_cause/`) re-execute the
  firmware on logged samples and must match DShot bit for bit; use them to recover internals the log does not carry.
- **Scope.** Keep changes to the layer being executed. Prefer fixing harness or setup limitations over adding
  tolerance terms, and after any new term re-confirm the fine negative controls (0006 decision 24).
