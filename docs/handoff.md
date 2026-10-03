# Handoff: quad L6 stage (c) opened (decision 0014); stages (a) and (b) closed

2026-10-01. Written for the next agent working in this repository. This file records state; it adds no scope. The
specs remain the only planning documents (core §0 rule 5).

**Update rule (Luis, 2026-09-30):** "From here on, update it at every stage close and every decision, not only at
layer end." Rewrite this file in the same change as every decision record and every L6 stage close.

## Read first, in this order

1. `CLAUDE.md`: the rules CI enforces (ACTIVE-spec rule, number rule, UNKNOWN rule, gates G1–G8), commands,
   conventions. **Verified means `ci/local_ci.sh`** (fresh clone, the Actions containers, as root, under the measured
   runner limits).
2. `docs/spec/00-core-contracts.md` (ACTIVE): §2 numbers and provenance, §3 conventions, §4 firmware boundary, §5
   sensor profiles, §7 testing, freezing, and §7.5 (convergence rule, flight rate groups).
3. `docs/spec/10-quad-flight-software.md` (ACTIVE): §4 the spine. **§4 "L6 — Sensor models, the gyro chain and the D
   term"** is the current layer: Builds, stages (a)–(e) and the pass bar, all approved by Luis (0009). §6.2 QF-3 now
   includes `Ms_max`; QF-7 names the notch harmonics.
4. `docs/spec/20-ground-segment.md` and `30-rocket.md` are PARKED: read them for context, never implement from them.
5. `docs/decisions/0001`–`0012`. **0012** is L6 stage (a): every owner ruling of 2026-10-01 verbatim (datasheet, crystal, clock model, offsets, A–D, licence, Bonferroni), the build P1–P7 and its evidence. **0011** moves the T3 references to generator + inputs + `SHA256SUMS`. **0009 opens L6**: Luis's L6 decisions verbatim, D1–D7 (D term), F1–F3 (ω×Jω), the
   new register entries and the gate file. 0010 is the L5 chirp-cache memory fix (frozen edit, approved). 0005 and
   0006 hold the L4 and L5 choices: read them whole before touching the rate loop, the attitude loop, their gains or
   any L04/L05 test.

Luis (the owner) makes the final calls. Numbers you cannot source are tagged `UNKNOWN` and you stop to ask.

## Luis's instructions (verbatim)

2026-09-30:

> Order: T3 storage migration first (with its decision record), then stage (a),
> so no L6 reference data ever gets committed.
>
> Before either: rewrite docs/handoff.md to match master now. From here on,
> update it at every stage close and every decision, not only at layer end.

2026-10-01, on stage (a) (the S1/S3/S4/S9 answers of 2026-09-30 are superseded where these differ):

> 1. **Datasheet.** Cite DS-000577 rev 1.0. Both ACTIVE specs still cite DS-000489, in the core §5 table and the quad
>    §5.5 FIFO packet line. Log this as a spec gap in the stage (a) record. Check core §5's ICM-45686 figures and the
>    20-byte FIFO packet against DS-000577, then send me the two-line spec fix to approve. If any figure differs, keep
>    both values and mark them UNVERIFIED.
> 2. **Crystal.** Use the worst-case linear sum, ±65 ppm: ±30 tolerance, ±30 stability and ±5 first-year aging, from
>    the ABM8-272-T3 spec. Don't use RSS. Datasheet limits are bounds, not σ, and no source says the terms are
>    independent. Record that the aging term covers the first year only, and that these are the Pico 2 test setup's
>    values until the MARV V2 BOM crystal is known.
> 3. **Clock model.** Yes, apply the error only against the plant's clock. With CLKIN, the IMU's ODR and hal_time_us
>    share one error e, so draw e once per run, never separately for each. Keep the ±1.25 % / ±1 % internal-oscillator
>    figures (DS-000577 §3.3.2) in the profile as the non-CLKIN case, but don't fly them. "The ODR follows CLKIN
>    exactly" stays INFERRED until a committed test or measurement shows it. Exercise e at the corners −65, 0 and +65
>    ppm, not as a sampled distribution.
> 4. **S2 and S5–S8.** That list is lost, so don't reconstruct it. Check stage (a)'s pass-bar lines against core §5's
>    profile fields, and send me only the questions stage (a) is actually blocked on, each one answerable in one word.

Still standing from 2026-09-30: S3, "labelled scenario values, each with a written rationale, flagged in every run
report; replaced by my bench dataset later"; S4, the IMU is clocked from the MCU via CLKIN on GPIO18 (EMB-2, single
master clock); S9, "60 s is fine as the per-push threshold; record it as a labelled convenience value" (a check longer
than that runs nightly, 0009 second message item 4). The datasheet PDFs stay local in `datasheets/` (gitignored); cite
them by document number and revision.

Sources for stage (a):
- **ICM-45686:** DS-000577 rev 1.0 (07/25/2024).
  - CLKIN accepts 20–40 kHz (§4.14).
  - Internal clock: ±1.25 % initial and ±1 % over temperature with the gyro active (§3.3.2).
- **Pico 2:** RP-008299-DS, release 5. It names the crystal, Abracon ABM8-272-T3, but gives no ppm.
- **ABM8-272-T3:** Abracon Drawing #456603, rev IR, 2023-11-16.
  - 12.000 MHz; ±30 ppm tolerance at +25 °C.
  - ±30 ppm stability over −40…+85 °C.
  - ±5 ppm aging, first year, 25 ± 3 °C.

## Next steps, in Luis's order

1. **Done, stage (b)'s first item (0013):** the per-push limit is enforced as the CTest TIMEOUT of every per-push check,
   read from `per_push_check_time_max`; the reference-set steps are exempt. Two CI steps check it: a positive check
   and a planted-overrun control. The slowest check is the Allan test, 10.0 s in debug (6× under the limit). Stage (a)
   is closed (0012), and its tests under `tests/regression/quad/L06/` are frozen.
2. **Stage (b), the gyro chain: closed and pushed (`1159d5c`, decision 0013).** Stage (c) starts from a worst-corner
   PM of 35.5° at today's L4 gains with the chain in the loop.
3. **Stage (c), the D term and ω×Jω (decision 0014).** Luis's eight decisions are in 0014.
   - PI × lead meets PM 45° and Ms 2 with the chain; N* is set by Ms at the flown ESC.
   - The FF is lag-compensated, built inert, and goes live at (e) with the gate file. FF-on evidence is reported in (c).
   - J corners are the physical 3-D ones.
   - R2's setup change (steady-tumble rotor speeds) gets its own record, after the lower-bound proof is committed and
     reviewed.
   - The spec lines (the FF Builds bullet, the combined D + FF noise line) are approved in Luis's wording.
   - Built: commits 1–2 (`539855e`, `69c62f2`, local) and commit 3 (the atomic switch, uncommitted). Every frozen
     file commit 3 changes is listed in 0014, "Frozen files changed by stage (c)".
   - **FF-on report runs (0014, W4; `tests/regression/quad/L06/results/ff_on/`):** acro FAIL → PASS (roll margin
     6.07 mrad/s, a knife edge); R1X FAIL → PASS; **R2 FAIL → FAIL** (13923 → 10398 violations, worst w_y +1.975,
     DShot at both range ends). Diagnosed (0014, W4): mainly the gz step-0 zero-rate read, kicked by D and FF; then yaw
     saturation with the integrator freeze.
   - **Close round (0014, fourth round, 2026-10-03).** Luis's order:
     1. answer three sent-back lead rulings (the chain divisor check, the chirp reference, the shared notch speed);
     2. S9: share the configuration-set design (B), then move what is still over 60 s to nightly (A), after
        confirming that a per-push check pins the generated gains;
     3. commit 3, with no tag;
     4. core CI on that exact commit;
     5. push on his go-ahead;
     6. 0016, the gz first gyro read returns the starting rates (approved; its own commit);
     7. stage (d) under 0017 (rulings saved for 0017).
     0015 is approved as built, subject to a seven-point checklist confirmed in 0015.
   - **Fifth round (0014, 2026-10-03).** Luis's order:
     1. the divisor check becomes f_c < f_s/(2D) with D ≥ 1, and the frozen `fault_test.cpp` edit is made net-stricter;
     2. three new tests: the attitude Ms over the set, the rate-loop stability certificate, and the per-motor mixes
        (plus a test that rate_lead with the lead off reproduces rate.py's chirp reference);
     3. the generated product parameter table, committed and frozen under `tests/regression/quad/L06/`, compared byte
        for byte on every push;
     4. two output-identical speed-ups, then anything still over 60 s moves to nightly (no design cache);
     5. full local CI, commit 3, core CI, push on his go-ahead;
     6. then 0016 and stage (d).
     **State (2026-10-03):**
     - Items 1–4 are built (0014: "Divisor (b), as built", "Fifth round, items 2–5" and "item 6 (S9)").
     - Full local CI passes on snapshot `b4c9e16`: core, gz-l2, gz-l4, gz-l5. The per-push tools step is 1013 passed and
       1 skipped in 274 s.
     - Luis answered the last open items (0014, sixth round): the chirp check uses option (i); `test_attitude_t3.py`
       goes nightly; the extra divisor frozen line is accepted.

## Handover (2026-10-03): stage (c) commit 3

**Git state.**
- `master` at `69c62f2` (stage (c) commit 2), with `539855e` (commit 1) before it. Neither is pushed;
  `origin/master` is `1159d5c`.
- Commit 3 is entirely uncommitted in the working tree: about 78 modified and 21 untracked paths, among them
  `fw/rate_group/`, `docs/decisions/0015-…`, `tests/regression/quad/L06/{rate_group, product_params, results/ff_on,
  results/r1x_coupling, results/r2_envelope_a}/` and the new L06 tools tests.
- Full local CI passed on snapshot `b4c9e16` (`refs/tmp/stage-c`, a temporary commit-tree, not on any branch).
- The tree since `b4c9e16` differs only in docs: this handoff, 0014's CI-evidence lines, the fifth- and sixth-round
  quotes, and one docstring sentence in `L06/tools/test_notch_mixes.py`.
- Safety copy: `~/marv-handover-2026-10-03/` (`git diff HEAD`, a tar of the untracked files, and the scratch evidence
  for 0016 and stage (d)).
- Local branches `l6-t3-storage-state` (datasheets; Luis deletes it), `quad-l5` and `quad-l6-spec` are untouched.

**Remaining steps, in order:**
1. **Done: the chirp lead-off test** (0014, sixth round, "(i), as built"):
   `tests/regression/quad/L06/tools/test_chirp_lead_off.py`, per push, 2.5 s. Control: the lead at its design values (T_f
   alone has no effect at kd = 0).
2. Commit 3, with no tag. End the message with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
3. Core CI on that exact commit: `MARV_CI_BASE_REF=1159d5c ci/local_ci.sh --commit <rev> core`.
4. Push on Luis's go.
5. 0016: the gz first gyro read returns the initial rates (approved; 0014 W4 has the diagnosis).
6. Stage (d) as 0017 (Luis's rulings are verbatim in 0014, fourth round, item 5).

**Not in any file (the outgoing lead's notes).**
- **Check first:** that `git status` matches the safety copy, and that `datasheets/` is still present and untracked
  (it vanished once on a branch switch; restore with `git restore --source=l6-t3-storage-state --worktree datasheets`).
- **Timing is the fragile part.** `test_r1x_coupling.py` is at 58.9 s alone, with 1.1 s of margin. Host load inflates
  every number (one run read 332 s), so measure with load < 1.5. The code-index daemon can spike to 100 % CPU.
- **Acro FF-on margin is 6.1 mrad/s.** It will be a knife edge at stage (e).
- **0016 will change R2's FF-off numbers** in 0015 and 0014 W4, and the prop-strike report. The diagnosis scripts that
  reproduce gz to 0.016 rad/s are in the safety copy (`r2ff/`: `tool2.cpp`, `b_ladder.py`). After the read fix, R2
  should drop to about 2416 violations, worst w_z +0.24, and still fail. **If R2 passes, stop: it is a strict xfail.**
- **R2's next blocker after 0016** is yaw saturation plus the yaw integrator freeze, which the linear design model
  lacks. Luis expects the numbers for that choice: model the allocation and freeze in the design model, or rule R2
  yaw-limited.
- **The T3 oracles** (`rate_t3_oracle.py:340`, `attitude_t3_oracle.py:1699`) still check the old f_s/2 cutoff bound.
  It is consistent at D = 2, frozen, not changed.
- **Stage (d)'s record number** is 0017. Its architect draft is in the safety copy (`stage_d_draft.md`).
- **Subagent permission denials:** never route around one; show the prompt to Luis.
4. Still owed to Luis from 0009: the final CI split with measured times, and the T4 confirmation seed count with its
   cost. Add to it the T4 turn-on-corner count (nominal plus each scenario's worst T3 corner) and its cost (0012).
5. **Stage (c), carried:**
   - Re-run `L05/results/step_cause/step_cause.py` at N = 1 when D7 regenerates L5, under stage (c)'s record (Luis,
     2026-10-01). Today's `cause.txt` describes att_loop_ratio 2 (H = 10372) from before `0533044`.
   - The per-sample noise convention σ_d = N·√(f_s/2) (0012) feeds the D-path noise budget.

## L6 stage (a) in one paragraph (decision 0012)

The profile drives an opt-in IMU model in `marv_plant`. Each sample is the truth delayed by L samples, plus a bias
random walk from the turn-on corner, plus white noise, quantised to the 20-bit FIFO step and saturated with a flag per
axis. The noise comes from counter-mode SplitMix64 streams with Box–Muller normals; `log`, `sin` and `cos` are vendored
from musl 1.2.5 (MIT, Arm MIT, Sun fdlibm notices; `THIRD_PARTY_NOTICES.md`).
- The clock error e enters once, as the adapter's true tick t_nom/(1 + e). `fw/` never sees it.
- `tools/card/gen_imu_config.py` generates the config from the profile, refusing UNKNOWN.
- The Allan check derives its own record (13.2 M samples) and runs per push in about 4 s.
- There is no generated reference data for stage (a), so nothing went into `refdata`.

## T3 reference storage (decision 0011, approved 2026-10-01)

- **What moved:** five files are no longer committed: `L04/t3/reference/rate_t3_{golden,envelope}.txt` and
  `L05/t3/reference/attitude_t3_{golden,envelope,q}.txt`. Each reference dir holds `SHA256SUMS` instead.
- **How they are made:** `tools/refdata/refdata.py ensure <id>` generates into `build/reference` (or
  `$MARV_REFERENCE_DIR`) and checks every hash. ctest gets them through a fixture setup test, the Python consumers
  through `refdata.reference_dir`.
  - L5 takes about 2 min at `--procs 4` and peaks at 0.26 GiB. The first local ctest or pytest run on a fresh tree
    generates it.
- **Changing a golden:** regenerate in the image, update `SHA256SUMS`, write a decision record.

## Pre-L8 gate and hardware list (Luis, 2026-10-01, decision 0014)

- **Gate:** before L8, measure J with its σ (S0, pendulum), then re-run the L4 acro check over the measured band,
  both with FF on and with FF off; FF flies only if it helps over the measured band (at the physical J corners FF can
  make things worse than PID: 5.14 → 9.46 rad/s on pitch at corner 11). The pilot doesn't fly until it passes there. Today the acro item closes at the card plant only, and its excess at the
  physical J corners is recorded in 0014.
- **Hardware list:** measure J (S0) and the motor τ_m on the bench. That shrinks the band box and raises the achievable
  crossover by rule. The closed-loop time constant of about 0.161 s at the J+ corner is the "sluggish" question again.
- **The hover-rotor tumble** (the collision or prop-strike case, `scenarios/quad/L05/recover_tumble_prop_strike.yaml`,
  decision 0015) is run and reported, and has **no pass bar until L8**. Propose one at L8, from an absolute recovery
  requirement rather than the linear envelope.
- **Finding:** the ESC clock error is the largest single limit on crossover. A crystal ESC would roughly double it.
  This is not a requirement, since the flown ESC would fail it (Luis, decision 7).
- **B3 candidate (INFERRED, an idea only):** estimate each ESC's clock scale in flight from the gyro's harmonic peaks,
  which tightens ε without new hardware.

## Known failing items (safety)

All three have the same cause class and the same gating. **L6 cannot close until all three pass, and all must pass before L8
(pilot in the loop).** Each is a strict pytest xfail (`raises=AssertionError`, strict) whose condition is
"`tests/regression/quad/L06/XFAIL_GATE_CLOSED` does not exist" (0009). Stage (e) creates the file; from then on each is
a normal test and must pass, and an unexpected pass fails CI. Do not re-seed envelopes from wound-up state and do not
tune bounds (0005 decision 12, 0006 decisions 5 and 15).

1. **L4 acro, combined full-stick segment** (0005 decision 12). `tests/regression/quad/L04/gz/test_t4_acro.py`,
   evidence `tests/regression/quad/L04/results/acro_cause/`. The PI baseline (ω_c 8.3 rad/s) cannot reject ω×Jω.
2. **L5 R2, inverted and tumbling at rate_max on all axes** (0006 decision 15). `tests/regression/quad/L05/gz/
   test_t4_recovery.py`, evidence `tests/regression/quad/L05/results/recovery_cause/`. |ω×Jω| at the first execution
   is 41/44/33 % of τ_held. The vehicle recovers, but leaves the envelope by up to 0.80 rad and 5.3 rad/s.

   Since decision 0015 R2 starts from a steady tumble (c* 8.917 N); it is proven infeasible from the old hover start
   (0014, c3 and c6).
3. **L5 R1X, the exact-180° α envelope predicate only** (0014, third round, item 3). It is a strict xfail on the same gate
   file. From the singular start, Gazebo takes a mixed-axis branch whose ω×Jω lag the coupling-free envelope can't
   follow; the tighter stage (c) envelope exposes it. R1X's other assertions stay normal tests.

All three must pass at stage (e), with the lag-compensated FF live (0014). With FF on through the harness (0014, W4),
items 1 and 3 pass and item 2 (R2) still fails (diagnosis in 0014, W4; awaiting Luis).

## Current state

- **Tags.** `quad-L5-pass` on `5ac1cf4` (placed on the first commit green in Actions, per 0009; not re-checked here, `gh` is not installed). Earlier: `quad-L4-pass` `daac5d9`, `quad-L3-pass`
  `dcaf2fb`, `quad-L2-pass` `320cf18`, `quad-L1-pass` `b8ed690`, `quad-L0-pass` `6cca7ec`.
- **Since the L5 handoff** (`33ac37b`):
  - `b966588`: L5 chirp tests keep one axis's runs in memory (0010). `gz_l5` peak 18.33 GiB → 7.37 GiB, same results.
  - `5ac1cf4`: CI split into parallel jobs `core`, `gz-l2`, `gz-l4`, `gz-l5`; nightly cron and `workflow_dispatch` run
    them with `MARV_CI_MODE=full` (where stage (e)'s Monte Carlo will hook in; today both modes run the same steps);
    a PEAK-MEM line per step; `ci/local_ci.sh` runs the same jobs as root under the runner's limits.
  - `7d5a663`: the L6 spec section, the register entries (`Ms_max` 2.0, `d_path_noise_budget` 0.5,
    `t4_pass_probability_{tracking,safety}` 0.90/0.95, `t4_confidence_{tracking,safety}` 0.90/0.95,
    `allan_check_confidence` 0.99), the `L06/param_ids` manifest and the gate-file condition (0009).
  - `3cf3e98` (pushed 2026-10-01, no tag): the T3 reference storage migration (0011) and the handoff. Its gz-l5 job
    generated the L5 reference set itself on a fresh clone and passed.
  - The next commit on `master`: L6 stage (a) (0012) and the two spec fixes it carries.
  - `3158fb9`: `local_ci.sh` limits from the measured runner (4 CPUs, 16765378560 B RAM, 1025118208 B headroom, no
    swap locally), sampler and docker-start guards.
- **L6 progress.** Opened: spec, register and gate (0009). Stage (a) is closed (0012). Stage (b) is built and verified,
  and awaits Luis's close (0013). Stages (c)–(e) are open.
- **Measured wall-clock** (marv-ci-gz, before the split): L02 gz about 1.7 min, `gz_l4` 2 min 21 s, `gz_l5` about
  9 min (548 s before 0010, 539 s after); the L05 T3 oracle about 50 s on the host, 2 min 20 s in marv-ci.
- **Expected results with stage (b).** ctest 610/610 (debug and release); core tools tests 963 passed,
  1 skipped (pre-existing); L04 gz 40 passed, 1 xfailed; L05 gz 63 passed, 1 xfailed.
- **Local leftovers.** Branches `quad-l5`, `quad-l5-p9b`, `worktree-agent-*` are merged into `master`; worktrees under
  `.claude/worktrees/` can be removed.

## How to verify

```
uv sync --frozen
cmake --preset host-debug && cmake --build --preset host-debug && ctest --preset host-debug
cmake --preset m33 && cmake --build --preset m33
ci/local_ci.sh [--commit <rev>] [core|gz-l2|gz-l4|gz-l5 ...]     # the definition of verified
```

Set `MARV_CI_BASE_REF` to the push range base to exercise the regression-change check as Actions does. `local_ci.sh`
tests a fresh clone of a commit, so uncommitted files are not tested. `test_truth_gyro.py` (L04 gz) needs
`build/host-gz` as well as `build/host-gz-l4`, or it skips; a skip in the gz image is a failure.

## Findings that shape L6

- **DShot quantisation at hover** (`L05/results/step_cause/`): hover sits at DShot 765; one step is 4.56 mN per motor,
  a request dead band of ±8.0e-4 / ±6.0e-4 / ±1.8e-4 N·m (roll/pitch/yaw), a ±0.006 rad attitude limit cycle. Q
  absorbs it at T4 (0006 decision 19). T4 angle steps catch gross failures only; delay and gain errors are enforced
  at T3. Stage (d) evaluates DShot error diffusion (it changes L3 output: its own decision record).
- **The roll/pitch chirp** carries about 2.5° of measurement uncertainty from that limit cycle (0006 decision 20); yaw
  is clean (0.085°).
- **Regeneration at L6.** Gains, the T3 fixtures (`--refresh-inputs`, `--refresh-q-inputs`), golden/envelope/Q, step
  and yaw scenario execution counts and the chirp scenarios (`uv run python tools/sim/gen_l5_chirp.py`) all regenerate
  by rule; tests assert rule properties and fail loudly when a committed value is stale.

## Carried forward (not L6 unless the spec or Luis says so)

- **From L5 (0006 "Carried forward").** L8: yaw deadband (0 today) with real sticks; only test compositions may carry
  `TRUTH_STATE`. L9: re-check the attitude rate against EMB-3 with measured WCET (decision 21). `run_l4` passes
  `rotor_speed_rad_s` through but `l4_scenario` does not accept it yet (0007). The recovery envelope's T3 rounding term
  is borrowed (INFERRED). The runner fills the L2 separatrix field with the state's own value for recovery scenarios.
  The plugin applies initial rates one host step late (0003 item 11).
- **From L4 (0005).** QF-8 open until motor τ σ exists; the replay fidelity gate is DShot-quantised; the composition's
  extended parameter set relies on link order (L9); the step predicate cannot resolve an envelope shift below about
  15 ms.
- **Snapshot-style frozen tests in L00–L03** (listed for Luis; unchanged): L01 `test_flatten_report.py`, L01
  `test_card_lint.py:482`, L02 `test_run_scenario.py:358, 385-395`, L03 `test_mixer_params.py`, L00
  `test_lint_g1.py:98-104`. Not `test_gen_sdf_plant_config.py` (pinned by 0007).
- **Spec gaps** are logged in 0005, 0006 and 0009.
- **S9 is unenforced for pytest** (Luis, 2026-10-03). The 60 s per-push limit is enforced only as a CTest TIMEOUT. CI
  runs the tools tests as one pytest session, so no per-file limit applies. Close this at the next CI change. Until
  then, measure per file in the CI image.
- **Known, accepted gate limits.** G1 and G3 do not cover `sim/gz` (review only). `ci/run_ci.sh` does not run the L02
  pytest suites; `run_ci_gz.sh l2` does.

## Working notes

- **Result files (Luis, 2026-10-01, optional going forward).** New result files print a "produced at <commit>" line
  in their header, so a stale hash explains itself without a record. Old files stay as they are.

- **Numeric literals.** Every literal under `fw/` other than 0, 1, 2 and ½ fails CI. Cited constants go in
  `constants.hpp` with `Citation:` and `Kind:`. Test numbers are derived or labelled `scenario test value` with a
  reason.
- **New product-set parameters.** Add them to the layer's `tests/regression/quad/Lnn/param_ids` manifest (0004).
- **Goldens.** Regenerated only inside the image, with the command recorded next to them; CI reproduces them byte for
  byte with a perturbed-input control.
- **Gazebo tests.** One scenario per gz process (`run_gz_process`), `gz sim --force-version 8`. Protobuf "already
  exists" lines are noise. Keep parsed logs out of long-lived caches (0010).
- **Parallel workers.** Worktrees start from `master`; fast-forward them to the working branch first. Size packets to
  finish inside a worker's turn budget.
- **Replays.** L4 (`tests/regression/quad/L04/replay/`) and L5 (`results/{step,recovery}_cause/`) re-execute the
  firmware on logged samples and must match DShot bit for bit.
- **Scope.** Keep changes to the layer and stage being executed. Prefer fixing harness or setup limitations over
  adding tolerance terms, and after any new term re-confirm the fine negative controls (0006 decision 24).
