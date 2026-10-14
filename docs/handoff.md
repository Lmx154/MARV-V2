# Handoff: quad L6 stage (e) in progress (0019); L11 S1–S5 built, S6+ paused (0020); stopped 2026-10-14 01:30

2026-10-05. Written for the next tech lead, from git and the records, not session memory. This file states where things
are and adds no scope. The ACTIVE specs and decisions 0009–0019 are the authority. Update this file in the same change as
every decision record and every stage close (Luis, 2026-09-30).

## Read first
- `CLAUDE.md`: the ACTIVE-spec rule, the number rule, the UNKNOWN rule (stop and ask), the frozen-test rule, gates
  G1–G8. "Verified" means `ci/local_ci.sh` passed on the exact commit.
- `docs/spec/10-quad-flight-software.md` §4 "L6": the stages (a)–(e) and the pass bar. The (e) noise-term line was
  replaced with Luis's approved wording (0019 item 5).
- `docs/decisions/0019-l6-stage-e-noise-monte-carlo-and-gate.md` in full: Luis's rulings for (e), quoted verbatim in two
  rounds, and items 1–8 as built.
- Background, as needed:
  - 0012: stage (a) noise model, corners, question E;
  - 0013: stage (b) gyro chain;
  - 0014: stage (c) PI × lead with D, the FF (inert);
  - 0015: R2 starts from a steady tumble;
  - 0016: gz first read;
  - 0017: stage (d) DShot diffuser, live;
  - 0009: the strict-xfail gate file.

## Test-suite redirect (Luis, 2026-10-14), the current focus; branch `test-blast-radius` from `quad-l11`
Verbatim: "I need you to start creating new rules for testing. We have created testing hell in this repo. adding basic
functionality and running the entire test suite is not a sustainable way of developing anything. I need you to split up
the test suite such that we have one unified test that is fast instead of testing literally everything. one test that
is reliant on everything working else it fails. leave the test suite robust but only even run the entire thing if we
are sure that the blast radius of the changes made affected everything, else only run tests in locations that were
possibly affected. Also, in the creation of new tests be extremely careful. do not create tests for every single
feature anymore this is an absolute waste of time. things that are simple can and should be tested but the act of
continuing the entire repo without updating the tests and letting them accumulate to some monster test suite without
trimming it or consolidating it should not be allowed at all. I need this to be your focus, trim the testing suite.
make blast radius based testing instead of just one massive run at the end of everything, because this is making
development impossible. It would be palatable if it was just slowing things down, but it has become a problem."
This supersedes the per-push half of the CI consolidation proposal below; L11 S6+ stays paused.
Luis, 2026-10-14, follow-up verbatim: "keep the full runs as a safety net and as a CI that is only ever run when we are
finally going to merge to master". So: development pushes and local checks run the smoke test plus the blast-radius
selection; the full suite runs only on the merge into `master` (pull request into `master` or push to `master`).
- Built (uncommitted on `test-blast-radius` until Luis says commit): core §7.6/§7.7, CLAUDE.md, decision 0021;
  `ci/smoke.sh` (the L11 composition through the SIL ABI, 37 entries, 0.4–0.9 s warm); `ci/affected.py` +
  `ci/blast_radius.yaml` (fail-safe: an unmapped path or a root selects everything; Python per file by import closure;
  `--check-manifest` cross-checks the map against compile_commands.json, Python imports and test file reads);
  `ci/dev_check.sh` (smoke, map check, selected core steps, selected gz groups, in the working tree's own build trees);
  run_ci.sh hooks `MARV_CI_ONLY`/`MARV_CTEST_LABELS`/`MARV_PYTEST_PATHS`, ctest in parallel, the duplicate frozen pass
  removed; every regression test labelled `Lnn`; Actions and `ci/local_ci.sh` full-only, no schedule; `tests/unit`
  (1 + 1 = 2) deleted. Evidence in 0021.
- Development check, warm trees: docs 0.8 s; `fw/vertical` ~22 s + G1; `tools/sim/l6_noise_term.py` ~21 s (was the
  whole L06 tools dir, 293 s). G1 needs `clang-tidy-18` on the host (not installed on Luis's host: `lint_g1.py:217`).
- Not yet run: the full merge gate (`ci/local_ci.sh`) on this branch's head.
- Open for Luis (trims that change what runs; each needs his word and goes into a decision record): (1) the L5 T3
  one-ulp control on one output file, not the whole set (−~4 min whenever L5 is touched and at the merge); (2) the L5
  gz T4 suite (740 s): steps in development checks, chirps/yaw/recovery only in the full run; (3) consolidate the twin
  truth-channel C++ suites (L05 TruthState / L11 TruthVertical), the three scenario-schema pytest files and the three
  param-prefix scripts (about 50 fewer entries, little time); (4) merge-run speed (ccache, one Gazebo tree for the four
  plugin variants, cached references, pytest-xdist: ~70 → ~25 min).

## Stop point (Luis, 2026-10-14 01:2x: "find a spot to stop at quickly and commit. then write in the handoff")
- Branch `quad-l11` (from `recovery/hover-demo`, from `master` = `b48e202`). Commits, oldest first: `c5d7da5` S0 (spec +
  0020), `253b620` S1, `50b5ff6` S2, `21ba942` S3, `e08179e` S4, `63cf5ae` S5 (amended: the report guard), `aeec084`
  S6a-1 parked, then the docs commit that carries this section. Verified: S1, S2, S3 each by a full `ci/local_ci.sh`
  on the exact commit; S4+S5 as one batch — core, gz-l2, gz-l5 and gz-l11 on `534498d` (S5 before the amend) and
  gz-l4 on `63cf5ae` (PASS, 1002 s, 2026-10-14 01:26). S6a-1 is inert (no consumer) and is not CI'd on its own.
- `origin/quad-l11`: pushed through S3 at 01:05; the rest pushed at the end of this session if gz-l4 is green (the
  commit list on origin is the record). `master` untouched. Luis merges `recovery/hover-demo` then `quad-l11`
  (fast-forward chain) when he wants them.
- Working tree clean at the stop. Nothing is running. The demo kit, the L11 demo world and screenshots, the altitude
  packet/survey/decision and the CI timing logs are in `~/marv-handover-2026-10-13/` (the batch CI logs with
  per-step timestamps: `~/marv-handover-2026-10-13/ci/`).
- Local CI: `ci/local_ci.sh [--commit <rev>] [core|gz-l2|gz-l4|gz-l5|gz-l11 ...]`, five jobs, ~75 min of job time; run
  per push batch on the batch head, in parallel lanes (three fit the 16-core host), not per commit.
- To resume L11: S6a-2 (the C++ T3 test against `build/reference/quad/L11/t3/vertical_t3_golden.txt`; notes from the
  engineer who stopped: the hover_thrust_n fixture check needs a bound, not equality — the rule uses the card's double
  values and rounds one ulp off the f32-rebuilt value; thrust_offset_n has no table source to recompute from; L05's
  T3 test does not compare its fixture with the table; `ci/run_ci.sh:257` and decision 0013:401 name the two exempt
  reference steps), then S6b (envelopes, Q, the coupled-plant shove), S7 (scenarios, `run_l11.py`, the T4 gz suite
  in the gz-l11 job), S8 (close). All of it is test work, which Luis stopped; it resumes only on his word.
- Open for Luis: the CI consolidation Tier 2 rulings (section "CI consolidation"); the L11 pass bar (the spec's T3/T4
  items are unbuilt while S6/S7 are paused: the layer cannot be declared passed or frozen); the R2 package and C6a
  (unchanged, see below); the gz 10 packages that shadow Harmonic's render engine (the demo uses an LD_PRELOAD shim).

## Recovery inspection (2026-10-13)
Luis's brief, verbatim: "The project is currently unusable. Your objective is to restore the smallest possible working
demonstration: a simulated quadrotor achieving stable hover." Finding: not the case at `b48e202`. On Luis's host the
documented host-debug build and ctest pass (661 of 661), the `host-gz` and `host-gz-l5` plugins build from HEAD, the
frozen L2 hover check passes (`test_analytic.py -k hover`: 13 passed, 0 skipped) and the frozen L5 attitude checks pass
(`test_truth_plumbing.py` and `test_t4_steps.py`: 21 passed, 0 skipped); Actions is green on `b48e202` for its push run
and for every nightly full run from 2026-10-05 to 2026-10-13. The hover demonstration is the existing L2 hover scenario
and the L5 level hold at m·g (CLAUDE.md, Commands); the ACTIVE spine has no altitude or position loop, so nothing else
is a hover, and no new test was added (Luis, 2026-10-13, chose to rely on the frozen tests). What was wrong was local
only: `.venv` had been created by a CI container run against the repository directory (INFERRED from its
`uv = 0.12.20`), so it was root-owned and its entry points carried the container's path (`#!/src/.venv/bin/python`):
`uv run pytest` could not spawn, while `.venv/bin/python -m pytest` ran. Fixed: Luis ran `chown`, then
`rm -rf .venv && uv sync --frozen`. The three plugin builds predated `5fc37b6` and `d361d54` (rebuilt), and C6a sat
uncommitted (now on `l6-c6a-wip`). No code changed. What remains broken is unchanged: the known failing items under
"Open items".
- Luis asked to see the hover in the Gazebo GUI. Two things hid it, neither in the flight code: (1) gz-sim 8's GUI opens
  `/usr/lib/x86_64-linux-gnu/libgz-rendering-ogre2.so`, which the gz 10 packages (libgz-rendering10-ogre2, installed
  2026-10-08) own, so the 3D view died with "Found no render engine plugins in [gz-rendering-ogre2]"; headless runs never
  render, so CI and the tests were untouched; (2) the generated worlds are headless by design: no `<visual>` and only the
  Physics system listed, so the GUI gets no scene. A demo kit outside the repository (`~/marv-handover-2026-10-13/demo/`)
  adds visuals (the Fuel "X3 UAV" meshes at the card's rotor positions, a stand-in until Q-D1's Chameleon mesh), the
  SceneBroadcaster, RTF 1 and an LD_PRELOAD shim for the render engine. Luis (2026-10-13): no `--view` option in the
  repository for now; the X3 stand-in is fine. The permanent GUI fix is his: remove the gz 10 rendering packages or keep
  the shim.
- What the demo showed, measured from the plugin's logs and the live pose topic: L5 holds attitude as specified (a 0.2 s
  roll shove of 0.02 N·m tilts it 8–9° and it is level again in 0.7–0.95 s); nothing holds altitude: each shove leaves
  about 0.1 m/s of descent for good (no drag at v0), and the fixed-thrust hover is an unstable equilibrium in height
  because the plant evaluates g at the current height (marv_plant.h:83–84) while `l5_thrust_n` is m·g(h0): measured
  +0.63 mm/s² at z = +205 m = the free-air gradient 3.086e-6 s⁻² × 205 m, time constant 569 s. The recover_inverted
  scenario acquires 6.9 m/s of descent during the 0.7 s flip and keeps it. Luis chose to scope an altitude loop; see
  the next section.

## L11, the vertical loop on truth state: opened (decision 0020, 2026-10-13)
- Luis ruled on the architect's proposal (0020, his eight answers verbatim): a new layer L11, truth-fed and SIL-only,
  built in parallel with L6 stage (e); the spec text as proposed; `thrust_gain_robustness_band` = 0.5 (his labelled
  choice); no climb/descent clamp; the T4 roll shove is 0.02 N·m for 0.2 s (labelled scenario value); the spec's
  "T/W ≈ 4.5" is marked UNVERIFIED against the card's 6.47.
- Stages, one commit each with full local CI (0020, "The proposal as ruled", section 7): S0 spec + record (this commit,
  docs-only); S1 truth channel (`marv_truth_vertical`, TRUTH_VERTICAL library option, G3 rule and controls, T1); S2
  `fw/vertical` law; S3 `tools/card/vertical.py` and the budget entry with its `L11/param_ids` claim (changes the frozen
  L06 product_params table: the record names it); S4 composition `l11_vertical_scripted`; S5 plugin element, adapter,
  log type 8, the L5 byte-identity check; S6 T3 oracle, golden, envelopes (0011 storage); S7 scenarios, runner, T4 gz
  tests and controls, a `gz-l11` job; S8 close. Estimate 11.5 working days (not a commitment). S1 landed (full local CI green on 253b620). S2 landed (reviewed PASS). S3 landed (reviewed PASS: the generator,
  the budget entry, the design site in design/scenario_values.yaml, the L11 manifest, the regenerated product table).
  S4 landed (reviewed PASS: the composition, its register, SIL library and 37 T1 tests). S5 landed (reviewed PASS: the plugin element, adapter, log record 8, the host-gz-l11 preset, the l11 CI group and
  gz-l11 job, the I1 guard). Next: S6 (S6a-1, the vertical T3 oracle and golden, in progress).
- Coupling: stage (e)'s FF switch-on commit must also keep L11 green (INFERRED low risk). The R2 collective question is
  untouched by L11 (no recovery scenario).
- The packet, the survey and the architect's full decision: `~/marv-handover-2026-10-13/altitude/`.
- **2026-10-14, 00:28, Luis, verbatim:** "push whatever is done and verified and validated. it is insane that the
  tests take so long. i need you to start consolidating the test suite to run only tests that test unified
  functionality and do it fast. we can't possibly be stuck in testing hell all this time we need to start developing
  and finxing things NOW. STOP testing stop creating tests and start consolidating the suite. do that or just manage
  the testing suite in a much more efficient manner because this is unacceptable we will never get anything done at
  this rate,"
  - Done on that: S1–S3 pushed to origin/quad-l11 after their full local CI; S4 and S5 verified as one batch (three
    parallel lanes) and pushed; the per-commit full-CI loop is dropped — one `ci/local_ci.sh` run per push batch, on
    the batch head, only the jobs a change can touch, in parallel lanes. S6/S7 test work stopped (above).
  - The loop as built flies: the L11 composition on the host-gz-l11 plugin in the Gazebo demo world
    (`~/marv-handover-2026-10-13/demo/demo_altitude_hold_L11.sdf`): undisturbed 19 s at z = 0.00 mm, tilt 0.00°; each of
    six 0.02 N·m / 0.2 s roll shoves tilts it 8–9°, is levelled in 0.7–1.0 s, costs 2–4 mm of height and is back at the
    reference within 3.5 s; after all six z = +0.0 mm, −0.01 mm/s. The L5 hover lost 59 mm per shove for good.
  - The CI consolidation proposal, measured on the batch run, is the next section; Tier 1 needs no test change.

## CI consolidation (owner request 2026-10-14): where the time goes, and the proposal

Measured on the S4+S5 batch (commit 534498d), three lanes in parallel on the 16-core host, the containers at the
Actions limits (4 CPUs each), per-step timestamps. Solo runs are about 20 % faster.

## Where a full run goes (five jobs, ~75 min of job time; Actions wall = the longest job)
| Job | Step | s | What it is |
| --- | --- | --- | --- |
| core | host-debug: configure, build, ctest | 472 | full Debug build (294 targets, -Werror) + 745 ctest entries |
| core | host-release: configure, build, ctest | 460 | the same again in Release |
| core | tools tests (71 pytest files) | 303 | per-push tools tests; 6 slow files already nightly-only |
| core | L5 T3 oracle regeneration | 268 | regenerated every run (decision 0011) |
| core | frozen suites (ctest -L frozen) | 36 | the 744 frozen entries run a second time |
| core | m33: configure, build | 322 | the target build from scratch |
| core | L5 T3 one-ulp negative control | 309 | regenerates the whole L5 set again with a perturbed input |
| core | the other 31 controls + every gate check | ~60 | planted manifests/targets, nm, token scans |
| gz-l2 | gz_build | 423 | the plugin tree from scratch |
| gz-l2 | gz_sensors + gz_analytic + smoke/tools | ~150 | the tests |
| gz-l4 | gz_build_l4 | 410 | the plugin tree from scratch, again (different SIL library) |
| gz-l4 | gz_l4 | 116 | the tests |
| gz-l5 | gz_build_l5 | 409 | the plugin tree from scratch, again |
| gz-l5 | gz_reference_l5 | 290 | the L5 T3 reference regenerated a second time |
| gz-l5 | gz_l5 | 606 | the L5 T4 suite (steps, chirps, yaw, recovery) |
| gz-l11 | build l5 + build l11 + 13 tests | ~900 | two plugin trees from scratch, then the L11 gz tests |

Per full run (job time, contended): builds ≈ 2,900 s (Debug, Release, m33, four or five plugin trees); the L5 T3
reference ≈ 870 s (regenerated in core, regenerated again by its control, regenerated again in gz-l5); tests
proper ≈ 1,350 s (745 ctest entries 2×, 71 pytest files, the gz suites); everything else ≈ 100 s.

## Tier 1 — no test changes, no decision record (CI scripts only); expected full run 70 → ~25 min wall, local batch ~15 min
1. ccache in both images with a persistent cache (local: a volume under ~/.cache; Actions: actions/cache keyed on the
   compiler and the lockfiles). Every from-scratch build above becomes 30–90 s when the change is small.
2. One Gazebo job instead of four: the plugin's own objects are identical across l2/l4/l5/l11; only the linked SIL
   library differs. Build all four plugin variants in one tree (MARV_GZ_SIL becomes a per-target list) and run the four
   test groups sequentially in one job: 4 × 7 min of builds → one build; the L5 reference regenerated once.
3. Cache build/reference across runs (refdata's stamp already skips regeneration when the inputs are unchanged): the
   two 270–290 s regenerations drop to 0 s unless an input changed. The one-ulp control (309 s) stays per push unless
   Luis rules it nightly (Tier 2, item 9).
4. pytest-xdist for the tools tests (4 workers): 303 s → ~100 s; the per-file time guard keeps its meaning (it measures
   alone time).
5. Drop the duplicated `frozen suites` ctest pass (36 s): host-debug's ctest already ran every frozen entry.

## Tier 2 — test-semantics changes, needs Luis's ruling and a decision record (core 7.2, 7.3)
6. host-release: build only per push (catches -O2 compile differences), run its ctest nightly: −7 min per push.
7. The L5 T4 gz suite (606 s): keep the step tests per push, move chirps/yaw/recovery to nightly; same for L4's
   chirps. Per-push gz time ≈ 4 min of tests instead of 15. Nightly keeps the full set (MARV_CI_MODE=full exists).
8. Negative controls: keep every control, but run the ones that only exercise checker scripts as pytest cases of the
   checker (several already have both forms) — only the controls that need a planted build stay as CI steps.

9. The L5 T3 one-ulp control (309 s per push): run it nightly, or keep it per push but on one output file (the
   SHA check fails on the first mismatching file; generating all three is not needed to prove it).

## What this does not change
- No frozen test is loosened or deleted; the full set still runs nightly and before every stage close and tag.
- "Verified" stays `ci/local_ci.sh` on the pushed head — once per push batch, not once per commit.

## Git state (2026-10-05)
- `master` = `origin/master` = `b48e202` (C5). Everything committed on master is pushed, each commit after a full local
  CI pass on that exact commit.
- Stage (e) commits on master, in order:
  - `dd003e4`: 0019 opens the stage; `r2_ff_diagnosis/regenerate.sh` pinned to `4a76755`;
  - `5fc37b6`, `d361d54`, `dbaa70e`: W1, the plugin's IMU model, rotor-speed telemetry and clock corners, the sensor
    replay, the L4/L5 driver passthrough, and the accel-tie test;
  - `c5af152`: `noise_term_confidence` = 0.99 and the spec line;
  - `7033721`: the S9 pytest guard;
  - `0b8cdb7`: C4, `tools/sim/l6_noise_term.py`;
  - `b48e202`: C5, `tools/sim/l6_bias_delta.py` plus the hooks in `l6_ff_eval.py`.
- Local branches:
  - `recovery/hover-demo` (2026-10-13, from `b48e202`): the recovery inspection's docs-only commits (this file and
    the CLAUDE.md Commands section). Luis merges it.
  - `l6-c6a-wip` (`14e3c3a`, from `b48e202`): C6a as found in the working tree on 2026-10-13, see "C6a" below.
  - `l6-0018-c1` (`7ea4d29`, never pushed): the held 0018, c1 back-calculation anti-windup. Kept until Luis rules on the
    R2 package. It loses yaw trim under sustained roll/pitch saturation, and its L5 rate-bypass golden is unresolved.
  - `quad-l6-spec` (`3158fb9`, checked out in a stale worktree under an old session's /tmp scratchpad): an old CI-limits
    commit, already on master's history. Leave it.
  - `quad-l5` (`e9bfaae`): merged L5 history. `l6-t3-storage-state` (`602e9ed`, its remote is gone): it carries the
    datasheets; Luis deletes it.
- Stash: only `stash@{0}` (an old quad-l0 WIP). Not ours, don't touch it.
- `datasheets/`: present and gitignored (7 PDFs).

## C6a, stopped part-way (Luis, 2026-10-05); parked on `l6-c6a-wip` (`14e3c3a`, 2026-10-13). Not reviewed, not CI'd.
C6a was the `quad/L06/t4e` reference set: the T4e envelopes plus the noise σ per point and its max, per (e) scenario,
stored per 0011.
- `tools/sim/l6_t4e_model.py` (new): the T4e linear model, including the L5 model with the attitude Jacobian frozen at
  kp and 0.
- `tests/regression/quad/L06/t4e/reference/` (new): `t4e_reference.py` (the generator), `t4e_inputs.txt`, `SHA256SUMS`.
  `__pycache__/` there is junk.
- `tests/regression/quad/L06/tools/test_l6_t4e_model.py` and `test_t4e_refdata.py` (new): the model checks, and the per-push
  regeneration of one file against its SHA256SUMS.
- `tools/refdata/refdata.py` (modified): adds the `shared`/`depends` inputs for the new set.
- `ci/run_ci.sh` (modified): the full set (about 2 h at 2 processes) in full mode only.
- The `refdata ensure quad/L06/t4e --procs 2` run was killed after 47 min. Whether SHA256SUMS is complete or correct is
  UNKNOWN.
- The next lead decides: finish C6a from these files, or discard them and recut it. Its packet: 0019 rulings plus the
  memo `~/marv-e-round-2026-10-04/e_noise/design.md` §3 and §5.

## Saved outside the repo (evidence and drafts, not committed)
- `~/marv-e-round-2026-10-04/`:
  - `e_round/`: the architect's (e) decision round;
  - `e_w1/`: the W1 design memo;
  - `e_noise/design.md`: the noise suite memo, with the C4–C7 plan;
  - `close_notes.md`;
  - `r2yaw2.tar.gz`: the R2 model that reproduces gz to 0.010 rad/s;
  - `r2witness.tar.gz`: the R2 in-range witness, reviewed PASS;
  - `r2study.tar.gz`: the residual study, candidates (i)–(iii);
  - `awvar.tar.gz`: the anti-windup variants V1–V3 and the trim analysis;
  - `c1.tar.gz` (3.6 GB, raw gz logs): the c1 design. Trim it once its evidence is committed;
  - `acronoise.tar.gz`: acro's margin under noise.
- `~/marv-r2yaw-2026-10-04/r2yaw.tar.gz`: the first R2 yaw analysis.
- `~/marv-handover-2026-10-03/`: the stage (c) handover copy.
- `~/marv-handover-2026-10-05/`: this handoff, the git state, the C6a diff and a tar of the untracked files.

## Open items, all Luis's unless marked
- **R2 package** (sent 2026-10-05; he asked for the nine-heading shape; not yet ruled). Five questions:
  1. anti-windup: freeze (recommended) / V2 / c1;
  2. may the allocator raise the collective in a recovery? The witness needs ΣT up to 17.2 N; today the thrust request is
     m·g (0015);
  3. candidate (i), the rotor-lag-inverting allocation, as the study's base (a spec change);
  4. commit the witness, 0011 style;
  5. keep or delete `l6-0018-c1`.
  - R2 fails under every realisable law: the floor is 1072 violations, with FF on and (i) and no anti-windup.
- **W per point or constant** (asked; not ruled). On the card member, acro's σ at the binding execution is 10.3× smaller
  than the run's max. The constant W reaches 34.4 mrad/s on a 3×3 grid, against acro's T4e margin of 25.1 mrad/s. The
  per-point W is about 1.9 mrad/s. Lead recommendation: per point.
- **The gz re-measure** (ruling 7): only when Luis says the host is quiet (load < 1.5). It fixes the shard count, and C7
  waits for it.
- **L11 (lead):** paused after S5 by Luis (2026-10-14); S6a-1 parked uncommitted; S6a-2, S6b, S7, S8 not started.
  Resume only on his word. Local CI has five jobs (gz-l11 added).
- **Remaining (e) builds:**
  - C6a: see above;
  - C6b: the T3 corner scan (27-row table), the L5 and chirp plan adapters;
  - C6c: the nightly σ̂ χ² check with the new register entry `noise_sigma_bound_confidence` = 0.99 and a planted control;
  - C7: the noise suite on the T4e envelopes, in new files, with the decision-24 fine-control guard.
- **Known failing items** (strict xfail while `tests/regression/quad/L06/XFAIL_GATE_CLOSED` is absent):
  - L4 acro: FF on passes by 5.72 mrad/s at T4c; under noise and bias it fails at T4c; T4e is about +17 at the worst
    corner (INFERRED);
  - L5 R2: 2431 violations with FF on;
  - L5 R1X α: passes with FF on.
- **The FF switch-on and the gate:** one atomic commit sets `rate_ff_enable` 0 → 1, creates the gate file and adds a
  per-push FF-iff-gate check. It lands last, only when every item passes in both the frozen T4c tests and the T4e noise
  suite. If any fails: no gate, no per-item gates, the record goes to Luis.
- **Still owed to Luis:**
  - the CI split with measured times;
  - the T4 seed and corner counts with their cost. Estimated so far: tracking 22 and safety 59 seeds, about 3.1 h serial
    gz time for full confirmation, before the wiring re-measure.

## Rules and rulings the next lead must not break
- Never loosen a frozen test or a product check. Every change under `tests/regression/` needs a decision record in the
  same change, naming each full path (`tools/ci/check_regression_changes.py`).
- Records whose tests reproduce them byte for byte are regenerated by their own generator. If any data line changes,
  stop.
- Code or test commits: `ci/local_ci.sh` on the exact commit, then push. Docs-only commits: the regression-change check
  is enough. If anything is red after a push, fix forward; never rewrite pushed history. No tags without Luis.
- One writer to the repo at a time. Numbers: cited, sourced, derived with their rule, or labelled; UNKNOWN means stop
  and ask.
- Reports to Luis:
  - built from the records and git;
  - results first, numbers with units;
  - quote in full anything he must approve;
  - every question answerable in one word, with a recommendation.
- (e) rulings, 0019 verbatim:
  - NT-2, Bonferroni inside the run, with the point count taken from the predicate;
  - predicates judge the gz truth state;
  - the bias delta comes from the nonlinear model; the coupling never enters the envelope;
  - per push, PP-A: seed 1 at the nominal corner; nightly and before every stage close and tag, 22/59 seeds ×
    {nominal, worst corner};
  - clock corners inside the T3 search, and the Gazebo step rounded outward (0012 question E);
  - exact ties only, each with a per-push bit-identity test;
  - frozen extremes for the time-varying scenarios, checked by the nightly σ̂ check;
  - the decision-24 fine controls must still fail, else stop;
  - S9 for pytest is enforced under local CI and only reported on Actions. Watch `test_r2_lower_bound.py`: 57.4–58.3 s
    alone against 60 s.
- Actions times are noisy (runner speed varies about 2×). Measure S9 and timings in the local CI image at load < 1.5.
- Luis's machine is shared: his games load the CPU. Check `uptime` before any timing.
