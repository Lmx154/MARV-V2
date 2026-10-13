# Handoff: quad L6, stage (e) in progress (decision 0019); stages (a)–(d) closed

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
