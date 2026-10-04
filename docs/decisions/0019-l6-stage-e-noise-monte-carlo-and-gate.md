# 0019: quad L6 stage (e), the noise Monte Carlo, the corner search and the gate

Stage (e) of quad spec §4 "L6": the L4 and L5 scenarios rerun with noise and filters, the T3 Monte Carlo over noise
seeds, and the gate file that turns the three known failing items into normal tests with the feed-forward live
(decisions 0009 and 0014). This record opens the stage. It holds Luis's rulings of 2026-10-04 and the stage's commits as
they land. The infrastructure lands first, in its own commits, while R2's path is being decided. The gate commit comes
last, once every item passes.

## Owner decisions (Luis, 2026-10-04, verbatim)

On the (e) decision round (the architect's proposal; the lead's report of 2026-10-04):

"**3. Noise-term rule: NT-2. F1 is a real contradiction in the ACTIVE spec, and this is the fix.**
- At quantile p, a build that is exactly correct passes all N seeds with probability p^N ≈ 1 − c. So the test would reject a correct build 90–95 % of the time.
- With the noise term at the per-seed quantile 1 − (1 − k)/N (Bonferroni over the N seeds), a correct build passes every seed with probability at least k. That is the same family-wise convention as the Allan, independence and moments checks.
- **Guard.** The decision-24 fine controls must still fail with the noise term in place. If one stops failing, stop and report; don't adjust anything.

**4. The register entry: yes, 0.99.** Use a new entry (one entry, one purpose), for example `noise_term_confidence`, claimed in L06/param_ids, with the same rationale as the other family-wise entries.

**5. Predicates judge the gz truth state: yes.**
- The reason is what R2 and acro are for: they ask whether the vehicle's rotation recovers.
- Judging the received sample would test the sensor's noise against an envelope, not the vehicle. The noise still reaches the truth trajectory through the loop, and the propagated noise term covers that.
- That acro fails on the received sample at T4c is not the reason for this choice.
- Truth is read by the test harness only; G3 still holds for flight builds.

**6. Bias corners from the nonlinear design model: yes, as a delta only.**
- The bias term is (nonlinear model with the bias) − (nonlinear model without it), added to the linear envelope.
- The coupling itself never enters the envelope. Otherwise acro and R2 would stop asking whether the controller rejects it.

**7. Per push: PP-A.** It is the spec's split: every scenario at its committed seed per push, the confirmation seeds nightly. Run the confirmation seeds also before every stage close and every tag. Re-measure the gz times after the plugin's IMU, rotor and clock wiring (F4) before you fix the shard count.

**8. Keep 22 and 59: yes.**

**9. Clock corners inside the T3 corner search: yes.** This is my stage (a) and (b) ruling: T3 searches the corners, and T4 flies the nominal plus each scenario's worst corner.

**10. Question E, the Gazebo step rounding: outward, not nearest.**
- ±65 ppm is a bound. Nearest rounding (±64.004) never reaches it. Outward (±70.4 at m = 1, ±67.2 at m = 2) covers it, at negligible cost (≤ 0.16 mrad/s at 65 ppm).
- One condition: measure E at the nominal clock, which is exact at both m. That way the m-dependent corner value never enters E.
- Record both realised values in 0012's question E entry. If outward breaks anything the 0.16 mrad/s figure doesn't explain, stop and report.

**11. Structural ties fly one canonical corner: yes, for exact ties only.**
- Each tie needs a per-push test proving it bit-identical. For example, two corners differing only in accel bias give identical logs in L4 and L5.
- At L7 the estimator reads the accelerometer, so that test will fail and the ties will break by themselves.
- The chirp tie under a constant bias collapses only if it is shown to be exact. If it is approximate, fly it.

**12. Close S9 for pytest in the (e) CI change: yes,** with the planted-overrun control.

**The gate commit:** agreed as you wrote it. FF on, the gate file and the per-push FF-iff-gate check go in one atomic commit. If any item still fails, there's no gate and no per-item gates; FF stays off and the record comes to me."

Also from that round: "Worst T3 corner = smallest predicate slack per scenario: yes, normalised. Per scenario, take the corner
with the smallest slack divided by that channel's own bound, minimised over channels so units compare. Keep both corners
on a tie. It is computed on the T3 design model, never picked from Gazebo results." (0017, Approval.)

**The spec line (F1), approved:** "Spec line: approved as you wrote it. ... Approved: Luis, 2026-10-04." It is applied in
the same change as the `noise_term_confidence` register entry:

> Envelopes carry a noise term, derived by propagating the noise model through the design model at the per-seed
> quantile 1 − (1 − `noise_term_confidence`)/N, so that a correct build passes every seed with probability at least
> `noise_term_confidence`. The fine negative controls of 0006 decision 24 still fail with the noise term in place.

On c1 (the anti-windup law of the R2 round), 2026-10-04: "**Hold 0018. Don't commit it to master yet.** ... Park it on a
local branch, `l6-0018-c1`. Commit the tree there, don't push, and return master's tree to 4bfd66c. ... The (e)
infrastructure commits then proceed on master as ruled." And: "**Now, independent of c1: pin
`r2_ff_diagnosis/regenerate.sh` to the commit it records.** It goes in its own small commit, or with the first (e)
infrastructure commit. It is a reproducibility fix, so the old-law outputs stay reproducible whatever the law becomes."
When an anti-windup change does land, the L5 rate-bypass golden follows ruling (a): its generator is re-pinned to the
commit that introduces the law, and the test is renamed to say which law it reproduces. The quad-L4-pass golden is kept
as its negative control and must fail on the saturating rows. `l6_ff_eval.py` follows the law in the same commit, and
its committed sweeps are confirmed byte-identical.

## Why

The spec's (e) pass bar (quad §4 L6): the T3 Monte Carlo over noise seeds, the T4 reruns with noise and filters, and
both known failing items passing. The decision round found that the ACTIVE spec's noise-term line contradicts its
every-seed rule (F1). The T4 corners are now searched at T3 (decisions 0012 and 0013; rulings 9 and 11). The
per-push / nightly split keeps per-push within S9.

## What changed

**1. `r2_ff_diagnosis/regenerate.sh` is pinned to the commit it records** (a reproducibility fix; Luis, 2026-10-04).
- The pinned commit is `4a76755bedb5dfb07a26aa65653c1a2b35b5008e`, decision 0016's commit:
  - it is the only commit that ever touched `r2_ff_diagnosis/`, and the directory is unchanged since;
  - it introduced the `MARV_GZ_TEST_ZERO_FIRST_READ` switch.
- `PIN` is one variable at the top. The script builds and runs everything from a detached `git worktree` of that commit,
  removed at exit:
  - the host-gz-l5 build, the refdata, `capture_gz.py` with the switch, `build_tool.sh`, `build2.sh` and the Python
    scripts;
  - Python runs through the main `.venv` (`uv.lock` and `pyproject.toml` are unchanged since `4a76755`).
- The hash and small-file checks compare against the current tree's record directory. So a run also shows that the
  committed record equals the pinned outputs.
- Result on `4bfd66c` (the diffuser live): 35 hashed files and every committed small file reproduce, with 0 mismatches,
  in 424 s at 2 build jobs (331 s of it the cold pinned build and the refdata).

Frozen files changed by item 1:
- `tests/regression/quad/L06/results/r2_ff_diagnosis/regenerate.sh`: the pin.
- `tests/regression/quad/L06/results/r2_ff_diagnosis/README.md`: the regenerate section says what is pinned and why.
No data file, `SHA256SUMS` or summary changes.

## Evidence

Each item's checks are listed with it as it lands.

## Approval

Owner decisions: Luis, 2026-10-04, as quoted. The stage: open.
