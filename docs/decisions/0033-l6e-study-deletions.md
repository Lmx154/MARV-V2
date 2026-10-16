# 0033: delete the L6 stage (e) study tests

## What changed

Packet P4 of decision 0029. Seven pytest files under `tests/regression/quad/L06/tools/` are deleted, and `ci/run_ci.sh`
drops `test_r1x_coupling.py` from `nightly_tools_tests` and a comment naming `test_r2_lower_bound.py`;
`tests/tools/test_affected.py` points its Python-closure case at `tools/sim/l6_ff_eval.py` and `test_l6_ff_eval.py`
instead of the deleted `test_l6_noise_term.py` (same check, a surviving pair):

| Deleted | What it was | What still covers it |
| --- | --- | --- |
| `tests/regression/quad/L06/tools/test_l6_noise_term.py` | the stage (e) noise term W of the Monte Carlo envelopes (`tools/sim/l6_noise_term.py`) | nothing, it was a study; the Monte Carlo is deferred (quad §4.0) |
| `tests/regression/quad/L06/tools/test_l6_bias_delta.py` | the stage (e) bias delta and the nonlinear hooks of `l6_ff_eval.py` | nothing, it was a study; `test_l6_ff_eval.py` stays (stage (c) pass-bar line) |
| `tests/regression/quad/L06/tools/test_r2_lower_bound.py` | R2 lower-bound proof, re-run of `results/r2_lower_bound/` | nothing, a study; R2 is now S and B in `L05/gz/test_t4_recovery.py` (decision 0031) |
| `tests/regression/quad/L06/tools/test_r2_e_measured.py` | R2's E measured on its own two runs | nothing, a study; E = \|m1 − m2\| is measured per run by the T4 tests (SIM-7) |
| `tests/regression/quad/L06/tools/test_r2_envelope_a.py` | R2 under envelope A | nothing, a study; the design-model envelope is gone at T4 (decision 0029) |
| `tests/regression/quad/L06/tools/test_r1x_coupling.py` | R1X coupling re-run of `results/r1x_coupling/` | nothing, a study; R1X is S and B (or a strict xfail) in `L05/gz/test_t4_recovery.py` |
| `tests/regression/quad/L06/tools/test_ff_on.py` | feed-forward-on records, "reported, not asserted" | nothing, it asserted no verdict |

Gap stated (decision 0029, invariant 4): large-angle recovery has no T3 check. Every `results/` directory stays as
evidence, the scripts and raw output included. `tools/sim/l6_noise_term.py` and `tools/sim/l6_bias_delta.py` stay; no
test imports them now. `ci/blast_radius.yaml` needed no change: no rule names a deleted test, and the rule that names those
tools also names `l6_ff_eval.py`, whose test stays. Quad §4.1 L6 names none of the seven tests, so it is unchanged.

## Why

Core §7.2 item 6 and decision 0026: a study is not a requirement. These tests re-ran the L6 (e) studies for the
design-model envelope, the noise term and the R2/R1X analysis; none traces to a pass-bar line, and the design-model
envelope comparison went with decision 0029. They cost CI time (`test_r1x_coupling.py` 61 s nightly, `test_r2_lower_bound.py`
58 s per push).

## Evidence

`uv run pytest tests/regression/quad/L06 --co` collects 281 tests; `uv run pytest tests/regression/quad/L06/tools` 178
passed; `uv run pytest tests/tools/test_affected.py` 6 passed; `ci/affected.py --check-manifest` manifest OK;
`ci/dev_check.sh --base HEAD` on this change alone (clean worktree): all steps passed.

## Approval

Luis, 2026-10-15, "1,2,3 all yes" (decision 0029).
