# 0011: the T3 reference files are generated and hash-checked, not committed

## What changed

Five committed, regenerable T3 reference files are removed and replaced by a generator, its committed inputs and a
committed SHA-256 per file. Nothing about the reference changes: each file is byte-identical to what the oracle writes
(Evidence), and `SHA256SUMS` holds the SHA-256 of the file as it was committed.

Deleted (generated from the committed inputs and the committed oracle, 8.6 MB together):

- `tests/regression/quad/L04/t3/reference/rate_t3_golden.txt`
- `tests/regression/quad/L04/t3/reference/rate_t3_envelope.txt`
- `tests/regression/quad/L05/t3/reference/attitude_t3_golden.txt`
- `tests/regression/quad/L05/t3/reference/attitude_t3_envelope.txt`
- `tests/regression/quad/L05/t3/reference/attitude_t3_q.txt`

Added: `tests/regression/quad/L04/t3/reference/SHA256SUMS` and `tests/regression/quad/L05/t3/reference/SHA256SUMS`
(`sha256sum` format); `tools/refdata/refdata.py` (python stdlib only: `ensure <id>` generates into
`$MARV_REFERENCE_DIR`, default `build/reference`, copies the inputs in, regenerates when a file is missing, differs or was
generated from other inputs, then checks every file against `SHA256SUMS` and exits non-zero naming the file and both hashes;
it never rewrites `SHA256SUMS`; `reference_dir(id)` does the same for Python); `cmake/reference_data.cmake`.

Modified frozen files (the path of the generated files only; no predicate, bound, tolerance, envelope or test name changes):

- `tests/regression/quad/L04/t3/CMakeLists.txt` and `tests/regression/quad/L05/t3/CMakeLists.txt`: `MARV_L4_T3_REFERENCE_DIR`
  and `MARV_L5_T3_REFERENCE_DIR` point at the generated directory (it also holds copies of the inputs, so `t3_test.cpp` is
  unchanged); a ctest fixture setup test runs `ensure`, and `t3_l4_rate` and `t3_l5_attitude` require it (label `frozen` kept).
- `tests/regression/quad/L04/t3/README.md` and `tests/regression/quad/L05/t3/README.md`: file tables and regenerate
  instructions. A changed golden now means: regenerate in the image, update `SHA256SUMS`, write a decision record.
- `tests/regression/quad/L05/tools/test_attitude_t3_q.py` and `tests/regression/quad/L05/tools/test_l5_scenario.py`:
  read `attitude_t3_q.txt` and `attitude_t3_envelope.txt` from `refdata.reference_dir("quad/L05/t3")`.
- `tests/regression/quad/L05/gz/test_t4_steps.py`, `tests/regression/quad/L05/gz/test_t4_yaw.py` and
  `tests/regression/quad/L05/gz/test_t4_recovery.py`: the same, for the envelope and Q files; inputs and the oracle are still
  read from the source directory.
- `tests/regression/quad/L05/results/step_cause/step_cause.py` and
  `tests/regression/quad/L05/results/step_controls/step_controls.py`: the same, for the golden, envelope and Q files.

Outside `tests/regression/`: `CMakeLists.txt` (includes `cmake/reference_data.cmake`), `ci/run_ci.sh` (the two "T3 oracle
reproduces" steps become "regenerate and check against `SHA256SUMS`", moved ahead of the tools tests because those read
the generated files; each negative control, a one-ulp perturbed input, now requires the perturbed output to fail the
`SHA256SUMS` check, naming `rate_t3_golden.txt` for L4 and `attitude_t3_golden.txt` and `attitude_t3_envelope.txt` for L5),
`ci/run_ci_gz.sh` (group l5 ensures `quad/L05/t3` before pytest; group l4 reads only the inputs and the oracle).

Stay committed, untouched:

- `tests/regression/quad/L05/unit/rate_bypass/reference/rate_bypass_acro_golden.txt` (and the fixture): regenerating the acro
  golden needs the tag `quad-L4-pass` and the `marv-ci` libm, so it is not a plain generator run.
- `tests/regression/quad/L01/unit/prim/gravity_golden.csv` and `tests/regression/quad/L00/replay/golden_l0_trace.txt`: no CI
  regeneration exists for them, and they are small.
- `tests/regression/quad/L01/unit/plant/reference/plant_ref_expected.txt`: small, and CI already reproduces it.
- Every `*_inputs.txt` and `*_q_inputs.txt`: owned fixtures, the inputs of the generators.
- `tests/regression/quad/L04/results/qf8/qf8_curve.txt` and every `results/*/cause.txt` and `controls.txt`: small derived
  results.

## Why

Owner decision (Luis, 2026-09-30, verbatim; `docs/decisions/0009-l6-staging-and-choices.md` owner decision 2):

> Storage: generator + inputs + SHA-256, as you propose. No LFS. Removal of the committed files gets its own decision
> record; no history rewrite.

His reason: "The T3 reference data is 8.6 MB and will grow each layer. Since it's regenerable by rule in the pinned
image, propose committing the generator plus a content hash instead of the data". His order: "T3 storage migration
first (with its decision record), then stage (a), so no L6 reference data ever gets committed."

The bar is not loosened. The reference is the same bytes, pinned by SHA-256 instead of stored; the tests read the same
content and every predicate is unchanged. A generated file that differs from `SHA256SUMS` fails by name, never
silently. History is not rewritten: the old blobs stay in history.

## Evidence

- Host regeneration equals the committed files, byte for byte, before any deletion (`cmp` on all five; the oracles run
  with `--dir` on a copy of the committed inputs, `--procs 16` for L5).
- SHA-256 of the committed files, now in `SHA256SUMS`:
  - `04436adf3f7a9ec80208f848358b722669c3a608d952f7d557a2e70ef13eb8af` `rate_t3_golden.txt`
  - `e489c374103de845a654e2fae036a66a32ec05d126e1b0ac1bf8bc64eecfab73` `rate_t3_envelope.txt`
  - `c63d108f6fa12574dda0568a7f0efeb109677e8bc0d3b2c9ca9f52ca85a8a8c4` `attitude_t3_golden.txt`
  - `16edf2a74b9d787f0c01d477268fb451e87a363e073740d2617802124c76f761` `attitude_t3_envelope.txt`
  - `a666b1af9c534aad4463b76bee1c596e95d18761677fcb27754aeb62d5a848c6` `attitude_t3_q.txt`
- `tests/regression/quad/L05/results/step_controls/controls.txt:6` pins the envelope and the Q file at these hashes.
  `tests/regression/quad/L05/results/step_cause/cause.txt:6` pins `b318f8e7...3c04`, the envelope of `f9c3eeb`, before
  the N = 1 regeneration of `0533044`. The run was at att_loop_ratio 2 (H = 10372, `cause.txt:11`), so the whole file
  describes the configuration from before `0533044`, not just line 6. It is left as it is (Luis, 2026-10-01).
  `step_cause.py` is re-run at N = 1 in L6 stage (c), when D7 regenerates L5, under that stage's record.
- From a fresh `build/host-debug` and no `build/reference`: `uv sync --frozen && cmake --preset host-debug && cmake --build
  --preset host-debug && ctest --preset host-debug`: 456 of 456 passed (454 before, plus the two fixture setup tests), the
  6 `L4T3*` and 14 `L5T3*` tests among them ran and passed.
- Controls: a corrupted generated file is regenerated and passes; an edited `SHA256SUMS` line makes `ensure` exit 1,
  naming `rate_t3_golden.txt` with both hashes; an edited input exits 1 naming the golden and the envelope.
- `uv run pytest tests/regression/quad/L04/tools tests/regression/quad/L05/tools -q`: 285 passed.
- The CI steps, run from `ci/run_ci.sh` on the host: the L4 and L5 regenerate-and-check steps pass (L5 about 2 min at
  `--procs 4`, where the old step ran the oracle on one process); both negative controls fail the `SHA256SUMS` check as
  required (L4: the golden and the envelope differ; L5: the golden, the envelope and the Q file all differ).
- Full CI, `MARV_CI_BASE_REF=master ci/local_ci.sh --commit 6ae19f4 core gz-l2 gz-l4 gz-l5`. `6ae19f4` is a temporary
  commit of this change on top of `3158fb9`. The run was a fresh clone, as root, `--cpus 4 --memory 15740260352`:
  - **core:** ALL STEPS PASSED in 549 s.
    - Regression-change check: 16 frozen files, 1 decision record.
    - ctest 456/456 (debug) and 456/456 (release). The frozen suite (`ctest -L frozen`) gave 455/455, the two
      reference setup tests pulled in by their fixtures.
    - The L5 regenerate step, at `--procs 4`, peaked at 0.26 GiB.
  - **gz-l2:** passed in 128 s.
  - **gz-l4:** 40 passed, 1 xfailed, in 179 s.
  - **gz-l5:** 63 passed, 1 xfailed, in 579 s. 0 skipped in any job.
- Limit of that run: `local_ci.sh` runs every job in one clone. So gz-l5 found `build/reference` already generated by
  core and only checked it against `SHA256SUMS`. On Actions each job checks out afresh, and gz-l5 generates the set
  itself. That generation is the same command, under the same limits, that core ran above.

## Approval

Luis, 2026-10-01, "Approved."
