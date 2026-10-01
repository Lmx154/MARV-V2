# 0010: the L5 chirp tests keep one axis's runs in memory at a time

## What changed

`tests/regression/quad/L05/gz/test_t4_chirp.py`, frozen since L5 reached `master` at `e9bfaae`. `measure()` now calls
`_CACHE.clear()` after its cache lookup misses and before it flies a new axis, so the cache holds one axis's `Chirp`
(its `runs`, the parsed logs) instead of all three. Before, `_CACHE` kept every axis for the whole session. One line
added. Not changed: the predicate, the bounds, the envelopes, the negative controls, the test names, the run order,
what is printed, and every other file.

## Why

The L5 Gazebo suite reached 18.4 GiB of container memory on 4 CPUs. GitHub Actions run 36790931918 on `33ac37b`: the
`gz_l5` step was killed on the runner (about 16 GB) and printed no pytest summary. Each `Chirp` in `_CACHE` holds, in
`runs`, the fully parsed log of every run of its axis (roll and pitch 7 runs each, yaw 4), and nothing freed them.

The first check was whether the tests read only derived values from each run, so that the logs could be discarded
after measuring. They do not. `test_chirp_margin_meets_qf3` reads each run's raw log after `measure()` returns
(`tests/regression/quad/L05/gz/test_t4_chirp.py:243`, `run_scenario.complete_trailer(s.step.run.log, ...)`, which checks
the trailer counts against `len(log["steps"])`, `len(log["ticks"])` and `len(log["applied"])`). So the per-axis clear is
the allowed form. It is sound because the module-scoped `axis` fixture runs all of one axis's tests together, and
pytest tears down that axis's `chirp` and `control` fixtures before it sets up the next axis's, so the cleared entries
are unreachable.

## Evidence

Both runs: `marv-ci-gz` as root, `--cpus 4`,
`uv run pytest tests/regression/quad/L05/gz -q -rs -s`, memory sampled every 2 s from the host with
`docker stats --no-stream --format '{{.MemUsage}}'` (maximum taken). Before is `33ac37b`; after is the same tree plus
this change.

- **Peak memory.** Before 18.33 GiB (182 samples). After 7.37 GiB (177 samples).
- **Same results.** Before: 63 passed, 1 xfailed, no skips, in 548 s. After: 63 passed, 1 xfailed, no skips, in 539 s.
- **Identical output.** The two captured outputs are byte-identical once only the wall-clock fields are normalised
  (`gz wall N s`, pytest's `in N.NNs (h:mm:ss)`, and the `/tmp/pytest-of-<user>/pytest-N/` report paths). Every
  numeric result (PMs, crossovers, margins, stale counts, peak |y|) matches exactly.
- **Other suites.** `uv run pytest tests/regression/quad/L05/tools -q`: 149 passed.

## Approval

Luis, 2026-09-30: "(a), recorded in 0010. First check: if the chirp tests only read derived values (margins,
crossover) from each run, cache those and discard the parsed logs after measuring, instead of the per-axis clear. Use
the per-axis clear only if some test needs raw logs."
