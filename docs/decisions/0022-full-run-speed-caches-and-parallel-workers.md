# 0022: the full run made faster: compiler cache, cached reference sets, control verdicts and flatten outputs, parallel workers

Decision 0021 moved the full suite to the merge into `master`. That run still cost about 75 min of job time
(`~/marv-handover-2026-10-13/ci/`, the S4+S5 batch): about 48 min compiling from scratch in fresh clones, about
14.5 min regenerating the L5 T3 reference set three times (core 268 s, its one-ulp control 309 s, gz-l5 290 s), the
L5 gz T4 suite 740 s, the tools tests 303 s.

## Owner decision (Luis, 2026-10-14, verbatim)

On the list "a compiler cache, one Gazebo build instead of four, cached reference data, and parallel Python and ctest
[...] Running the one-ulp control on one file instead of all three cuts another ~4 minutes, but that one needs your
OK": "this leads me to believe that we're not taking advantage of low hanging fruit for optimization. i need you to do
this, so that even the full suite runs faster".

## What changed

1. **Compiler cache.** ccache in both CI images, used as the compiler launcher when present (host and m33); a
   persistent cache directory mounted at `$MARV_CACHE_DIR` (`/cache`) in every CI container: a named docker volume for
   `ci/local_ci.sh` (`--no-cache` uses a throwaway one), `actions/cache` per job on Actions. Docker image layers are
   cached on Actions. `ci/local_ci.sh --parallel N` runs jobs side by side. The four Gazebo plugin trees stay separate
   builds; they share the compiler cache.
2. **Reference sets are cached, with a complete stamp** (decision 0011 regenerated them in every run). The stamp of a
   set is now the sha256 manifest of its inputs, its generator, the generator's repo-local import closure, every repo
   file that closure reads (resolved statically; reads it cannot resolve must be declared per set or the stamp fails),
   `uv.lock`, the interpreter, numpy, libc and OS versions. With `$MARV_CACHE_DIR` set, a set whose stamp is current is
   verified against the committed `SHA256SUMS` (hashes only) and used; otherwise it is regenerated and stored. A
   tampered entry fails the verification and is regenerated. Modules whose reads cannot be resolved statically are
   listed per set (`unresolved_ok`); a new runtime read in one of them must be declared in that set's `depends`.
   Without the cache (development trees), `--force` as before. The reproduction property is unchanged: a cache entry
   exists only because the generator produced the committed bytes from exactly the stamped closure.
3. **Not done: the one-ulp control on fewer files.** A `--skip-q` option in `attitude_t3_oracle.py` (the control
   generating only the golden and the envelope) was built and withdrawn: the oracle's sha256 is recorded in two frozen
   L06 results (`results/r1x_coupling/coupling.txt`, `results/r2_envelope_a/`), so any edit to the oracle fails their
   byte-for-byte tests. The controls run as at decision 0011; item 4 removes their cost when nothing changed.
4. **Control verdicts are cached.** With `$MARV_CACHE_DIR` set, the recorded failure of a T3 one-ulp control is reused
   while its key is unchanged: the set's stamp manifest, its SHA256SUMS, the perturbation, the verify code and the
   control's own text. Any change to any of them re-runs it. Without the cache it always runs.
5. **Parallel workers** (pytest-xdist, `--dist loadfile`, one file per worker so module fixtures run once): the tools
   tests in the full run and the Gazebo suites. Every gz run already has its own transport partition
   (`tools/sim/run_scenario.py` `gz_env`) and temporary directory, and no test asserts on wall time. The development
   check keeps the tools tests serial under the per-file time guard, which measures alone time and cannot run under
   workers.
6. **The card flatten step is cached** (found while measuring item 1: the m33 build was four `tools/card/flatten.py`
   runs of about 309 s, not compilation). `tools/card/flatten_cached.py` wraps the build's flatten command: without
   `$MARV_CACHE_DIR` it runs `flatten.py` unchanged; with it, the outputs are stored under a key of flatten's import
   closure and the repo files it reads (the same resolver as item 2), the card, budget and scenario files, `uv.lock`,
   the arguments, the interpreter, numpy, scipy, libc, OS and CPU model, and an entry is used only if every stored file
   matches its manifest hash. A lint finding still fails the build and stores nothing. The four parameter sets flatten
   identical inputs to identical bytes, so one run serves all four, in every tree.

Frozen files: none.

## Why

The merge run repeated work whose result was already known: identical compilations, a reference set regenerated from
identical inputs three times, a control regenerating files it never checks, and single-process test runs on 4 CPUs.
None of these changes what any test checks; the caches are keyed on everything the cached result depends on.
No file under `tests/regression/` changes.

## Evidence

- Stamps: `uv run python tools/refdata/refdata.py stamp quad/L05/t3` lists 33 files (inputs, oracle, 13 tools/card and
  7 tools/sim modules, the card, the budget and scenario values, two scenarios, the sensor profile, constants.hpp,
  marv_plant.h, uv.lock) plus the environment lines.
- Reference steps on Luis's host: cold 383 s, warm 186 s before control caching; invalidation shown for a closure
  module (`tools/sim/run_l4.py`) and a read file
  (`scenarios/quad/L05/step_roll.yaml`); a flipped byte in a cached golden is rejected.
- Control verdicts: both T3 controls 188 s on a miss, 0.8 s on a hit (recorded failures: rate_t3_golden.txt and
  rate_t3_envelope.txt; attitude_t3_golden.txt and attitude_t3_envelope.txt); appending a comment to
  `tools/sim/hover.py` (in the L5 closure only) made the L5 control a miss and left the L4 control a hit; without the
  cache both always run. A record is written only after every check of the control has passed.
- Parallel workers (`taskset -c 0-3` on Luis's host, other load present): the full-run tools tests 1000 s serial,
  765 s with 4 workers (1176 passed in both; the same two failures in both, the L06 byte-for-byte tests that record the
  oracle's sha256, caused by the withdrawn `--skip-q` edit; they pass once it is withdrawn); `ci/run_ci_gz.sh l5`
  T4 step 695 s serial, 514 s with 4 workers
  (63 passed, 2 xfailed in both); `ci/run_ci_gz.sh l2` passes every step with workers. A planted skip still fails
  `pytest_no_skips` with and without workers. Shared paths checked for races: `/tmp/log.bin` in
  `L05/tools/test_truth_attitude_log.py` is only text in a generated world, never written; the git repository of
  `L00/tools/test_check_regression_changes.py` is the per-test `tmp_path`. `MARV_PYTEST_WORKERS=0` runs serially.
- Compiler cache, `ci/local_ci.sh core` on one commit (the caches only, before items 2, 4 and 5), 4 CPUs: no cache
  2296 s (ccache 8/377 hits), warm 2129 s (377/377). Compilation is a small part of the job: host-debug 391 → 338 s,
  host-release 416 → 321 s, m33 334 → 309 s, the m33 step being four `tools/card/flatten.py` runs of about 309 s each
  in parallel. ccache holds about 34 MB after a core run. In the Gazebo trees 140 of 142 translation units compile
  identically across the four variants; `lockstep.cpp` and `lockstep_log.cpp` differ by define and include path.
- Found while measuring: `tests/tools/test_affected.py` (decision 0021) fails in a container on a clone owned by
  another user (git "dubious ownership"); both container wrappers now mark `/src` as a safe directory.
- Flatten cache (Luis's host, 16 CPUs): the four sets uncached 126.6 s wall (28.5 CPU-min); cold cache 49.7 s (one
  run, three hits); the next tree 0.5 s. All 4 sets x 13 files byte-identical (`cmp`) to an uncached run, in
  host-debug, host-release and m33. A closure module, the card or the sensor profile changes the key; a truncated or
  bit-flipped entry is rejected and regenerated; a planted lint finding fails all four commands. The cost is
  `rate_lead.margins` (3007 calls of a pure-Python frequency sweep, 175 of 275 s serial), left unchanged.
- Timings of the cold and warm full runs with every item: added as they are measured.

## Approval

Luis's approval of the pull request (or his push) that merges this change into `master`.
