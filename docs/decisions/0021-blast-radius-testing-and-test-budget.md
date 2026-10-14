# 0021: blast-radius testing, a one-test smoke, the full suite only at the merge, and the test budget rules

The suite had grown until every change paid for all of it: one full run was about 75 min of job time, and the per-push
loop ran it from a fresh clone. Measured on the S4+S5 batch of quad-l11 (`~/marv-handover-2026-10-13/ci/`, three lanes,
4 CPUs each): builds from scratch about 2,900 s (Debug, Release, m33, four or five Gazebo plugin trees); the L5 T3
reference regenerated three times, about 870 s; the L5 gz T4 suite 740 s; the tools tests 303 s; the C++ ctest 156 s
over three passes, of which about 4 s is test bodies (the rest is launching 745 processes, serially).

## Owner decisions (Luis, 2026-10-14, verbatim)

"I need you to start creating new rules for testing. We have created testing hell in this repo. adding basic
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

"keep the full runs as a safety net and as a CI that is only ever run when we are finally going to merge to master"

## What changed

Rules (core §7.6 and §7.7, new; §7.1 "When" column; §7.2 item 2; quad SIM-2 and L6 (e) "nightly" lines; CLAUDE.md
"Frozen tests and decisions" and "Commands"):

1. Three tiers. **Smoke** (`ci/smoke.sh`): one test of the whole flight chain. **Affected** (`ci/dev_check.sh`): the
   smoke plus what `ci/affected.py` selects from the diff and `ci/blast_radius.yaml`; the check for development commits
   and branch pushes. **Full** (`ci/local_ci.sh`, Actions): only on the merge into `master` and on demand.
2. Fail-safe selection: a root everything depends on, or any path no rule maps, selects the full suite.
3. No nightly run (the schedule trigger is removed from `.github/workflows/ci.yml`; every Actions run is
   `MARV_CI_MODE=full`; `ci/local_ci.sh` defaults to full). Where the quad spec said "nightly" it now says "the full run".
4. Test budget rules (§7.7): a test needs a reason (pass bar, contract, guardrail, found defect); name the existing test
   that already catches the break or add nothing; extend before adding; consolidate at every freeze; report cost; trimming
   with a record is allowed, loosening is not; a negative control follows its test.

Machinery:

- `ci/blast_radius.yaml`, `ci/affected.py` (with `--check-manifest`, run by `ci/dev_check.sh` and as the merge-only
  CI step `blast_radius_manifest`; it cross-checks the map against the host-debug include paths, the Python import
  closures and the file reads of the tests; a review found six under-selections in the first map, and the cross-checks
  six more, all fixed before this change), `tests/tools/test_affected.py` (five cases: unmapped → full, root → full, docs → nothing,
  fw/vertical → L11 not L02, the manifest check passes).
- `ci/run_ci.sh`: `MARV_CI_ONLY`, `MARV_CTEST_LABELS`, `MARV_PYTEST_PATHS` selection hooks (unset or empty = today's
  behaviour); ctest runs with `--parallel $(nproc)`; `tests/tools` joins the tools tests; the `frozen_suites` step is
  deleted (it re-ran `ctest --preset host-release -L frozen` right after host-release's full ctest, 744 of 745 entries
  twice); the frozen-entry count is printed in the host-release step instead.
- `ci/smoke.sh`: builds `unit_l11_composition` and runs its 34 gtest entries and the 3 `l11_composition_param_ids*`
  script tests (label L11). It is the only existing test that runs card-derived parameters, gyro chain, rate, attitude,
  vertical, mixer and DShot in one process through the SIL ABI. No test body changed and no test was added for it.

- Trimmed (§7.7 item 1, no reason): `tests/unit/` (`smoke_test.cpp`, `EXPECT_EQ(1 + 1, 2)`, and its CMakeLists), its
  `add_subdirectory` in `CMakeLists.txt` and its mention in `cmake/marv_test_timeout.cmake`. Not frozen; every other
  gtest binary proves the same host test path. ctest now registers 744 entries, all frozen.

Frozen files:

- `tests/regression/CMakeLists.txt`: every test under `tests/regression/<product>/Lnn/` now carries the ctest label
  `Lnn` besides its own (gtest entries through an include file run after discovery; `add_test` entries by walking each
  layer directory after it is added). No test body, threshold, TIMEOUT or existing label changed.

## Why

Development waited on builds and suites the change could not reach. Ninja rebuilds only what a change touches, and the
selection runs only the tests that link or read it, so the development check costs what the change costs. The full
suite keeps every frozen test, control and gate; it moves to the one point where a regression would land on `master`.
The fail-safe and the merge gate bound the risk of a wrong map to time, never to coverage at the merge.

## Evidence

- Labels: `ctest --preset host-debug -N -L '^Lnn$'` gives L00 157, L01 43, L02 28, L03 14, L04 72, L05 141, L06 205,
  L11 84 = 744; `-N -L frozen` = 744 before and after; total 745 (the one other entry is `tests/unit`'s smoke).
- ctest host-debug: 745/745 pass serial (181.4 s) and `--parallel 16` (43.7 s), on Luis's host.
- `ci/smoke.sh`: 37/37 pass, 0.4 s on a warm tree, 0.8 s after `fw/vertical/src/vertical.cpp` is touched.
- `ci/affected.py`: `--check-manifest` passes; on b712151 (docs only) it selects nothing; on e08179e (S4) and 63cf5ae
  (S5) it selects the full suite (both changed build or CI roots).
- Python tests are selected per file for a changed `.py` (its import, spawn and read closure; anything unresolvable falls
  back to selecting more), per directory for any other changed file.
- `ci/dev_check.sh` on warm trees (scratch worktree of this change, Luis's host, 2026-10-14): a docs-only change 0.8 s
  (smoke only); a `fw/vertical` change: smoke, L11 ctest (84), flight gates, m33, product tables and gz l11 in about
  22 s plus G1 (G1 could not run: no `clang-tidy-18` on the host); a `tools/sim/l6_noise_term.py` change: 293 s with
  directory-level pytest, 20.8 s for the one selected file after per-file selection.
- Independent reviews: one found six under-selections in the first map (fixed; the self-check then found six more,
  fixed); one found none in the blind spots the self-check cannot see (spawned binaries, link-only libraries, runtime
  reads in C++, CMake-generated inputs); one passed the tiers, the step lists, the labels and the scope.
- The full merge-gate run on this change's head: recorded in `docs/handoff.md` when it runs.

## Approval

Luis's approval of the pull request (or his push) that merges this change into `master`.
