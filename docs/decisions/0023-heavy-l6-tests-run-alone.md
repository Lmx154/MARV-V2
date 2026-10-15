# 0023: the CPU-heavy L6 tests run alone under a parallel ctest

## What changed

- `tests/regression/quad/L06/allan/CMakeLists.txt`: `unit_l6_allan_check` is registered with `RUN_SERIAL TRUE`.
- `tests/regression/quad/L06/tracking/CMakeLists.txt`: `unit_l6_tracking` is registered with `RUN_SERIAL TRUE`.

`marv_frozen_suite` passes the property to every test discovered in the two executables (`AllanT1.*`, `Tracking.*`).
CTest then runs each of them with no other test beside it; every other test still runs in parallel. No test source,
bound, tolerance, seed or timeout changed: the TIMEOUT stays `per_push_check_time_max` (`design/budget.yaml`).

## Why

Decision 0021 made `ci/run_ci.sh` run ctest with `--parallel $(nproc)`. These tests are single-threaded and CPU-bound
for seconds each; beside other tests on the runner's 4 CPUs their wall time grows, and the per-test TIMEOUT is wall
time. The first Actions run of the merged suite (run 38691769232, commit `e4e93cb`, job core, host-debug) failed on
four timeouts and passed a fifth test at 51.89 s of the 60 s limit:

| Test (host-debug) | Serial, local (batch `534498d`, before 0021) | Parallel, local (`e4e93cb`) | Parallel, Actions (`e4e93cb`) |
| --- | --- | --- | --- |
| AllanT1.NominalMatchesTheProfile | 10.49 s | 21.95 s | timeout (60 s) |
| AllanT1.NoiseDensityPlus10PercentIsRejected | 10.47 s | 21.47 s | timeout (60 s) |
| AllanT1.BiasInstabilityPlus10PercentIsRejected | not in the extract | 21.04 s | timeout (60 s) |
| Tracking.QuantisedCheckResolutionRule | 11.32 s | 20.95 s | timeout (60 s) |
| Tracking.UnquantisedCompanionDownToTheThreshold | not in the extract | 20.98 s | 51.89 s |

The tests did not get slower; the run around them did. The limit applies to one check's own cost (S9, decision 0009),
which is what running them alone measures again. The ctest of host-debug passed on Actions in every nightly run from
2026-10-05 to 2026-10-13, when ctest was serial.

Not chosen: turning ctest parallelism off for the whole run (costs every other test the speed-up of 0021 and 0022);
raising the limit (a design-budget number; it would loosen the check).

## Evidence

- Actions run 38691769232, job core: `99% tests passed, 4 tests failed out of 744`, the four `(Timeout)` lines above.
- Local, `e4e93cb`: `/tmp/marv-local-ci-logs-e4e93cb2-3023974/core.log` (host-debug and host-release ctest timings).
- Serial timings before 0021: `~/marv-handover-2026-10-13/ci/batch_534498d_core.log`.
- The margin of these tests on Actions when run alone is not yet measured; the first Actions run after this change
  measures it.

## Approval

Luis, 2026-10-15, on the two options (run only the heavy tests alone, or turn ctest parallelism off): "sure go with
option one".
