# L5 rate-loop bypass (T1, invariants I-A1 to I-A4)

Quad spec 4 L5, decision 0006 "A". `RateLoop<float>::execute_bypass(imu, reference)` runs the L4 PID law on a given
reference, skipping only the reference-model prefilter; `execute` is unchanged. `rate_bypass_test.cpp` (gtest, label
`frozen`) holds the tests; each has a negative control.

| Test | Invariant |
| --- | --- |
| `L5RateBypassAcro` | I-A1: `execute` reproduces the golden bit for bit; control: kp one ulp up (each axis) does not. |
| `L5RateBypassOnlyPrefilter` | I-A2: `execute_bypass` with r from the header's prefilter formula is bit-equal to `execute` (torque, faults, count, integrator); control: r one ulp up at one execution breaks it. |
| `L5RateBypassSwitchBack` | I-A3: `execute` after bypassed executions continues from the last applied reference; control: a prefilter seeded from the gyro differs. |
| `L5RateBypassFaults`, `L5RateBypassDeathTest` | I-A4: a NaN reference, a cleared GyroValid and a non-finite output behave as in `execute`; a spacing beyond 1 us of T panics; control: a run without a fault does not satisfy the check. |

## I-A1 files

| File | Role |
| --- | --- |
| `reference/rate_bypass_inputs.txt` | The L04 T3 inputs (gains with kd = 0, tau_ref, tick), copied from tag `quad-L4-pass`; owned by this test. |
| `gen_fixture.py` | Writes `reference/rate_bypass_fixture.txt` (stdlib only; a fixed xorshift64* stream). |
| `reference/rate_bypass_fixture.txt` | 512 scripted executions: gyro, setpoint, L3 saturation flags, requested and achieved torque (binary32 bit patterns), five scripted faults. |
| `acro_scenario.hpp` | The scenario runner shared by the test and the generator (`execute`, `record_allocation`, `integrator` only). |
| `golden_gen.cpp`, `regenerate.sh` | Compile the runner against `fw/` extracted from git ref `quad-L4-pass` (never modifying it) and write the golden. |
| `reference/rate_bypass_acro_golden.txt` | Per execution: torque, fault_active, fault_latched, fault_count and integrator bit patterns. |

Regenerate the fixture (byte-identical everywhere):

    uv run python tests/regression/quad/L05/unit/rate_bypass/gen_fixture.py

Regenerate the golden from `fw/rate` at `quad-L4-pass`. The golden is regenerated only inside the `marv-ci` image
(CLAUDE.md; `expf` comes from its libm). Needs the tag in the repository:

    docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci \
        tests/regression/quad/L05/unit/rate_bypass/regenerate.sh

`regenerate.sh [--ref REF] [--inputs FILE] [--out FILE]` also writes elsewhere or from perturbed inputs (CI uses both to
reproduce the golden and to show that a kp one ulp up does not reproduce it).

A change under `tests/regression/` needs a decision record (core 7.3). The golden survives L6 unless L6 changes the
prefilter or the PI arithmetic; that change then needs its own record.

## Scenario values

The fixture constants (seed, 512 executions, 32-execution setpoint hold, gyro scale 1/4 of rate_max, saturation flag
on about half the executions, achieved = requested on about a third and half of requested otherwise, the fault
indices) are labelled scenario values in `gen_fixture.py`. The test values (T = 1 ms, gains, tau_ref = 10 T, seeds,
execution counts) are labelled in `rate_bypass_test.cpp`.
