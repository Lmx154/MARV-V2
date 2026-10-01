# L4 T3: the rate loop against a double oracle

Quad spec 4 L4, decision 0005 "T3". The firmware `RateLoop<float>` from `rate::load_config()` is closed with the
design plant `J w' = u_m`, `tau u_m' = u - u_m` in double, and its step responses are compared with an independent
double oracle within a derived rounding tolerance.

## Files

| File | Role |
| --- | --- |
| `t3_test.cpp` | The harness and the assertions (gtest, label `frozen`). |
| `reference/rate_t3_oracle.py` | The independent oracle: law, plant, impulse responses, tolerance, band envelope. |
| `reference/rate_t3_inputs.txt` | The fixture: the f32 gains, tau_ref, inertias, motor tau, rate_max, tick and band values that both the oracle and the test read (hex floats). It equals the product parameter values on 2026-09-30. |
| `reference/SHA256SUMS` | The committed SHA-256 of the two generated files below (`sha256sum` format). |
| `rate_t3_golden.txt` (generated, not committed) | Per axis: trajectory `y_n`, the `l1_*` norms, the `rho_*` injections and the `tolerance`. |
| `rate_t3_envelope.txt` (generated, not committed) | Per axis: pointwise min / max of the band-box step responses (for T4). |

The golden and the envelope are not committed (decision 0011). `tools/refdata/refdata.py` generates them from the committed
inputs and the oracle into `build/reference/quad/L04/t3/` (or `$MARV_REFERENCE_DIR`), together with copies of the inputs, and
checks them against `reference/SHA256SUMS`; the ctest fixture `marv_reference_quad_L04_t3_ensure` does it before `t3_l4_rate`
runs, and a mismatch fails naming the file and both hashes:

    uv run python tools/refdata/refdata.py ensure quad/L04/t3

Generate them by hand (byte-identical on the host and in the `marv-ci` image, python3 stdlib only):

    uv run python tests/regression/quad/L04/t3/reference/rate_t3_oracle.py --dir <dir with rate_t3_inputs.txt>

Regenerate the inputs from a built product parameter table (needed only when a parameter changes):

    uv run python tests/regression/quad/L04/t3/reference/rate_t3_oracle.py --refresh-inputs \
        build/<preset>/generated/marv_params/marv/params/param_defaults.cpp

Goldens are regenerated only inside the CI image (CLAUDE.md). A changed golden means: regenerate in the image, update
`reference/SHA256SUMS` from the new files (`sha256sum`), and write a decision record (core 7.3). The inputs are a fixture: the T3 test builds its `RateConfig` (gains, tau_ref, period) and its plant
from `rate_t3_inputs.txt` and does not read the live product parameters, so a legitimate parameter change (a B1 card
update, say) does not fail T3. The rotor geometry is left at its zero default (the mixer is out of the loop). The rule
that produces the gains is tested on the live parameters by `tools/test_rate_design.py`, and `from_params` /
`load_config` by `unit/rate/params_test.cpp`. CI (`ci/run_ci.sh`) regenerates the golden and the envelope from the
fixture and checks them against `reference/SHA256SUMS`, and shows that a perturbed input fails that check.

## Scenario

Per axis a, nominal J (`inertia_xx/yy/zz`) and tau (`motor_tau`): the setpoint steps from 0 to `rate_max_a` at
execution 1, the other axes are 0. Ticks are 625/4 us; the controller runs every `rate_loop_divisor` ticks with stamp
floor(tick x 625 / 4) us, so `dt` alternates 312 / 313 us against the design T = 312.5 us (both the oracle and the
harness use the stamps; the plant advances by exactly 312.5 us). The gyro is the plant omega at that tick as a float
with GyroValid; the torque is held until the next execution (zero computation delay). Execution 1 seeds the prefilter
from the gyro, so it outputs 0 and the step is first seen by execution 2. Mixer out of the loop, `record_allocation`
not called (the freeze flags stay false).

Horizon: N = 1 + ceil(10 tau_ref / T) executions (scenario test value 10: ten reference time constants). At the horizon
the roll response is 0.014 rad/s (0.12 %) from the setpoint (the integral mode is slower than tau_ref), after a peak
overshoot of 1.58 rad/s.

## Tolerance (derived)

Convention: `u = 2^-24` is the binary32 unit roundoff (round to nearest), epsilon = 2u = 2^-23; every float operation
returns `(1 + d) x exact` with `|d| <= u`. `expf` is assumed within 1 ulp of its result (`ulp32`). The bound is first
order (terms of order u^2 dropped) and uses the double trajectory for the operand magnitudes.

The float law has four nodes where rounding enters; the injection bound `rho` of each at execution n >= 2, with
`s = |sp - r_(n-1)|`:

| Node | Float operations | rho_n |
| --- | --- | --- |
| r (prefilter) | `dt = fl(dt_us / 1e6)`, `x = fl(dt / tau_ref)`, `exp`, `alpha = 1 - exp` (exact by Sterbenz, u alpha kept), `fl(sp - r)`, `fl(alpha t)`, `fl(r + p)` | `d_alpha s + 2 u alpha s + u abs(r_n)`, `d_alpha = e^-x 2 u x + ulp32(e^-x) + u alpha` |
| e | gyro cast `omega -> float` (u abs(omega)), `fl(r - y)` | `u (abs(omega_n) + abs(e_n))` |
| I | `dt` (u), `fl(ki e)`, `fl(. dt)` (2 u), `fl(I + inc)` | `3 u abs(ki e_(n-1) dt_n) + u abs(I_n)` |
| u | `fl(kp e)`, `fl(. + I)`, `+ d` (d = 0 exactly, kd = 0) | `u abs(kp e_n) + u abs(u_n)` |

`r` and `e` carry the setpoint-path and the measurement-path errors; the alpha error `d_alpha` (dominated by the
expf ulp, about 3e-5 of alpha) is the largest single contribution. An injection `w` at node p of execution k moves
`y_m` (m > k) by `g_p(m - k) w` where `g_p` is the impulse response of the closed loop (prefilter state, integrator
state and stored error included) from that node to omega. The oracle computes `g_p` for injections at executions 2
and 3 (the two dt phases; every later injection is a shifted copy of one of them) over the horizon, and takes
`l1_p = max over the two phases of sum abs(g_p)`. The tolerance is

    TOL_a = l1_r max_n(rho_r) + l1_e max_n(rho_e) + l1_I max_n(rho_I) + l1_u max_n(rho_u)

The plant and the oracle are double: their rounding is 2^-29 of a float operation's and is not in the sum. The test
checks `TOL = sum(l1 x rho)` from the generated numbers (`ToleranceIsTheSumOfL1TimesRho`).

## Negative controls (quad 4 L4 pass bar)

Each must leave the tolerance (asserted in `L4T3Step`): (a) every gain (kp, ki, kd) x 1.1 through a modified
`RateConfig`; (b) one tick of added delay between the controller output and the plant (the first tick of every
execution still applies the previous torque).

## Band envelope (recorded, robustness only)

`rate_t3_envelope.txt`: per execution, min and max over J (1 +- inertia_robustness_band) x tau (1 +- tau_robustness_band)
of the same step response with the nominal-plant f32 gains and prefilter; the 17 x 17 grid, which contains the corners
and the 9 x 9 grid, and `halving_max_change` is the largest change of any envelope point from the 9 x 9 grid to the
17 x 17 one. The test only asserts that the nominal float trajectory lies inside it.

The halving change is recorded per axis in the file (`halving_max_change`): roll 0.04024355, pitch 0.04024358, yaw
0.04024350 rad/s. T4 widens the envelope by it.
