# L4 T3: the rate loop against a double oracle

Quad spec 4 L4, decision 0005 "T3"; quad spec 4 L6 stage (c), decision 0014. The stage (c) rate path, the firmware
`GyroChain<float>` in front of the firmware `RateLoop<float>` (with the D term and its low-pass), is closed with the
design plant `J w' = u_m`, `tau u_m' = u - u_m` in double, and its step responses are compared with an independent
double oracle within a derived rounding tolerance.

## Files

| File | Role |
| --- | --- |
| `t3_test.cpp` | The harness and the assertions (gtest, label `frozen`). |
| `reference/rate_t3_oracle.py` | The independent oracle: law, plant, impulse responses, tolerance, band envelope. |
| `reference/rate_t3_inputs.txt` | The fixture: the f32 gains, D low-pass time constants, tau_ref, gyro chain configuration, inertias, motor tau, rate_max, tick and band values that both the oracle and the test read (hex floats). It equals the product parameter values on 2026-10-01 (stage (c), decision 0014). |
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
`reference/SHA256SUMS` from the new files (`sha256sum`), and write a decision record (core 7.3). The inputs are a fixture: the T3 test builds its `RateConfig` (gains, D low-pass, tau_ref, period), its
`GyroChainConfig` and its plant from `rate_t3_inputs.txt` and does not read the live product parameters, so a legitimate parameter change (a B1 card
update, say) does not fail T3. The rotor geometry is left at its zero default (the mixer is out of the loop). The rule
that produces the gains is tested on the live parameters by `tools/test_rate_design.py`, and `from_params` /
`load_config` by `unit/rate/params_test.cpp`. CI (`ci/run_ci.sh`) regenerates the golden and the envelope from the
fixture and checks them against `reference/SHA256SUMS`, and shows that a perturbed input fails that check.

## Scenario

Per axis a, nominal J (`inertia_xx/yy/zz`) and tau (`motor_tau`): the setpoint steps from 0 to `rate_max_a` at
execution 1, the other axes are 0. Ticks are 625/4 us; the controller runs every `rate_loop_divisor` ticks with stamp
floor(tick x 625 / 4) us, so `dt` alternates 312 / 313 us against the design T = 312.5 us (both the oracle and the
harness use the stamps; the plant advances tick by tick, exactly 156.25 us each). Every tick the gyro sample is the
plant omega at that tick as a float with GyroValid (no sensor latency: this suite's convention, kept from L4; the IMU
latency belongs to the sensor model) and goes through the gyro chain as the rate-group step (`fw/rate_group`) runs it:
on a rate tick `update_notches` first, then `filter`. T3 has no rotor-speed sample, so `update_notches` gets every motor
invalid, every notch is bypassed (the identity stage, exact; the harness checks `bypass_flags`) and the chain is its
second-order Butterworth low-pass at `gyro_lpf_cutoff_hz`, seeded with the first sample (`seed_first_sample`, as
`chain_from_params` sets it). The rate loop runs on the chain output with the D term on the measurement through its
low-pass (`rate_d_filter_tau_a`); the feed-forward is off (J = 0), as in the product parameters. The torque is held until
the next execution (zero computation delay). Execution 1 seeds the prefilter from the gyro, so it outputs 0 and the step
is first seen by execution 2. Mixer out of the loop, `record_allocation` not called (the freeze flags stay false).

Horizon: N = 1 + ceil(10 tau_ref / T) executions (scenario test value 10: ten reference time constants). At the horizon
the roll response is 0.018 rad/s (0.16 %) from the setpoint (the integral mode is slower than tau_ref), after a peak
overshoot of 1.78 rad/s.

## Tolerance (derived)

Convention: `u = 2^-24` is the binary32 unit roundoff (round to nearest), epsilon = 2u = 2^-23; every float operation
returns `(1 + d) x exact` with `|d| <= u`. `expf` and `tanf` are assumed within 1 ulp of their results (`ulp32`;
INFERRED). The host build compiles with `-ffp-contract=off`; a fused operation would only remove roundings. The bound is
first order (terms of order u^2 dropped) and uses the double trajectory for the operand magnitudes.

The float chain and law have seven nodes where rounding enters; the injection bound `rho` of each, per tick for the
chain nodes x and f and per execution n >= 2 for the others, with `s = |sp - r_(n-1)|`, `s_d = |d_n - Df_(n-1)|`:

| Node | Float operations | rho |
| --- | --- | --- |
| x (chain input) | gyro cast `omega -> float` | `u abs(omega_t)` |
| f (chain low-pass output) | `y = b0 x + b1 x1 + b2 x2 - a1 y1 - a2 y2` left to right: five products, four sums (the bypassed notches are exact); the f32 coefficients | `u (sum abs(products) + sum abs(partial sums) + abs(y_t)) + db0 abs(x) + db1 abs(x1) + db2 abs(x2) + da1 abs(y1) + da2 abs(y2)` |
| r (prefilter) | `dt = fl(dt_us / 1e6)`, `x = fl(dt / tau_ref)`, `exp`, `alpha = 1 - exp` (exact by Sterbenz, u alpha kept), `fl(sp - r)`, `fl(alpha t)`, `fl(r + p)` | `d_alpha s + 2 u alpha s + u abs(r_n)`, `d_alpha = e^-x 2 u x + ulp32(e^-x) + u alpha` |
| e | `fl(r - y)` (y is the chain output, a float) | `u abs(e_n)` |
| I | `dt` (u), `fl(ki e)`, `fl(. dt)` (2 u), `fl(I + inc)` | `3 u abs(ki e_(n-1) dt_n) + u abs(I_n)` |
| D (D low-pass state Df) | `d = fl(fl(-kd fl(y - y_prev)) / dt)` (4 u with dt), `alpha_d = 1 - exp(-dt / T_f)` as alpha, `fl(d - Df)`, `fl(alpha_d t)`, `fl(Df + p)` | `alpha_d 4 u abs(d_n) + d_alpha_d s_d + 2 u alpha_d s_d + u abs(Df_n)`, `d_alpha_d` as `d_alpha` with `x = dt / T_f`; `4 u abs(d_n)` when T_f = 0 |
| u | `fl(fl(kp e) + I)`, `fl(. + Df)` | `u (abs(kp e_n) + abs(kp e_n + I_n) + abs(u_n))` |

The f32 coefficient bounds `db0 .. da2` (`lowpass_f32_error`, first order): `theta = fl(fl(pi_f f_c) h_f)` with `pi_f`
and the tick period `h_f` rounded once each, so `|dtheta| <= 4 u theta`; `dK = (1 + K^2) 4 u theta + ulp32(K)` (tanf);
then each operation of `lowpass_coeffs<float>` as written: `dK2 = 2 K dK + u K^2`, `drk = sqrt(2) dK + 2 u sqrt(2) K`
(sqrtf(2) correctly rounded), `dden = drk + u (1 + rk) + dK2 + u den`, `dnorm = norm (dden / den + u)`,
`db0 = norm dK2 + K^2 dnorm + u b0`, `db1 = 2 db0` (doubling is exact), `db2 = db0`,
`da1 = 2 (dK2 + u abs(K^2 - 1)) norm + 2 abs(K^2 - 1) dnorm + u abs(a1)`,
`da2 = (drk + u abs(1 - rk) + dK2 + u abs(q)) norm + abs(q) dnorm + u abs(a2)` with `q = 1 - rk + K^2`. A coefficient
error is systematic, but to first order its effect is the sum of its per-tick products with the operands, an injection
at f like the others. The golden's header records the double coefficients and these bounds.

`r` and `e` carry the setpoint-path and the measurement-path errors, `x` and `f` the chain's. An injection `w` at node
p moves `y_m` (later executions) by `g_p w`, where `g_p` is the impulse response of the closed loop (chain history,
prefilter state, integrator, D low-pass and stored error included) from that node to omega. The oracle computes `g_p`
for injections at executions 2 and 3 (the two dt phases; every later injection is a shifted copy of one of them) over
the horizon, and takes `l1_p = max over the two phases of sum abs(g_p)`; a chain node is injected at every tick, so its
`g_p` sum covers the D tick positions of the execution (D separate injections, their sums added). The tolerance is

    TOL_a = sum over p in (x, f, r, e, I, D, u) of l1_p max(rho_p)

The plant and the oracle are double: their rounding is 2^-29 of a float operation's and is not in the sum. The test
checks `TOL = sum(l1 x rho)` from the generated numbers (`ToleranceIsTheSumOfL1TimesRho`).

Result (2026-10-01, every axis): TOL = 7.896e-4 rad/s, of which r 4.45e-4, I 2.09e-4, f 1.10e-4, D 2.4e-5, the rest
below 2e-6. Observed float error 1.13e-5 rad/s (1.4 % of TOL); the actual f32 coefficient errors are 1 to 14 % of their
bounds.

## Negative controls (quad 4 L4 pass bar)

Each must leave the tolerance (asserted in `L4T3Step`): (a) every gain (kp, ki, kd) x 1.1 through a modified
`RateConfig`; (b) one tick of added delay between the controller output and the plant (the first tick of every
execution still applies the previous torque). T_f is a time constant, not a gain, and is not scaled. Result
(2026-10-01, every axis): (a) moves the trajectory 0.2232 rad/s (282.7 x TOL), (b) 3.034e-3 rad/s (3.8 x TOL).

## Band envelope (recorded, robustness only)

`rate_t3_envelope.txt`: per execution, min and max over J (1 +- inertia_robustness_band) x tau (1 +- tau_robustness_band)
of the same step response with the nominal-plant f32 gains, prefilter, chain low-pass and D low-pass; the 17 x 17 grid, which contains the corners
and the 9 x 9 grid, and `halving_max_change` is the largest change of any envelope point from the 9 x 9 grid to the
17 x 17 one. The test only asserts that the nominal float trajectory lies inside it.

The halving change is recorded per axis in the file (`halving_max_change`): roll 0.00521230, pitch 0.00521230, yaw
0.00521232 rad/s. T4 widens the envelope by it.
