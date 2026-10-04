# L5 T3: the attitude loop against a double oracle

Quad spec 4 L5, decision 0006 F ("T3 step", "T3 envelopes"); quad spec 4 L6 stage (c) pass bar "the L4 and L5 T3 goldens
regenerate with the D term" (decision 0014). The float firmware path (`AngleMode`, then `AttitudeLaw`, then the gyro chain and
`RateLoop::execute_bypass` with its filtered D term, as `fw/rate_group` composes them, each at its period) is closed with the
design plant in double at tick resolution, and its trajectories are compared with an independent double oracle within a
derived rounding tolerance.

## Files

| File | Role |
| --- | --- |
| `t3_test.cpp` | The harness and the assertions (gtest, label `frozen`). |
| `reference/attitude_t3_oracle.py` | The independent oracle (python3 stdlib only): law, plant, tolerance, band envelopes. |
| `reference/attitude_t3_inputs.txt` | The fixture: the f32 product values of 2026-10-01, L6 stage (c) (hex floats) plus `tau_held_yaw_nm`. |
| `reference/attitude_t3_q_inputs.txt` | The quantiser fixture (hex floats): the firmware mixer's f32 values, the scenario collective `l5_thrust_n`, the card's plant side in double. |
| `reference/SHA256SUMS` | The committed SHA-256 of the three generated files below (`sha256sum` format). |
| `attitude_t3_golden.txt` (generated, not committed) | Per script: the trajectory (angle, rate) per attitude execution, the node error bounds, the tolerances. |
| `attitude_t3_envelope.txt` (generated, not committed) | The band envelopes of the design model (for T4) and the fallback property. |
| `attitude_t3_q.txt` (generated, not committed) | The DShot quantisation term Q per script and channel (for T4). |

The golden, the envelope and Q are not committed (decision 0011). `tools/refdata/refdata.py` generates them from the committed
inputs and the oracle into `build/reference/quad/L05/t3/` (or `$MARV_REFERENCE_DIR`), together with copies of the inputs, and
checks them against `reference/SHA256SUMS`; the ctest fixture `marv_reference_quad_L05_t3_ensure` does it before `t3_l5_attitude`
runs, the Python consumers (`../tools/`, `../gz/`, `../results/`) call `refdata.reference_dir("quad/L05/t3")`, and a mismatch
fails naming the file and both hashes:

    uv run python tools/refdata/refdata.py ensure quad/L05/t3 [--procs N]

Generate them by hand (about 2 min on 16 CPUs, about 6 min on one; byte-identical on the host and in `marv-ci`, and
independent of `--procs`, which only spreads the Q runs over worker processes):

    uv run python tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py --procs 8 --dir <dir with the two inputs files>

Regenerate the quantiser fixture (when the card or a product mixer parameter changes, or at L6; it reads
`vehicles/uzh_neurobem_5in.yaml`, the `l5_thrust_n` rule of `tools/sim/run_l5.py` and the build's parameter table) and then Q:

    uv run python tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py --refresh-q-inputs \
        build/<preset>/generated/marv_params/marv/params/param_defaults.cpp

Regenerate the inputs from a built product parameter table (only when a product parameter changes; it imports
`tools/sim/run_l4.py` for `tau_held_yaw_nm`):

    uv run python tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py --refresh-inputs \
        build/<preset>/generated/marv_params/marv/params/param_defaults.cpp

The fixture equals the product parameter values of the commit that regenerated it (L6 stage (c), decision 0014: the PI x lead
rate gains with `rate_kd_*` and `rate_d_filter_tau_*`, the chain's `gyro_lpf_cutoff_hz`, `gyro_notch_*`, the attitude gain
and `att_yaw_t_cross` of the stage (c) rate loop; earlier att_loop_ratio = 1, owner decision 21, and 2026-09-30 at N = 2) and does not follow later changes (decision 0006, owner
decision 7: goldens only on fixed inputs the test owns; rule properties on the live parameters, in `../tools/`). The test
builds its `AttitudeConfig` and `RateConfig` from the fixture, not from the live parameters. Goldens are regenerated only
inside the CI image (CLAUDE.md). A changed golden means: regenerate in the image, update `reference/SHA256SUMS` from the new
files (`sha256sum`), and write a decision record (core 7.3). CI regenerates the golden, the envelope and Q from the fixtures
and checks them against `reference/SHA256SUMS`, and shows that a perturbed input fails that check.

## Harness

Tick `j` has the stamp `floor(j num / den)` us. The rate loop executes at `j = 0 mod D` (the seed execution at 0), the
attitude group at `j = 0 mod D N` and before the rate group of the same tick; its rate setpoint is held until the next
attitude execution, and the torque computed at tick `j` acts from tick `j` (zero computation delay). The rate group is the
composition of `fw/rate_group` (`rate_group.hpp`): every tick `GyroChain::filter` on the tick's sample, on a rate tick after
`update_notches`, and `RateLoop::execute_bypass` on the chain output. No rotor speed exists here (every valid bit clear), so
every notch is bypassed (the identity: its output is exactly its input) and the chain is its second-order Butterworth low-pass,
configured as `rate_group::chain_from_params` does (`seed_first_sample` set). The mixer and the allocation record (the
anti-windup) stay out of the loop, as before; the feed-forward is inert (J = 0), as in the product until stage (e). Per axis
the plant is `J w' = u_m`, `tau u_m' = u - u_m` with `u` held over a tick, exact per tick (ZOH closed forms), no `w x Jw`. The
body quaternion is integrated in double from the exact per-axis angle increments of each kinematic sub-step,
`q <- q (x) exp(dphi/2)`. The firmware sees `q` and `w` as float32 with `valid` set; the gyro sample of tick `j` is `w` at the
start of tick `j` as float32 with `GyroValid`, the convention of the harness before the chain (no sensor latency; the stage (c)
T4 flies the truth gyro, decision 0014 "Wiring").

## Scripts (scenario test values)

All start level and at rest. Attitude execution `a = 0` is the seed (sticks 0). The stick is 1 for `1 <= a < a_r`
(`a_r = 1 + H`) and 0 afterwards; the run has `1 + 2 H` executions.

- `step_roll`, `step_pitch`: full stick, then the release.
- `yaw_release`: a sustained full yaw stick, then the release (braking, the lock at the crossing, the pull-back to the lock).
- `yaw_fallback` (envelope only, for T4): the same script with the constant yaw torque `d = sigma_r tau_held,yaw` from the
  release execution on.

Segment length: `H = ceil(DUR / T_a)`, `DUR = 10 / k` seconds (ten attitude time constants), doubled (core 7.5) until every
envelope's end value is below `F`. The start value did not settle (3.17 s): the first doubling gives `DUR = 6.341 s`,
`H = 20292` executions (T_a = 312.5 us), 40585 executions per script. `F = T3 tolerance + the envelope's last-halving change + the kinematics
halving change` of the script; `F` is a term of the T4 tolerance `E + F` with `E >= 0`, so an envelope that ends below `F`
has settled below the T4 tolerance (for yaw: the yaw rate and the heading relative to the lock; the heading relative to the
release is then constant). The test asserts this on the generated files, and an envelope cut at the release fails it.

## Tolerance (derived)

Convention: `u = 2^-24` is the binary32 unit roundoff (round to nearest); every float operation returns
`(1 + d) x exact`, `|d| <= u`; each library call (`sin`, `cos`, `atan2`, `hypot`, `sqrt`, and from stage (c) `tan` and `exp`)
is within one ulp of its result (INFERRED for glibc, as `expf` was at L4; the test is the check). The bound is first order.

The oracle mirrors the float path operation by operation on an error-tracking number `E = (value, bound)`: the value is the
double computation, the bound propagates the operation errors and the input errors (the plant's double `q` and `w` cast to
float32, `u |x|`). Rounding nodes inject errors at every operation (`y` the chain output, `D = -kd (y - y_prev) / dt`):

| Node | Content | Bound |
| --- | --- | --- |
| `ref` | The attitude group: measurement cast, angle mode, law, `k/w` rounded once; the error of the held rate setpoint. | `E` bound of each attitude execution |
| `gyro` | The cast of `w` to float32 at the chain input, every tick. | `u abs(w)` |
| `lpf` | The low-pass's direct-form-I step at its output, every tick: five products and four sums, left to right (the bypassed notches are exact). | `u` times the sum of the magnitudes of the nine results |
| `e` | `fl(r - y)`. | `u abs(e)` |
| `I` | `dt = fl(dt_us / 1e6)`, `fl(ki e)`, `fl(. dt)`, `fl(I + inc)`. | `3 u abs(inc) + u abs(I)` |
| `D` | `fl(y - y_prev)`, `fl(-kd .)`, `fl(. / dt)` and the error of `dt` (`4 u abs(D)`), then `fl(Df + fl(alpha fl(D - Df)))`, into `Df`. | `alpha (4 u abs(D) + 2 u abs(D - Df)) + u abs(Df)` |
| `u` | `fl(kp e)`, `fl(. + I)`, `fl(. + Df)`. | `u (abs(kp e) + abs(kp e + I) + abs(u))` |

(The prefilter node of L4 does not exist in bypass.) An injection of size `rho_p(k)` at node `p` of tick or execution `k` moves
a channel at execution `m` by `g_p(m, k) rho_p(k)`, where `g` is the response of the linearised closed loop: per tick the ZOH
plant of the nominal plant and the chain's low-pass, per rate execution the law with the stamp `dt` of the integrator and of
the D path and its `alpha(dt)`, and the attitude feedback gain `k c` frozen at each of three values of `c` in `[c_min, 1]`
(`c = cos(e/2)` for tilt, `cos(w e/2)` for the yaw lock, `e` the trajectory's largest error; the largest bound over `c` is
kept). The error of a channel is at most `sum_p sum_k |g_p(m, k)| rho_p(k)`: every injection, every phase of the loop's period
(`L = lcm(N, 2)` rate executions, because the stamp `dt` alternates 312 / 313 us: `L D` tick phases for `gyro` and `lpf`, `L`
for the rate nodes, `L / N` attitude phases for `ref`), with `rho_p(k)` at its own injection (blocks of 512 injections take
their block maximum). The injections at the seed execution (no D, `y_prev` seeded) respond differently from the rest of their
phase, so each phase's response is taken one period later, and the oracle refuses unless every seed injection has a zero bound
(the scripts start at rest).

Two coefficient nodes are fixed offsets rather than per-operation roundings: a float coefficient computed once differs from the
exact one by one `delta`, `|delta| <=` its first-order bound, for the whole run. Its first-order effect at `m` is
`delta S(m)`, `S(m) = sum_k g(m, k) s(k)` the signed response of the linearised loop to the signal `s` the coefficient
multiplies, injected where the coefficient acts (one forward run per coefficient); the node's bound is `sum_i |delta_i| |S_i(m)|`,
its largest over `m`:

| Node | Coefficients | `s` | `delta` bound |
| --- | --- | --- | --- |
| `lpf_coef` | the low-pass `b0` (`b1 = 2 b0`, `b2 = b0` exactly), `a1`, `a2`: the period of `rate_group::chain_from_params` (two roundings), then `lowpass_coeffs<float>` (`pi` cast, `tanf`, `sqrtf`) | `x + 2 x1 + x2`, `-y1`, `-y2` | the `E` bound of each coefficient |
| `alpha` | `alpha_d = fl(1 - expf(fl(-dt / T_f)))` at each of the two stamp spacings | `D - Df` at the executions with that spacing | `exp(-x) 2 u x + ulp(exp(-x)) + u alpha`, `x = dt / T_f` |

The tolerance of a channel is the sum over all nodes of the largest such bound over `m`; the golden stores each node's bound
(`err_<channel>_<node>`) and `ToleranceIsTheSumOfTheNodeErrorBounds` checks the sum.
Before the yaw lock the attitude loop is open on yaw (the heading setpoint tracks the heading); at the lock the regime
changes, and the responses are composed through the componentwise bound of the error state at the lock (weighted by
`rho_p(k)`), with the copied lock heading as an added state. So the accumulated heading error of the hold is carried exactly,
and the injection bound of a short event (the braking's large integrator) weighs only that event. An earlier version used
the largest `rho` over the whole run times the l1 norm: its yaw tolerances were 9.7e-3 rad/s and 1.6e-3 rad, and the
one-tick-delay control cleared the yaw tolerance by x1.1; the localised bound is rigorous and gives the table below.

Assumptions not covered by the bound: the feedback gain is frozen per `c` (it varies slowly against the loop's time
constants); the lock happens at the same execution in the float and double runs (the lock's decision margin, the yaw rate on
both sides of the crossing, is 3.09e-3 rad/s (the side at the crossing; 4.45e-3 before) against a rate tolerance of 1.89e-3 rad/s plus the cast bound, asserted by
`YawLockIsAtTheGoldenExecution...`). Off-axis rate setpoints have a zero bound (the oracle refuses otherwise).

| Script | tolerance theta (rad) | tolerance omega (rad/s) | observed max theta | observed max omega |
| --- | --- | --- | --- | --- |
| `step_roll` | 2.56e-5 | 9.26e-5 | 1.66e-7 (0.65 %) | 7.1e-7 (0.77 %) |
| `step_pitch` | 2.56e-5 | 9.26e-5 | 1.33e-7 (0.52 %) | 5.1e-7 (0.55 %) |
| `yaw_release` | 8.86e-4 | 1.89e-3 | 4.9e-6 (0.55 %) | 3.5e-6 (0.19 %) |

Largest node contributions (stage (c)): tilt theta `I` 1.38e-5, `ref` 4.7e-6, `lpf_coef` 4.0e-6; yaw heading `lpf_coef`
4.4e-4 (before the lock the heading integrates the yaw rate, so the low-pass's DC-gain offset accumulates over the sustained
stick), `I` 2.0e-4, `ref` 1.4e-4, `lpf` 9.3e-5. The values at L5 (PI, no chain) were 1.78e-5 / 6.78e-5 (tilt) and
2.80e-4 / 1.27e-3 (yaw).

## Kinematics (core 7.5)

The golden uses 1 sub-step per tick; the oracle and the test also run 2 and require the trajectory change below the
tolerance: the largest change is 4.6e-14 rad (tilt) and 1.3e-11 rad (yaw). The scripts are single-axis, so the increments
commute and the change is rounding only; the check stays for scripts that are not.

## Negative controls (quad spec 4 L5 pass bar)

Each must leave the tolerance (largest of `max|diff| / tolerance` over the two channels):

| Control | `step_roll` | `step_pitch` | `yaw_release` |
| --- | --- | --- | --- |
| attitude gain `k` x 1.1 | x2465 | x2465 | x70.7 |
| rate gains (kp, ki, kd) x 1.1 (extra) | x1138 | x1138 | x206.3 |
| one tick of delay between the controller output and the plant | x28.1 | x28.1 | x5.1 |

The perturbed-input control in CI changes `att_kp` by one f32 ulp in a copy of the fixture: the golden must differ.

## Band envelopes (recorded, for T4)

`attitude_t3_envelope.txt`: per script and per attitude execution, the min / max over `J (1 +- inertia_robustness_band)` x
`tau (1 +- tau_robustness_band)` (17 x 17 grid, which contains the corners and the 9 x 9 grid) of the design model: the exact
sampled-data rate loop at tick resolution (bypass, nominal f32 gains, integrator and D step `T`, `alpha = 1 - exp(-T / T_f)`,
the chain's low-pass with exact coefficients and the notches bypassed: the stage (c) T4 configuration, decision 0014), with the
law in its single-axis closed form (tilt `r = 2 k sin(e/2)`; yaw lock `r = (k/w) 2 sin(w e/2)`). It is driven by the exact script from the exact initial
state. The yaw scripts include the release logic of decision 0006 D (braking, the crossing, the fallback with `att_yaw_t_cross`
and `att_yaw_alpha_min`, the lock), each member's headings relative to its own release and its own lock; the lock-execution
range is recorded. Values are rounded outward to 7 significant digits. `halving_max_change` is the largest change of any
envelope point from the 9 x 9 grid to the 17 x 17 grid, the envelope's last-halving change of the T4 tolerance `F`:

| Channel | halving max change |
| --- | --- |
| `step_roll` / `step_pitch` tilt angle | 5.98e-4 rad |
| `yaw_release` yaw rate | 8.65e-3 rad/s |
| `yaw_release` heading relative to the release / to the lock | 1.21e-3 / 1.25e-3 rad |
| `yaw_fallback` yaw rate | 7.28e-3 rad/s |
| `yaw_fallback` heading relative to the release / to the lock | 1.53e-3 / 1.03e-3 rad |

The locks of `yaw_release` fall in executions 20910..21229 (the crossing), before the fallback at 21230; in `yaw_fallback`
every member locks at the fallback execution 21230. `att_yaw_t_cross` is the latest crossing over the box corners and every
chain configuration (`tools/card/attitude_lead.py` rule step 11', lead decision of stage (c) commit 3); with the step-1'
configuration alone (0.274375 s) the latest members of this envelope (J+, tau-) lock at the fallback before their crossing.

## The fallback property (T3, decision 0006 F)

With the same script and `d = sigma_r tau_held,yaw` (0.1635 N m, the collective-held yaw torque envelope of 0005's chirp
rule), every box member keeps `sigma_r w > 0` from the release up to its fallback execution. If some member crossed, the held
stick would halve (core 7.5) until none does; the recorded `stick_scale` is 1 (full stick suffices), and the smallest
`sigma_r w` over the members is +0.904 rad/s. The test asserts it on the generated envelope, with the release script (no
disturbance, smallest `sigma_r w` = -1.451 rad/s, it crosses) as the control. The same property on the live parameters is
checked in `../tools/`.

## The quantisation term Q (owner decision 19, for T4)

The design model has a continuous torque input; the flown loop writes each motor's thrust as a DShot step. Since decision 0017
the rate group's `mixer::DshotDiffuser` does that: a per-motor error carry makes the mean applied command follow the request.
T4 checks gz within `envelope +- (E + F + Q)`; the T3 envelope and its F are unchanged. `Q` is computed by rule, not fitted to Gazebo, and
regenerates with the card (or L6):

`Q = max over the time and over the tau x J box of |quantised - unquantised|` of the design-model trajectory for the same script,
per channel the T4 predicates use (tilt scripts: angle and rate; yaw scripts: the rate and the headings relative to the release
and to the lock, each member against its own release and lock). The unquantised model is the envelope's. The quantised model
(`Quantiser` in the oracle, binary64, with the diffuser in the loop: `diffusion=True`, the default) puts between the rate loop's torque request and the design plant's torque lag:

1. `mixer::allocate` at the scenario collective `l5_thrust_n` (`fw/mixer/include/marv/mixer/mixer.hpp:145`), the request on the
   script's axis and the other two axes 0;
2. `mixer::DshotDiffuser` (`fw/mixer/include/marv/mixer/mixer.hpp`, decision 0017), once per rate execution and per motor, with
   every carry 0 at the start of the script run: `omega = sqrt(f / k)`, `d*` = the linear-in-omega ESC map inverted
   (`dshot_unrounded`); `d~ = f32(d*)` clamped to `[ceil(D(omega_idle)), 2047]`; `u = f32(d~ + e)`; the DShot `q` = `u` rounded to
   nearest (halves away from zero, `std::round`) and clamped to the same range; the carry `e = u - q`, exact. Clamping before the
   rounding keeps `|e| <= 1/2` and nothing saturated is carried. With `diffusion=False` the quantiser is
   `mixer::thrust_to_dshot`'s stateless rounding (`d*` rounded, clamped), the negative control of the diffusion and the rounding
   the dead bands below describe;
3. marv_plant's ESC map (`sim/plant/src/plant_model.hpp:47`, `Model::omega_cmd`: DShot 0 gives 0, 48..2047 map linearly to the
   card's `speed_range`), thrust `k_card omega^2`, the card's rotor geometry and yaw reaction, projected on the script's axis.

Its ESC map, thrust and torque are the same as the cause diagnosis's (`../results/step_cause/`, `step_cause_tool.cpp` `omega_cmd`
and `body_torque`), in the design model instead of the float path. The stateless path keeps the dead-band facts: hover DShot
765.06 gives 765, the request dead bands are 8.03e-4 (roll), 6.02e-4 (pitch) and 1.77e-4 (yaw) N m, one DShot step at hover is
4.562e-3 N (`../tools/test_attitude_t3_q.py` checks them, with `diffusion=False`, against `cause.txt`). The diffuser has no dead
band: a request inside one step moves the mean applied command with it. Off-axis torque of the rounding is not carried (the
design model is single-axis).

Evaluation: at the four corners of the box plus the nominal (`q_corners`), then over the 9 x 9 grid that contains them
(`q_grid`). `q` is `q_grid` where the grid exceeds the corners (`rule grid`), else `q_corners`. The rounding is not monotone in
the parameters, so the corners do not bound it: in 8 of the 10 channels the grid maximum exceeds the corner maximum (by 18 % to
131 %), and there `q` is the grid maximum. The recorded `saturated_executions` counts member x rate executions in which the
allocation scaled the request or moved the collective (none in the tilt scripts); for the yaw scripts `q_allocation_only` is the
same comparison with the DShot command left out (at most 1.8e-7): the allocation's effect is small against the diffuser's.

| Script | theta (rad) | omega (rad/s) | heading to the release (rad) | heading to the lock (rad) |
| --- | --- | --- | --- | --- |
| `step_roll` | 2.98e-6 | 6.01e-5 | - | - |
| `step_pitch` | 1.79e-6 | 5.35e-5 | - | - |
| `yaw_release` | - | 1.60e-5 | 1.14e-6 | 5.91e-7 |
| `yaw_fallback` | - | 1.53e-5 | 1.19e-6 | 6.06e-7 |

With the stateless rounding (before decision 0017) Q was 7.80e-3 / 3.67e-2 (`step_roll`), 7.03e-3 / 3.31e-2 (`step_pitch`),
4.78e-3 / 1.45e-3 / 9.85e-4 (`yaw_release`) and 3.48e-3 / 1.09e-3 / 6.23e-4 (`yaw_fallback`): the diffuser makes Q smaller by a
factor of 227 to 3921 per channel.

Checks: `t3_test.cpp` `L5T3Quantisation` (Q recorded, finite and positive for every script and channel); the control in
`../tools/test_attitude_t3_q.py` (the quantiser disabled, the identity map, gives Q = 0 exactly; enabled it gives Q > 0). The measured
gz excursions of the step cause diagnosis (roll +6.52e-3 and pitch +5.94e-3 rad outside the envelope, against the L5 Q theta of
8.28e-3 and 7.40e-3) belong to the PI loop before stage (c); the step_cause re-run on this envelope is decision 0014's commit 4.
