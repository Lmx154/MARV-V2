# L5 T3: the attitude loop against a double oracle

Quad spec 4 L5, decision 0006 F ("T3 step", "T3 envelopes"). The float firmware path (`AngleMode`, then `AttitudeLaw`, then
`RateLoop::execute_bypass`, each at its period) is closed with the design plant in double at tick resolution, and its
trajectories are compared with an independent double oracle within a derived rounding tolerance.

## Files

| File | Role |
| --- | --- |
| `t3_test.cpp` | The harness and the assertions (gtest, label `frozen`). |
| `reference/attitude_t3_oracle.py` | The independent oracle (python3 stdlib only): law, plant, tolerance, band envelopes. |
| `reference/attitude_t3_inputs.txt` | The fixture: the f32 product values of 2026-09-30 (hex floats) plus `tau_held_yaw_nm`. |
| `reference/attitude_t3_golden.txt` | Per script: the trajectory (angle, rate) per attitude execution, the node error bounds, the tolerances. |
| `reference/attitude_t3_envelope.txt` | The band envelopes of the design model (for T4) and the fallback property. |

Regenerate the golden and the envelope (about 1 min on the host; byte-identical on the host and in `marv-ci`):

    uv run python tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py

Regenerate the inputs from a built product parameter table (only when a product parameter changes; it imports
`tools/sim/run_l4.py` for `tau_held_yaw_nm`):

    uv run python tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py --refresh-inputs \
        build/<preset>/generated/marv_params/marv/params/param_defaults.cpp

The fixture equals the product parameter values of 2026-09-30 and does not follow later changes (decision 0006, owner
decision 7: goldens only on fixed inputs the test owns; rule properties on the live parameters, in `../tools/`). The test
builds its `AttitudeConfig` and `RateConfig` from the fixture, not from the live parameters. Goldens are regenerated only
inside the CI image (CLAUDE.md), and a change under `tests/regression/` needs a decision record (core 7.3). CI regenerates
the golden and the envelope from the fixture and compares them byte for byte, and shows that a perturbed input does not
reproduce them.

## Harness

Tick `j` has the stamp `floor(j num / den)` us. The rate loop executes at `j = 0 mod D` (the seed execution at 0), the
attitude group at `j = 0 mod D N` and before the rate group of the same tick; its rate setpoint is held until the next
attitude execution, and the torque computed at tick `j` acts from tick `j` (zero computation delay). Per axis the plant is
`J w' = u_m`, `tau u_m' = u - u_m` with `u` held over a tick, exact per tick (ZOH closed forms), no `w x Jw`. The body
quaternion is integrated in double from the exact per-axis angle increments of each kinematic sub-step, `q <- q (x) exp(dphi/2)`.
The firmware sees `q` and `w` as float32 with `valid` set; the gyro is `w` as float32 with `GyroValid`.

## Scripts (scenario test values)

All start level and at rest. Attitude execution `a = 0` is the seed (sticks 0). The stick is 1 for `1 <= a < a_r`
(`a_r = 1 + H`) and 0 afterwards; the run has `1 + 2 H` executions.

- `step_roll`, `step_pitch`: full stick, then the release.
- `yaw_release`: a sustained full yaw stick, then the release (braking, the lock at the crossing, the pull-back to the lock).
- `yaw_fallback` (envelope only, for T4): the same script with the constant yaw torque `d = sigma_r tau_held,yaw` from the
  release execution on.

Segment length: `H = ceil(DUR / T_a)`, `DUR = 10 / k` seconds (ten attitude time constants), doubled (core 7.5) until every
envelope's end value is below `F`. The start value did not settle (3.24 s): the first doubling gives `DUR = 6.48 s`,
`H = 10372` executions, 20745 executions per script. `F = T3 tolerance + the envelope's last-halving change + the kinematics
halving change` of the script; `F` is a term of the T4 tolerance `E + F` with `E >= 0`, so an envelope that ends below `F`
has settled below the T4 tolerance (for yaw: the yaw rate and the heading relative to the lock; the heading relative to the
release is then constant). The test asserts this on the committed files, and an envelope cut at the release fails it.

## Tolerance (derived)

Convention: `u = 2^-24` is the binary32 unit roundoff (round to nearest); every float operation returns
`(1 + d) x exact`, `|d| <= u`; each library call (`sin`, `cos`, `atan2`, `hypot`, `sqrt`) is within one ulp of its result
(INFERRED for glibc, as `expf` was at L4; the test is the check). The bound is first order.

The oracle mirrors the float path operation by operation on an error-tracking number `E = (value, bound)`: the value is the
double computation, the bound propagates the operation errors and the input errors (the plant's double `q` and `w` cast to
float32, `u |x|`). Two families of nodes inject errors:

| Node | Content | Bound |
| --- | --- | --- |
| `ref` | The attitude group: measurement cast, angle mode, law, `k/w` rounded once; the error of the held rate setpoint. | `E` bound of each attitude execution |
| `e` | Gyro cast and `fl(r - y)`. | `u (abs(w) + abs(e))` |
| `I` | `dt = fl(dt_us / 1e6)`, `fl(ki e)`, `fl(. dt)`, `fl(I + inc)`. | `3 u abs(inc) + u abs(I)` |
| `u` | `fl(kp e)`, `fl(. + I)` (`kd = 0`, so `+ d` is exact). | `u abs(kp e) + u abs(u)` |

(The prefilter node of L4 does not exist in bypass.) An injection of size `rho_p(k)` at node `p` of execution `k` moves a
channel at execution `m` by `g_p(m, k) rho_p(k)`, where `g` is the response of the linearised closed loop: the per-execution
ZOH rate loop of the nominal plant with the integrator's stamp `dt`, and the attitude feedback gain `k c` frozen at each of
three values of `c` in `[c_min, 1]` (`c = cos(e/2)` for tilt, `cos(w e/2)` for the yaw lock, `e` the trajectory's largest error;
the largest bound over `c` is kept). The error of a channel is at most `sum_p sum_k |g_p(m, k)| rho_p(k)`: every injection
execution, every phase of the rate executions within an attitude period, with `rho_p(k)` at its own execution (blocks of 512
injections take their block maximum). The tolerance of a channel is the sum over the nodes of the largest such bound over
`m`; the golden stores each node's bound (`err_<channel>_<node>`) and `ToleranceIsTheSumOfTheNodeErrorBounds` checks the sum.
Before the yaw lock the attitude loop is open on yaw (the heading setpoint tracks the heading); at the lock the regime
changes, and the responses are composed through the componentwise bound of the error state at the lock (weighted by
`rho_p(k)`), with the copied lock heading as an added state. So the accumulated heading error of the hold is carried exactly,
and the injection bound of a short event (the braking's large integrator) weighs only that event. An earlier version used
the largest `rho` over the whole run times the l1 norm: its yaw tolerances were 9.7e-3 rad/s and 1.6e-3 rad, and the
one-tick-delay control cleared the yaw tolerance by x1.1; the localised bound is rigorous and gives the table below.

Assumptions not covered by the bound: the feedback gain is frozen per `c` (it varies slowly against the loop's time
constants); the lock happens at the same execution in the float and double runs (the lock's decision margin, the yaw rate on
both sides of the crossing, is 1.15e-2 rad/s against a rate tolerance of 1.27e-3 rad/s plus the cast bound, asserted by
`YawLockIsAtTheGoldenExecution...`). Off-axis rate setpoints have a zero bound (the oracle refuses otherwise).

| Script | tolerance theta (rad) | tolerance omega (rad/s) | observed max theta | observed max omega |
| --- | --- | --- | --- | --- |
| `step_roll` | 1.82e-5 | 7.08e-5 | 1.47e-7 (0.81 %) | 6.5e-7 (0.92 %) |
| `step_pitch` | 1.82e-5 | 7.08e-5 | 8.4e-8 (0.46 %) | 3.8e-7 (0.54 %) |
| `yaw_release` | 2.86e-4 | 1.27e-3 | 2.3e-6 (0.79 %) | 3.4e-6 (0.26 %) |

## Kinematics (core 7.5)

The golden uses 1 sub-step per tick; the oracle and the test also run 2 and require the trajectory change below the
tolerance: the largest change is 1.7e-13 rad (tilt) and 1.7e-11 rad (yaw). The scripts are single-axis, so the increments
commute and the change is rounding only; the check stays for scripts that are not.

## Negative controls (quad spec 4 L5 pass bar)

Each must leave the tolerance (largest of `max|diff| / tolerance` over the two channels):

| Control | `step_roll` | `step_pitch` | `yaw_release` |
| --- | --- | --- | --- |
| attitude gain `k` x 1.1 | x3782 | x3782 | x188 |
| rate gains x 1.1 (extra) | x3199 | x3199 | x490 |
| one tick of delay between the controller output and the plant | x44.4 | x44.4 | x8.2 |

The perturbed-input control in CI changes `att_kp` by one f32 ulp in a copy of the fixture: the golden must differ.

## Band envelopes (recorded, for T4)

`attitude_t3_envelope.txt`: per script and per attitude execution, the min / max over `J (1 +- inertia_robustness_band)` x
`tau (1 +- tau_robustness_band)` (17 x 17 grid, which contains the corners and the 9 x 9 grid) of the design model: the exact
sampled-data rate loop (bypass, nominal f32 gains, integrator step `T`) lifted to `T_a`, with the law in its single-axis closed
form (tilt `r = 2 k sin(e/2)`; yaw lock `r = (k/w) 2 sin(w e/2)`). It is driven by the exact script from the exact initial
state. The yaw scripts include the release logic of decision 0006 D (braking, the crossing, the fallback with `att_yaw_t_cross`
and `att_yaw_alpha_min`, the lock), each member's headings relative to its own release and its own lock; the lock-execution
range is recorded. Values are rounded outward to 7 significant digits. `halving_max_change` is the largest change of any
envelope point from the 9 x 9 grid to the 17 x 17 grid, the envelope's last-halving change of the T4 tolerance `F`:

| Channel | halving max change |
| --- | --- |
| `step_roll` / `step_pitch` tilt angle | 2.91e-3 rad |
| `yaw_release` yaw rate | 0.127 rad/s |
| `yaw_release` heading relative to the release / to the lock | 6.7e-3 / 8.9e-3 rad |
| `yaw_fallback` yaw rate | 0.0857 rad/s |
| `yaw_fallback` heading relative to the release / to the lock | 4.4e-3 / 3.3e-3 rad |

The locks of `yaw_release` fall in executions 10551..10783 (the crossing), before the fallback at 10784; in `yaw_fallback`
every member locks at the fallback execution 10784.

## The fallback property (T3, decision 0006 F)

With the same script and `d = sigma_r tau_held,yaw` (0.1635 N m, the collective-held yaw torque envelope of 0005's chirp
rule), every box member keeps `sigma_r w > 0` from the release up to its fallback execution. If some member crossed, the held
stick would halve (core 7.5) until none does; the recorded `stick_scale` is 1 (full stick suffices), and the smallest
`sigma_r w` over the members is +0.0975 rad/s. The test asserts it on the committed envelope, with the release script (no
disturbance, smallest `sigma_r w` = -3.58 rad/s, it crosses) as the control. The same property on the live parameters is
checked in `../tools/`.
