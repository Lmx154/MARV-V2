# ESKF in shadow: design, T2 beside Mahony, gravity-noise change (M4, decision 0055)

SHADOW ONLY. Mahony flies (decision 0042). Whether the board runs the ESKF at all, and whether it ever controls, is Luis's decision
after this comparison. Evidence, not a flight result: the T2 log is a still bench log, so vibration and linear acceleration are not in it.

## Design (fw/estimator/include/marv/estimator/eskf.hpp, the header holds the derivation)
Nominal q and gyro bias b; 6-state error [dtheta (NED), db]; Sola 2017 error-state equations; EKF2-style (PX4 ddcaad66, read 2026-10-18,
INFERRED) sequential scalar fusion, Joseph form with symmetric write-back, per-state variance floors/ceilings, innovation gate, reset on
rejection timeout. Gravity is fused once per 75-sample window (EKF2_PREDICT_US 10 ms) from the time-weighted mean specific force.
Parameters: `fw/estimator/params/eskf_register.yaml` (12 entries, provenance in each); set `marv_params_fc3v2_eskf` (= FC3v2 + register).
Tests: `unit_eskf` (T1, 8 tests), `unit_l07_eskf_replay` (T2 beside Mahony on the committed log, same bar, controls).

## The bias leak path and its fix (variants: the engineer's diagnostic runs, same T2 log)
Defect: level and still, b_z walked to 5 deg/s in 47 s (same in double: not rounding). Path: the covariance prediction and the reset
Jacobian G ([dtheta/2]x terms) correlate the large heading variance with the tilt, so every tilt residual moved the heading; the heading
is correlated (about -1) with the bias along gravity b.d through -R db dt; each update also linearises at a slightly different d. The data
holds no information on heading or on b.d, yet the filter took it. Fix: every scalar update zeroes the heading gain and removes the gain's
component along the body down direction d (EKF2 clearInhibitedStateKalmanGains generalised to the one unobservable direction); the Joseph
form still holds. Second cause: EKF2's switch-on gyro-bias sigma (0.1 rad/s) is wrong after the boot calibration; replaced by the
calibration's residual 5.78e-5 rad/s (derived, PLACEHOLDER as its bias instability). Variants (the diagnostic runs, first 80 lines of variants.txt read: with and without each inhibition, bias sigma 0.1 vs 5.78e-5, window 1 vs
75): at sigma 0.1 most variants fail (a) or (c') on this log (for example window 75 without the heading inhibition: (a) err2 6.5e-2 rad); at
sigma 5.78e-5 and window 75 the variants without either inhibition also pass T2 on this 70 s log (their wander is slower than the log), so T2
alone does not discriminate the inhibitions; the T1 defect case does. T1 pins it
(`StillAndNoisyTheBiasAlongGravityIsNotLearned`, fails without the projection: 9.9e-6 against 3.0e-6 rad/s).

## Gravity-measurement model error (this change)
The averaged-accelerometer variance was sensor noise only, (N_a/g)^2/(2T): sigma 4.2e-4 rad, 5 sigma = 0.12 deg, so after a manoeuvre the
accelerometer was gated until the 7 s timeout, and flight vibration or linear acceleration would lock it out. EKF2 sets each gravity
component's variance to max(EKF2_GRAV_NOISE^2, 0.01^2) (aid_sources/gravity/gravity_fusion.cpp:53), EKF2_GRAV_NOISE default 1.0 m/s^2
(common.h:483), without dividing by g although the measurement is the unit vector (and gates at 0.25, line 81 - a different gate scale from
ours). Rule applied: a specific-force error e (m/s^2) turns the direction by e/|f|, |f| = g, so the added variance is (1/g)^2 = 1.040e-2 rad^2
(sigma 0.102 rad), parameter `eskf_accel_model_noise` = 1.0 m/s^2 (scenario value from EKF2). 5 sigma of the total is 0.51 rad (29 deg). The
value comes from EKF2's default and the rule; nothing was tuned to the log. T1: first-window posterior against the scalar update with the
added variance (`AtRestLevelConvergesToTheScalarSteadyState`, part 1b). Consequence to note: with R this large the accelerometer corrects
tilt slowly (about sqrt(Q/R) per window); the gyro dominates between fusions, as in EKF2.

## T2 on `t2_hand_rotation_2026-10-16.bin`: Mahony and ESKF side by side (raw: `t2_before_gravity_noise.txt`, `t2_after_gravity_noise.txt`)
Bars: (a) err <= 0.02090 rad; (c') err <= 1.20361 rad; each control must fail.
| | Mahony | ESKF before | ESKF after (gravity noise) |
| --- | --- | --- | --- |
| (a) err still 1 / still 2, rad | 2.372e-4 / 4.780e-4 pass | 1.268e-4 / 3.729e-4 pass | 3.103e-4 / 2.453e-3 pass |
| (c') err, rad | 1.640e-2 pass | 1.725e-2 pass | 1.659e-2 pass |
| settle in still 2, s | 0.347 | 0.374 | 0.325 |
| control (a) (still-2 accel negated), rad | - | 1.728 FAIL (as required) | 1.729 FAIL |
| control (c') (rotation gyro negated), rad | - | 1.462 FAIL | 1.462 FAIL |
| fused / gated / tilt resets | - | 6217 / 1572 / 1 | 7785 / 4 / 0 |
Margin: (a) still 2 at 2.5e-3 of 0.0209 rad (8.5x); the (a) error grew from 3.7e-4 to 2.5e-3 rad because the filter trusts the accelerometer less
(still within the bar). Bias at the end of still 2 after the change: (-0.021, -0.005, -0.004) deg/s (was (-0.003, 0.010, 0.013)). The Mahony
column is the frozen replay's number; the test also requires the generic path to reproduce it exactly.

## Cost (ESTIMATE, not measured on the board)
Predict about 310 flops per sample; fusion (three scalar updates, Joseph form) about 2200 flops every 75 samples. At an assumed 2-3 cycles
per flop on the M33 FPU (loads, stores, no hand scheduling) and 150 MHz: predict about 4-6 us per tick; a fusion tick adds about 30-45 us. The
tick is 133.3 us and EMB-3 measured U = 0.699 (about 93 us), so predict raises U to about 0.75 and a fusion tick may reach or exceed the
tick period (93 + 6 + 45 = 144 us). Options if the board is to run it: spread the three scalar updates over three ticks, or propagate the
covariance at the window rate (EKF2's own structure: one covariance prediction per 10 ms) and only the nominal state per sample. To be
measured on the board before any decision (EMB-3 method).

## SIL library and Gazebo
`marv_sil_quad_fc_eskf` (quad_fc.cpp under MARV_QUAD_FC_ESKF, the ESKF replacing Mahony for the attitude law and upright check, Mahony still
stepped) builds cleanly. Board unchanged: `-E -P` of quad_fc.cpp with the host board flags is byte-identical before and after (sha256
6526980a815b4f691f799c0fecabc40d01a3fe0091a231d379aea004a724249f); marv_sil_quad_fc's link line does not list the ESKF composition or set. The
Gazebo comparison (S2e with the ESKF library against `../../L10/results/s2e_drag/README.md`) was NOT run: the plugin links its SIL library at
build time, so it needs a second plugin build, `cmake --preset host-gz -B build/host-gz-quadfc-eskf -DMARV_GZ_SIL=marv_sil_quad_fc_eskf
-DMARV_GZ_PARAMS=marv_params_fc3v2_eskf`, then `tools/sim/run_quad_fc.py --plugin-dir build/host-gz-quadfc-eskf/sim/gz/plugin`.
