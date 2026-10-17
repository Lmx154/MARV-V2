# 0042: M3 attitude estimator: tilt budget share, bench replay, plant contact input, hover predicate, estimator validity

## What changed

Architect ruling for quad §4.0 M3 (2026-10-16); this record implements nothing. Items marked (Luis) wait for him.

1. Register: `attitude_tilt_error_max` (rad, design-budget), the angle between the estimate's down direction in body
   axes and the reference's; heading excluded (no magnetometer). PLACEHOLDER 0.0209 rad = `t4_settle_fraction` x
   `angle_tilt_max` until Luis rules; it blocks M4's angle-mode flight, not M3's commits. (Luis) Amended (lead, M3
   P3b): the entry is in `design/budget.yaml`, so the product id set gains it:
   `tests/regression/quad/L06/product_params/marv_params_product_table.txt` (regenerated in the CI image, identical to
   the host build) and the new manifest `tests/regression/quad/L07/param_ids`.
2. T2 bench replay, new in `tests/regression/quad/L07/`. The committed log holds every ImuSample the estimator consumes on
   the target, with no deadline miss, FIFO overflow or stamp gap. Protocol: still T_s, a hand rotation >= pi/2 about a
   horizontal axis within 10 s, still T_s; T_s = ln(theta_rot / share) / Kp at the gain that flies. The firmware
   estimator replays it (host, float, dt from stamps, no golden). (a) At the end of each still segment the estimate's
   down direction is within the share of -mean(f)/|mean(f)|; a still sample has every gyro axis below 15 deg/s and |f| in
   (0.9 g, 1.1 g) (Betaflight 4.5.1 gates). (b) From the estimate at the end of still 1, the estimator with the
   accelerometer correction off lands within theta_rot / 2 of still 2's direction. Controls: (a) fails with still 2's
   dominant accel axis negated in the estimator input only; (b) fails with one rotated gyro axis negated.
3. Plant: `marv_plant` takes an opt-in host contact force (NED) used only by the IMU specific force,
   f = (thrust along -z_FRD + R^T F_contact) / m, never by `marv_plant_step`; absent, every output is byte-identical.
   `tests/regression/quad/L06/imu/kat_test.cpp` gains at-rest cases (level reads (0, 0, -g); tilted reads R^T(0, 0, -g));
   existing cases unchanged. The plugin passes nothing; Gazebo ground contact joins the deferred list. (Luis)
4. Hover predicate: a new gz test in `tests/regression/quad/L07/`: L5's angle mode and attitude law with the estimator in
   authority on the model IMU at the nominal corner (no turn-on bias; no `<attitude_source>`, no TRUTH_STATE, preset
   `host-gz-l7`), from level at hover rotor speed with a body-rate kick [p0, p0, 0] (a pure roll kick sits on the
   intermediate-axis separatrix), sticks centred, L5's horizon; truth tilt from the log's STEP body records. S on tilt:
   |tilt| <= s Delta + E + `attitude_tilt_error_max` at the last execution; heading not checked. Control, the loop
   disabled: `att_kp` = 0 and `rate_ki_roll` = `rate_ki_pitch` = 0, every gain that returns tilt to level (as 0032's
   all-gains control); `att_kp` = 0 alone passes, because the rate integral restores the kick's tilt. The test shows the
   loop stays stable and level with the estimate in authority; estimate accuracy and the gyro sign are T1's and item 2's.
   `marv_sil_l5_attitude_scripted` and its compile closure do not change. (Amended by the architect after the P4
   measurements.)
5. Validity and arming: valid iff q was initialised from a sample with |f| in (0.9 g, 1.1 g) (one sample; no window) and is finite and unit.
   `ArmingInputs::estimator_valid` (default false); `ArmState::EstimatorInvalid` appended after `ArmedFailsafeCentred`;
   refusal order ImuStale, MotorTest, Failsafe, NoLink, ImuInvalid, Saturated, EstimatorInvalid, ThrottleUp, SwitchOff /
   SwitchNotCycled; it refuses and never disarms; the estimator steps before the arming step.
   `tests/regression/quad/L08/unit/arming_test.cpp`: the rig sets `estimator_valid = true`; one case added.
   `tests/regression/quad/L09/unit/quad_fc_gate_test.cpp` feeds (0, 0, -9.8) m/s^2; its cases do not change. Amended
   (lead, M3 P2 fix): it gains one case, samples whose GyroValid / AccelValid flags are clear are not fed to the
   estimator (gyro clear: no step, dt spans from the last stepped stamp; accel clear: no correction). Amended
   (lead, M3 P6, boot gyro calibration and the upright check): `ArmedGateTest::SetUp` runs a warm-up of quiet samples
   through the calibration (1.25 s) before every existing armed-gate case; `step_tick` gains accel and reference-bias
   plumbing whose defaults change nothing; in `ArmingStartsTheRateLoopFromZeroState...` the scenario gyro bias goes from
   (2, -3, 1.5) to (0.2, -0.3, 0.15) rad/s (after calibration a bias is real rotation to the estimator, and the larger one
   tilted it past 25 deg before arming); no assertion, threshold or expected value changes; a new `CalibrationGateTest`
   fixture adds four cases. `tests/regression/quad/L08/unit/arming_test.cpp` gains one case (Calibrating, NotUpright).

## Why

- QF-3 bounds phase margin and M_s; a constant tilt error changes neither, so L7's "derived from QF-3's allowance"
  gives no number. Item 1 ties the share to the angle-mode settle band Luis already chose.
- A gyro sign or axis error passes an accelerometer-referenced still check (the correction re-converges it) and a
  closure check (out and back cancels); check (b) is core §3's "tilt the board" on the real IMU path.
- Core §6 gives contact to the host; quad §3.1 puts it in the specific force only on the ground. The opt-in input keeps
  `FreeFallAccelIsExactlyZero`, the Allan check and every gz log byte-identical.
- The plant has no drag, so the accelerometer reads thrust along body -z at any tilt: an accelerometer-corrected estimator
  drifts toward level in a held 60 deg step and initialises level when inverted, so L5's steps and recoveries fail for a
  plant reason. A kick from level hover is falsifiable; a level start alone is not (0031).
- Betaflight 4.5.1 starts at identity, sets `attitudeIsEstablished` on the first update and gates arming on
  `isUpright()`; initialising from the first healthy sample is correct at the moment of validity, with no gain schedule.
- The plant has no drag, so in flight the accelerometer never shows tilt: a gyro bias drifts the truth without bound
  (0.044 rad over 6.3 s at 0.4 deg/s) and the tick-0 accelerometer noise stays as a fixed tilt (0.0098 rad at seed 1). On
  hardware both are bounded (b / Kp; the init error decays at Kp on the ground before arming), so T4 flies the nominal
  corner, and the gyro-bias budget is measured on the bench (item 2).

## Evidence

`sim/plant/src/marv_plant.cpp:256-264`; `sim/plant/include/marv_plant.h:102-111`; `sim/gz/plugin/src/lockstep.cpp:451-456,
732-734, 912-927`; `tests/regression/quad/L06/imu/kat_test.cpp:59-79`; `tests/regression/quad/L06/gz_compositions/
test_l5_sensors.py:19`; `docs/decisions/0031-l5-t4-predicates.md:43`; `fw/arming/include/marv/arming/arming.hpp:19-20,
33-47, 70-84, 159-179`; `fw/compositions/quad_fc/src/quad_fc.cpp:290-307`; quad spec lines 98-99, 215-220, 452-463,
645; core lines 74-77, 323-330, 399-410. Betaflight tag 4.5.1 (read 2026-10-16): `src/main/flight/imu.c`
(`imuCalcKpGain`, `imuIsAccelerometerHealthy`, `attitudeIsEstablished`, `isUpright`; dcm_kp 2500, dcm_ki 0,
small_angle 25) and `src/main/fc/core.c` `updateArmingStatus` (ANGLE, CALIBRATING). P4 measurements (estimated run, att_kp = 0, att_kp = 0 with
roll/pitch rate ki = 0, the bias corner): script, inputs and raw output committed under
`tests/regression/quad/L07/results/` with P4.

## Approval

Luis, 2026-10-16, answering four questions (each "(Recommended)"): tilt budget "1.2° (Recommended)" (item 1:
`attitude_tilt_error_max` 0.0209 rad is his ruling, no longer a PLACEHOLDER); spec text "Yes (Recommended)" (L7 names the
entry, M3's done-when names the L7 hover and its control, the deferred list gains Gazebo ground contact; applied in this
change); Betaflight's upright check "Yes (Recommended)" (arming refused above `small_angle` 25°); boot gyro calibration
"Yes (Recommended)" (arming refused while it runs). The bench measured the board's still gyro bias at 0.35 deg/s
perpendicular to gravity, b / Kp = 1.41° against the 1.2° budget (`tests/regression/quad/L09/results/m3_bench/`, with
the M3 bench card).
