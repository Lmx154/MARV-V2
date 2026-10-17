# 0042: M3 attitude estimator: tilt budget share, bench replay, plant contact input, hover predicate, estimator validity

## What changed

Architect ruling for quad §4.0 M3 (2026-10-16); this record implements nothing. Items marked (Luis) wait for him.

1. Register: `attitude_tilt_error_max` (rad, design-budget), the angle between the estimate's down direction in body
   axes and the reference's; heading excluded (no magnetometer). PLACEHOLDER 0.0209 rad = `t4_settle_fraction` x
   `angle_tilt_max` until Luis rules; it blocks M4's angle-mode flight, not M3's commits. (Luis)
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
4. Hover predicate: a new gz test in `tests/regression/quad/L07/`: L5's angle mode and attitude law with the estimator
   in authority on the model IMU (no `<attitude_source>`, no TRUTH_STATE, preset `host-gz-l7`), from level at hover
   rotor speed with a roll-rate kick, sticks centred, L5's horizon; truth tilt from the log's STEP body records. S on
   tilt: |tilt| <= s Delta + E + `attitude_tilt_error_max` at the last execution; heading not checked; control
   `att_kp` = 0 must fail. `marv_sil_l5_attitude_scripted` and its compile closure do not change. (Luis: spec text)
5. Validity and arming: valid iff q was initialised from a sample with |f| in (0.9 g, 1.1 g) and is finite and unit.
   `ArmingInputs::estimator_valid` (default false); `ArmState::EstimatorInvalid` appended after `ArmedFailsafeCentred`;
   refusal order ImuStale, MotorTest, Failsafe, NoLink, ImuInvalid, Saturated, EstimatorInvalid, ThrottleUp, SwitchOff /
   SwitchNotCycled; it refuses and never disarms; the estimator steps before the arming step.
   `tests/regression/quad/L08/unit/arming_test.cpp`: the rig sets `estimator_valid = true`; one case added.
   `tests/regression/quad/L09/unit/quad_fc_gate_test.cpp` feeds (0, 0, -9.8) m/s^2; its cases do not change. Amended
   (lead, M3 P2 fix): it gains one case, samples whose GyroValid / AccelValid flags are clear are not fed to the
   estimator (gyro clear: no step, dt spans from the last stepped stamp; accel clear: no correction).

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

## Evidence

`sim/plant/src/marv_plant.cpp:256-264`; `sim/plant/include/marv_plant.h:102-111`; `sim/gz/plugin/src/lockstep.cpp:451-456,
732-734, 912-927`; `tests/regression/quad/L06/imu/kat_test.cpp:59-79`; `tests/regression/quad/L06/gz_compositions/
test_l5_sensors.py:19`; `docs/decisions/0031-l5-t4-predicates.md:43`; `fw/arming/include/marv/arming/arming.hpp:19-20,
33-47, 70-84, 159-179`; `fw/compositions/quad_fc/src/quad_fc.cpp:290-307`; quad spec lines 98-99, 215-220, 452-463,
645; core lines 74-77, 323-330, 399-410. Betaflight tag 4.5.1 (read 2026-10-16): `src/main/flight/imu.c`
(`imuCalcKpGain`, `imuIsAccelerometerHealthy`, `attitudeIsEstablished`, `isUpright`; dcm_kp 2500, dcm_ki 0,
small_angle 25) and `src/main/fc/core.c` `updateArmingStatus` (ANGLE, CALIBRATING).

## Approval

Pending Luis: the share (item 1); spec text for L7 (lines 459-460), M3's done-when (line 220) and the deferred list
(item 3); optionally Betaflight's upright check (`small_angle` 25 deg) and boot gyro calibration as arming refusals.
Items 3 and 5 add cases and set a new input in frozen files; no threshold or predicate is loosened.
