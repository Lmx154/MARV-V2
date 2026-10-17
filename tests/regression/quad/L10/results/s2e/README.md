# HIL S2e: quad_fc flies the real FC3v2 card in Gazebo (angle mode, launch pad, N = 10)

SIMULATION ON INFERRED / UNVERIFIED / PLACEHOLDER CARD VALUES (vehicles/marv_5in_fc3v2.yaml), NOT A FLIGHT RESULT. Evidence, not a
test: nothing tuned (no fw/, card, gain or threshold change). Tool: `tools/sim/run_quad_fc.py` (docstring: world, script, hover-stick
chain with file:line). Commit of the runs: 7407bb6 (clean tree; the reports say so).

```sh
cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc -j 6 && uv sync --frozen
uv run python tools/sim/run_quad_fc.py --out-dir <dir>/half_stick --runs both            # flight + control, stick 0.5
uv run python tools/sim/run_quad_fc.py --out-dir <dir>/tenth_stick --runs flight --step-stick 0.1
```
Each run: ~6 s wall (RTF 3.5), 161 632 ticks (21.55 s), deterministic (two flight runs: the same log sha256; the 80 MB logs are
not committed, their sha256 is in each `analysis.txt`: flight 2aeb8293..., control b27f7e4b..., tenth-stick flight bf807a6d...).
Per run dir: `world.sdf`, `report.txt` (L2 run report), `analysis.txt` (all numbers below), `traces.csv.xz` (234 Hz), `scenario/hover.yaml`.

Superseded for tilt behaviour: there is no drag in this plant (decision 0049). The re-run on the drag plant is `../s2e_drag/README.md`
(truth tilt 3.8 deg at the end of roll_centre, 3.3 deg at the end of pitch_centre, 5.8 deg at the end of the run, against 13.0, 18.6 and 18.9 deg here).

## Result
- Hover works. Armed at 2.7522 s; throttle raised to the DERIVED hover stick 0.0716427 at 3.0 s (m g(0.82 rad, 500 m) = 7.1783 N;
  s = (m g - T_min)/(T_max - T_min), T_min = 0, T_max = 100.196 N; omega_h 800.58 rad/s, D* 583.06); **lift-off (pad release) at tick
  27340, 3.6452 s** (645 ms after the raise: the thrust approaches the weight with the 50 ms lag and the DShot dither first crosses it
  there). Hold 1 (lift-off to 8 s): **tilt mean 0.164 deg, max 0.242 deg**; z drift 0.0005 m, vz 0.0002 m/s (the exact derived hover:
  no drift). Disarm: stop command 4.2 ms after the switch edge.
- Roll/pitch steps with the loop closed show a defect of the estimator in this sim, not of the loop (diagnosed, not fixed). Stick 0.5
  (30 deg setpoint): the tilt overshoots to 36.3 (roll) and 38.5 deg (pitch), steady error of the last 0.5 s 4.2 deg (roll), and **after
  centring the quad does not return to level**: tilt 13.0 deg at the end of roll_centre, 18.6 at the end of pitch_centre, 18.9 deg at the
  end (tilt max 38.9 deg overall; no flip, no divergence). Stick 0.1 (6 deg): overshoot 1.27 deg, steady error 0.86 deg, residual tilt
  2.7 deg after roll, 4.0 deg after pitch, stays 4.2 deg.
  Evidence (analysis.txt "Accelerometer"): in every flying phase |f|/g = 1.000 and the accelerometer-implied tilt is 0.26 deg whatever the true
  tilt (open-loop throttle at hover, no aerodynamic drag in the plant: the specific force is thrust along body z). The firmware's Mahony
  estimator (kp 0.25, ki 0, no gating, mahony.hpp) therefore drifts toward level at about kp x tilt; a binary64 replica on the logged IMU
  samples (`mahony_replica`, analysis only, INFERRED to match the firmware's f32) gives estimate-vs-truth z-axis error 4.3 deg (mean) in the 30 deg
  step, 12.7 deg after it, 18.9 deg at the end, while the loop tracks the ESTIMATE (replica setpoint error 4.7 deg steady = the estimator
  drift rate over att_kp 2.22/s). So angle mode holds the estimated attitude; truth stays tilted. With drag (a real quad) the specific
  force realigns; the sim plant has none. Also horizontal speed grows to 31.8 m/s (no drag) and, with the tilt, the open-loop throttle loses altitude.
- Throttle dip (0.3 s, stick 0 < arm_throttle_min 0.0605). Stick 0.1 run (cleanest): DShot 48..108 (mean 65: the mixer keeps roll/pitch/yaw
  authority over collective thrust), rotors fall to 22 rad/s; altitude lost in the dip window 0.480 m, descent speed gained 2.67 m/s and
  not recovered by the open-loop throttle (12.0 m lower 3.3 s after the dip start); tilt max 4.06 deg in the dip, 4.29 after (no change);
  body rate max 0.005 rad/s in the dip, 0.022 rad/s in the first second after; commanded torque after the dip 0.0026/0.0023/0.0006 Nm,
  the same level as the hold's 0.0026/0.0015/0.0008 Nm. Stick 0.5 run: 2.98 m in the window, tilt 18.7 -> 18.9 deg. **Nothing abnormal after
  the dip, but the rate integrator itself is not in the log and the integrator hold (decision 0045) is NOT visibly demonstrated**: the card
  is symmetric (no trim torque), so the integrator is about zero before the dip. Suggested follow-up (not done): dip while a stick is held.
- Control (att_kp = 0, rate_ki_roll = rate_ki_pitch = 0 via sil_override; same script, stick 0.5): the stick steps do nothing (error 29.96 deg
  = the setpoint), tilt drifts linearly to 0.30 deg at 21.5 s (hold 1: mean 0.011, max 0.034 deg). The tracking of the commanded tilt comes
  from the loop; level hold on the symmetric perfect-model card needs no loop, so the hold phases do not discriminate (as decision 0042 notes).

## Scenario values (labelled) and derived
Tick 1953125/14649 us (rounded to 133328 ns, -1.602 ppm), N = 10 ticks (0046), m = 1, model gyro with no turn-on bias, site of L2 hover.yaml.
Ready = gyro_cal 1.25 s + recovery 0.5 s + 1.0 s margin = 2.75 s; armed idle 0.25 s; hold 5 s; steps 2 s, centre 3 s; dip 0.3 s; hold 3 s;
tail 0.25 s; frames every 8 ms; ch5 -1/+1 arm, ch8 +1 angle. Step stick 0.5 (packet example) and 0.1; setpoint = stick x angle_tilt_max (60 deg).
Analysis values: steady = last 0.5 s of a phase; settle band 10 % (never reached in these runs). Setpoint error uses the truth heading.

## Noticed, not changed
- tests/regression/quad/L10/gz/test_quad_fc_plumbing.py:90 asserts rc_channel_angle == 0 and fails at collection since 09f900f (register is 8).
- Stale gz steps: 2194 in 3.65-4.18 s right after lift-off (net force about 1e-5 N, gz pose exactly unchanged); an artefact of the exact hover.
- The MARV L2 report prints the scratch paths of the run.
