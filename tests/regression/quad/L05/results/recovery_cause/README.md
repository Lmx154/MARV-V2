Cause of the L5 T4 large-angle recovery failures (decision 0006 C "At and near 180° tilt", F "T4 large-angle recovery";
owner decisions 5 and 15). Raw output of `recovery_cause.py` (the script, in this directory).

The script:
- flies `scenarios/quad/L05/recover_inverted.yaml` (R1) and `recover_tumble.yaml` (R2) at m = 2 and m = 1, as
  `tests/regression/quad/L05/gz/test_t4_recovery.py` does;
- replays each m = 1 run through the composition path with `recovery_cause_tool`, and refuses to report unless the replay
  reproduces every DShot command bit for bit;
- runs a counterfactual ladder: the same float firmware path around binary64 plants (design; allocation plus motor lag and
  map; plus DShot rounding; plus −ω×Jω; rotors from rest or at hover);
- computes an axis-invariant design-model envelope for R1 from `recovery_model.py` over the 17 × 17 τ × J grid.

Files:
- `recovery_cause.py`: the script.
- `recovery_cause_tool.cpp`: `../step_cause/step_cause_tool.cpp` extended with an initial state, the initial rotor speed, the
  gz step-0 zero-rate read, a logged-state injection and the `_gyro` variants. It is harness only, not firmware.
- `build_tool.sh`: builds the tool against the host-gz-l5 libraries.
- `cause.txt`: the raw output.

Command (repository root, after `uv sync --frozen`, gz-sim 8,
`cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5`; about 35 s):
`uv run python tests/regression/quad/L05/results/recovery_cause/recovery_cause.py --work <scratch dir> --out tests/regression/quad/L05/results/recovery_cause/cause.txt`

The gz logs and the tool files go to `--work`; a rerun regenerates them (decision 0003).

Findings (measured, `cause.txt`, section by section):

1. **Initial-rate timing (harness fact, recorded in 0003 item 11).**
   - The plugin reads the body state (`sim/gz/plugin/src/lockstep.cpp:559`) and runs the SIL for host step 0
     (`:576`) before it sets the scenario's rates (`:649-656`).
   - So the TRUTH record and the gyro of ticks 0 .. m − 1, which is execution 0 for m = 1 and m = 2, read ω = 0.
   - The body itself moves from (q0, ω0) at t = 0: the step-1 read is within 2.1e-6 rad and 1.6e-5 rad/s (m = 1) of a
     torque-free rigid body started there.
   - The firmware's first rate execution is the seed execution. It commands hover (765 × 4) either way.
   - L2 already starts its rotation reference at step 1 (`tests/regression/quad/L02/gz/test_analytic.py:269`). No L4
     scenario has nonzero initial rates.
2. **R2 is gyroscopic coupling, not saturation.**
   - No L3 flag is set, and s = t = 1 at all 20747 rate executions.
   - At execution 1, |ω×Jω| is 0.302, 0.245 and 0.055 N m. That is 41 %, 44 % and 33 % of τ_held, and above the largest
     roll and pitch torques the loop ever requests (0.158 and 0.229 N m).
   - The unquantised counterfactual without −ω×Jω lies 10.1 (the largest channel distance) from gz. Adding the term brings
     it within 1.7e-2, and adding DShot rounding as well brings it within 7.6e-3.
   - The vehicle recovers:
     - α < 90° at 0.291 s and α < 0.1π at 0.468 s;
     - final α 8.7e-4 rad, heading error 1.3e-4 rad, |ω| 5.7e-4 rad/s.
3. **Rotors start from rest.**
   - marv_plant has no initial rotor speed: `marv_plant_config` (`sim/plant/include/marv_plant.h:39-57`) has no field for
     it, and `Model::omega_` is zero-initialised (`sim/plant/src/plant_model.hpp:108`). Neither the scenario nor the
     plugin can set one.
   - Spin-up against a start at hover, R1: 1.7e-4, 1.2e-3 and 1.1e-2 rad of tilt and 0.05, 0.16 and 0.47 rad/s at 10,
     20 and 50 ms.
4. **R1 is the noise-selected split branch.**
   - At execution 1 the gz attitude is off the singular set by rounding noise (w = z = 4.3e-17). The split then takes the
     diagonal tilt axis at 135° with ψ = −π/2.
   - The command is bent by +6.447°, which equals −wψ/2 and is within wπ/2 = 12.893°. That is 38.6° from the
     ρ = 0 branch's pure-roll axis, which the design model takes from exactly q0.
   - Injecting gz's execution-1 state into the counterfactual reproduces gz within 2.8e-2. From exactly q0, the
     counterfactual rolls purely about x.
   - The vehicle rights itself: α < 90° at 0.342 s, α < 0.1π at 0.637 s, final α 7.8e-4 rad, final heading error
     −1.6e-4 rad.
   - An axis-invariant envelope from exactly 180° holds the gz tilt angle α: at most 1.5e-3 rad outside, against
     E 1.7e-3 and a halving change of 4.1e-2.
   - It does not hold |ω_xy| (0.76 rad/s outside at 0.79 s) or the yaw rate (2.0 rad/s, where the model has 0). Both come
     from the π/2 heading error that the split leaves for the yaw loop to remove.
