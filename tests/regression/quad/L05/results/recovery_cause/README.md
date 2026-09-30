Cause file of the L5 T4 large-angle recovery (decision 0006 C, F "T4 large-angle recovery"; owner decisions 5, 15 and 23), on
the final scenarios. Both start with the rotors at the hover speed (`initial_state.rotor_speed_rad_s: hover`, decision 0007):
- R1, `scenarios/quad/L05/recover_inverted.yaml`: roll π − 0.01;
- R2, `recover_tumble.yaml`: inverted, at rate_max on all axes.

The raw output of `recovery_cause.py`, the script in this directory, is `cause.txt`. The script:
- flies R1 and R2 at m = 2 and m = 1, as `tests/regression/quad/L05/gz/test_t4_recovery.py` does;
- replays each m = 1 run through the composition path with `recovery_cause_tool`, and refuses to report unless every DShot
  command is reproduced bit for bit;
- runs the counterfactual ladder of the same float firmware path around binary64 plants, with the rotors at the world's
  initial speeds:
  - design;
  - cont: allocation, motor lag and rotor map;
  - quant: plus the DShot rounding;
  - `_gyro` variants: plus −ω×Jω;
- compares the R1 counterfactual with gz against the test's own E + F + Q (`make_design`).

Files:
- `recovery_cause.py`: the script.
- `recovery_cause_tool.cpp`: `../step_cause/step_cause_tool.cpp` extended with the initial state, the initial rotor
  speeds, the gz step-0 zero-rate read, a logged-state injection and the `_gyro` variants. It is harness only.
- `build_tool.sh`: builds the tool against the host-gz-l5 libraries.
- `cause.txt`: the raw output.

Command (repository root, after `uv sync --frozen`, gz-sim 8,
`cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5`; about 35 s):
`uv run python tests/regression/quad/L05/results/recovery_cause/recovery_cause.py --work <scratch dir> --out tests/regression/quad/L05/results/recovery_cause/cause.txt`

The gz logs and tool files go to `--work`; a rerun regenerates them (decision 0003).

Findings (measured, `cause.txt`):

1. **Initial state in gz.**
   - The rotors start at 1100.6 rad/s.
   - The execution-0 gyro and TRUTH read ω = 0, because the plugin sets the initial rates after its step-0 read
     (`sim/gz/plugin/src/lockstep.cpp`, 0003 item 11).
   - The body moves from (q0, ω0) at t = 0: the step-1 read is within 2.1e-6 rad and 1.6e-5 rad/s (m = 1) of a torque-free
     body started there.
   - The test skips the rate channels at execution 0.
2. **R2 is the gyroscopic coupling limit** (the L4 safety item's pattern).
   - The replay is bit-exact at all 20747 rate executions. No L3 flag is set, and s = t = 1 throughout.
   - At execution 1, |ω×Jω| is (0.302, 0.245, 0.055) N m, which is 41 %, 44 % and 33 % of τ_held (0.743, 0.557,
     0.163). That is above the largest roll and pitch torques the loop requests (0.136 and 0.213 N m).
   - The largest channel distance to gz along the ladder:

     | Variant | Distance to gz |
     |---|---|
     | design | 7.96 |
     | cont | 8.66 |
     | cont + ω×Jω | 1.46e-2 |
     | quant + ω×Jω | 7.9e-3 |

   - The vehicle recovers:
     - α < 90° at 0.259 s (design 0.124 s) and α < 0.1π at 0.471 s (design 0.975 s);
     - final α 8.1e-4 rad, heading error 1.6e-4 rad, |ω| 5.1e-4 rad/s.
3. **R1.**
   - The replay is bit-exact at all 20747 rate executions.
   - The start is off the singular set (ρ = 5.0e-3), so the split's tilt axis is x and the command is a pure roll,
     (−6.1745, 0, 0).
   - The test predicate passes on gz with 0 violations.
   - The quant_gyro counterfactual matches gz within E + F + Q on every channel. The largest distance is 8.1e-4 rad
     (err_x) and 3.9e-3 rad/s (w_x), and the worst slack is −2.8e-4.
   - α < 90° at 0.332 s and α < 0.1π at 0.579 s.

Superseded. Commit 7028cf8 analysed the earlier scenarios: rotors from rest, and R1 exactly at 180°. There, R1 took the split
branch picked by gz's rounding noise. That start is now `recover_inverted_exact.yaml`, covered by the test's α side check.
The rest-rotor R2 had the same classification. Those numbers are in the git history and no longer apply.
