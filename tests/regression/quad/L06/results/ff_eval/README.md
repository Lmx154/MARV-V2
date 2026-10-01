The ω×Jω feed-forward evaluation of L6 stage (c): a T3 design-model run of the L4 acro combined full-stick segment
(decision 0014, owner decisions 1 to 4; decision 0005, owner decision 12). The results are reported, not asserted (owner
decision 2).

Question: does the stage (c) rate loop pass the acro recovery predicate of
`tests/regression/quad/L04/gz/test_t4_acro.py` at the card plant, and by how much does it miss at the physical J
corners?

Files:
- `tools/sim/l6_ff_eval.py`: the tool. Its docstring holds the model, the rules and the inputs.
- `sweep.txt`: the full run, the card plant and all 12 corners, with the card-plant sensitivity cases.
- `card_worst.txt`: the per-push case, the card plant and corner 11 (the worst for the chosen form).
- The pytest is `tests/regression/quad/L06/tools/test_l6_ff_eval.py`, in the CI tools step. It re-runs the per-push case,
  compares the output with `card_worst.txt` byte for byte, and asserts the cross-check and its controls.

Commands (repository root, after `uv sync --frozen`):
- Full: `uv run python tools/sim/l6_ff_eval.py --out tests/regression/quad/L06/results/ff_eval/sweep.txt --corners all --sensitivity`
- Per push: `uv run python tools/sim/l6_ff_eval.py --out tests/regression/quad/L06/results/ff_eval/card_worst.txt --corners 11`

Inputs: the tool reads all of them, retypes none, and the outputs list each with its sha256.
- The card, `design/budget.yaml`, `design/scenario_values.yaml` and the sensor profile, through `rate_lead.design`.
- `scenarios/quad/L04/acro.yaml`.
- The L4 T3 fixture, `rate_t3_inputs.txt`. It holds the L4 product parameters; the cross-check shows it equals the gz
  build's table.
- The l4_rate_scripted register.
- `L04/results/acro_cause/cause.txt` and decision 0005's table, for the cross-check.

Variants:
- **PID** is rate_lead's N* design: the f32 kp, ki, kd and the D low-pass T_f.
- **PID+FF** adds the plain feed-forward g = ω×(Jω).
- **PID+FF+lag** adds the lag-compensated term, g + τ_m ġ with ġ through T_ff (owner decision 1).
- The firmware side knows only the card's J and τ_m.
- τ_ref follows rate.py rule step 9 with rate_lead's τ_cl, the existing rule applied to the new loop (lead decision).

Model:
- **Plant.** A nonlinear rigid body with ω×Jω, using the plant's J. Each motor is a first-order rotor-speed lag (the
  card's τ, exact ZOH per tick) with thrust and torque ∝ W², through the ESC map.
- **Firmware.** Double-precision mirrors of the rate law, the mixer allocation with air-mode priority, the anti-windup
  freeze and thrust_to_dshot. Each tick is one host step (m = 1), and the stamps alternate 312 and 313 µs.
- **Predicate.** |ω| ≤ Z + F + E over the recovery window.
  - E = 0, because E needs two gz runs. This is stricter.
  - Z comes from the linear design model of the same loop without the coupling, over the test's 17×17 τ×J grid.
  - F = H, the 9×9 to 17×17 halving change. TOL = 0 for the D law, also stricter.
- **The PI cross-check** uses `test_t4_acro.design_bound` itself.

Sensors:

| Key | Configuration |
| --- | --- |
| T3 | The evaluation's worst case. Latency 1 tick; every notch fixed at ω_th (rate_lead's operating point); the low-pass. |
| T4c | The stage (c) T4 configuration: truth gyro, notches bypassed. |
| L4 | No chain, no latency. |
| T4e | The stage (e) flight configuration, below. |

T4e in detail:
- Latency 1 tick, then the low-pass and 12 notches.
- At each rate execution, before that tick's filter, `update_notches` runs on the plant's rotor-speed telemetry. The
  telemetry is the speed after the last plant step, on the eee mmmmmmmmm period grid of 1 µs units with 14 poles,
  truncated, decoded to float, and delayed 1 rate period (2 ticks).
- The ESC clock error and Betaflight's 100-eRPM rounding are not modelled, as in the plant's own telemetry model.
- Its linear model, which gives Z and F, holds every notch at ω_hover.

Cross-check (`sweep.txt`, the PI law as flown in acro_cause), model against the gz runs, per axis roll / pitch / yaw:
- Executions outside: 1378 / 1036 / 352, against 1377 / 1037 / 350.
- Excess: 8.4798 / 2.8312 / 0.1407 rad/s, against 8.48 / 2.83 / 0.14.
- The trace in cause.txt is matched to at most 0.0055 rad/s.
- `design_bound` on the fixture reproduces cause.txt's pitch bound bit for bit.
- The linear model is `run_l4.script_response` bit for bit.
- Doubling the RK4 steps per tick changes ω by less than 1e-9 rad/s.
- Control: with ω×Jω removed from the plant, nothing is outside on any axis, and the cross-check fails.

Findings at the card plant: executions outside, of 3201 (roll / pitch / yaw); max excess over Z + F in rad/s, where a
negative value is a margin.

| Sensor | PID | PID+FF | PID+FF+lag |
| --- | --- | --- | --- |
| T3 | FAIL: 1488/1747/623, +5.586 | FAIL: 1270/225/0, +0.733 | **FAIL: 311/0/0, +0.0581 (roll)** |
| T4c | FAIL: +5.234 | FAIL: +0.441 | PASS, margin 0.0032 |
| L4 | FAIL: +5.224 | FAIL: +0.435 | FAIL: 24/0/0, +0.0013 |
| T4e | FAIL: +5.351 | FAIL: +0.538 | **PASS, margin 0.0241** |

- Lag-compensated FF meets the owner's bar only on a knife edge, and the sensor configuration decides it.
  - It passes in the flight configuration (T4e) by 24 mrad/s and in the T4 (c) configuration by 3 mrad/s.
  - It fails by 58 mrad/s with every notch fixed at ω_th (T3), and by 1.3 mrad/s with no chain.
- No saturation flag is set in any run.

Corners (T3): no variant passes at any of the 12 corners.
- Corner 11, J = (0.00375, 0.00315, 0.00215) kg m², J/J0 = (1.5, 1.5, 0.5), is the worst for both FF forms. PID+FF+lag
  exceeds the bound by 9.455 rad/s on pitch, with 2161 pitch executions outside.
- For PID+FF+lag, the largest excess per axis over all corners is 4.34 / 9.46 / 3.93 rad/s.
- PID alone is worst at corner 8 (8.67 rad/s, roll).
- At several corners FF makes the excess worse than PID alone. For example, corner 11 pitch rises from 5.14 to 9.46
  rad/s: FF cancels the coupling only where J is known.
- This excess goes to the pre-L8 gate (owner decision 4).

UNKNOWNs and limits:
- **E is not modelled.** It is set to 0.
- **τ ±30 % is not swept at the corners.** Every plant uses the card's τ.
- **The acro test cannot judge the stage (c) law yet.** `run_l4.script_response` and the T3 oracle model a PI law only
  (kd is ignored). Commit 3 must extend them to the D law and its T_f before the test's Z describes the stage (c) loop.
- **The outputs depend on `rate_lead.design`.** Re-run both commands whenever it changes.

Run time: on 16 CPUs the full command takes about 32 s and the per-push case about 26 s (`rate_lead.design` about 17 s).
In the marv-ci image under `--cpus 4` the pytest takes 76 s and reproduces `card_worst.txt` byte for byte.
