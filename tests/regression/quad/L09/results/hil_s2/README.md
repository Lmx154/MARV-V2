# HIL S2b: N_max of the command delay (decision 0046 item 2), 2026-10-17

Question: the largest delay N (ticks) of the DShot command, applied at tick j from tick j - N, for which the rate loop's
phase margin at IMU-equivalent latency 3 + N samples still meets PM_min, with the flight gains held fixed.

Command (from the repository root; about one minute on 10 processes, the output does not depend on the process count):

    uv run python tools/card/hil_delay_margin.py --procs 10 > tests/regression/quad/L09/results/hil_s2/hil_delay_margin.txt

Inputs: `vehicles/marv_5in_fc3v2.yaml`, `sensors/profiles/madflight_fc3v2_revc.yaml` (latency_samples 3),
`fw/compositions/quad_fc/params/fc3v2_scenario_values.yaml` (tick 1953125/14649 us), `design/budget.yaml`; the rate guard
is `mean_period`, the arguments of `marv_params_fc3v2` (fw/params/CMakeLists.txt). `rate_lead.design` yields the flown f32
gains; the script then evaluates `rate_lead.configuration_rows` (rule step 13: the notch grid at the level that step resolves,
every notch bypassed; the f32 firmware loop, the libm variants, nominal and the four band corners) on those gains with the
model's latency set to 3 + N. Nothing in rate_lead.py or the budget is changed.

The delay is a pure delay of N ticks in the loop, exactly as the latency is (gyro_chain_design.py:37, 230:
`F = ... z_s^-latency`), so the loop with command delay N at latency 3 is the loop at latency 3 + N.

## Result

N_max = 0. At N = 0 the worst PM is 45.004496 deg, 0.004496 deg above PM_min (45 deg): the gain rule (rate.py's sup rule,
rule step 4) places the crossover where the worst corner's PM equals PM_min, so the flight design has no phase margin to
spend on a delay. Each tick of delay costs 0.131840 deg (pitch axis, corner J-,tau+, notches at omega_th):

| N | latency | worst PM (deg) | PM - PM_min (deg) | worst Ms |
|---|---|---|---|---|
| 0 | 3 | 45.004496 | +0.004496 | 1.999400 |
| 1 | 4 | 44.872656 | -0.127344 | 2.007545 |
| 3 | 6 | 44.608976 | -0.391024 | 2.024012 |
| 10 | 13 | 43.686097 | -1.313903 | 2.083571 |
| 20 | 23 | 42.367698 | -2.632302 | 2.174219 |

The full table N = 0..20 is `hil_delay_margin.txt`. The S1 requirement N >= 10 (round trip max 1.26 ms = 9.5 ticks,
`hil_s1/README.md` on branch hil-s1) is 1.314 deg below PM_min and 0.0836 above Ms_max (2.0); Ms_max already fails at N = 1
(the Ms column is information: decision 0046 item 2 defines N_max by PM only). So an HIL with the S1 delay does not meet the
stage (c) design floors with these gains. This is a design result for the lead, not a defect fixed here: the options are a
redesign of the flight gains with N (a lower crossover or a larger margin in the budget), or a stated relaxation for the HIL
comparison only. Neither is made in this change.
