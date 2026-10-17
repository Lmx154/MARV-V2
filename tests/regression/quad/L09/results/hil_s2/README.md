# HIL S2b: N_max of the command delay (decision 0046 item 2, as re-ruled), 2026-10-17

Question: the largest delay N (ticks) of the DShot command, applied at tick j from tick j - N, for which every configuration
row at the card's nominal J and tau (all notch operating points, latency 3 + N samples, the flight f32 gains held fixed) meets
QF-3: PM >= PM_min and Ms <= Ms_max.

Command (from the repository root; about three minutes on 8 processes, the output does not depend on the process count):

    uv run python tools/card/hil_delay_margin.py --procs 8 > tests/regression/quad/L09/results/hil_s2/hil_delay_margin.txt

Inputs: `vehicles/marv_5in_fc3v2.yaml`, `sensors/profiles/madflight_fc3v2_revc.yaml` (latency_samples 3),
`fw/compositions/quad_fc/params/fc3v2_scenario_values.yaml` (tick 1953125/14649 us), `design/budget.yaml`; the rate guard
is `mean_period`, the arguments of `marv_params_fc3v2` (fw/params/CMakeLists.txt). `rate_lead.design` yields the flown f32
gains; the script then evaluates `rate_lead.configuration_rows` (rule step 13: the notch grid at the level that step resolves,
every notch bypassed; the f32 firmware loop, the libm variants) on those gains with the model's latency set to 3 + N, once
with the nominal corner alone (the rows N_max is decided on) and, for N = 0..20, with the nominal and the four band corners
(the worst-corner table). Nothing in rate_lead.py or the budget is changed.

The delay is a pure delay of N ticks in the loop, exactly as the latency is (gyro_chain_design.py:37, 230:
`F = ... z_s^-latency`), so the loop with command delay N at latency 3 is the loop at latency 3 + N.

## Result

**Nominal N_max = 249**, and it is at least 10, the HIL S1 requirement (round trip max 1.26 ms = 9.5 ticks of 133.3 us,
`hil_s1/README.md` on branch hil-s1): the requirement is met with room. The binding floor is Ms: at N = 250 the nominal Ms is
2.000920 against Ms_max 2.0, while the nominal PM is still 56.59 deg (11.59 deg above PM_min). At N = 10 the nominal PM is
72.46 deg and Ms 1.4172.

| N | latency | nominal PM (deg) | nominal Ms | worst-corner PM (deg), not a margin result for HIL |
|---|---|---|---|---|
| 0 | 3 | 73.126198 | 1.398170 | 45.004496 |
| 1 | 4 | 73.060041 | 1.400059 | 44.872656 |
| 3 | 6 | 72.927728 | 1.403848 | 44.608976 |
| 10 | 13 | 72.464630 | 1.417221 | 43.686097 |
| 20 | 23 | 71.803061 | 1.436624 | 42.367698 |
| 249 | 252 | 56.653138 | 1.997864 | not evaluated |
| 250 | 253 | 56.586982 | 2.000920 (above Ms_max) | not evaluated |

The full listing (N = 0..20 both ways, then every 25th nominal row and the last two) is `hil_delay_margin.txt`.

Worst corner, labelled "not a margin result for HIL": the gain rule (rate.py's sup rule, rule step 4) places the worst
corner (pitch, J-, tau+, notches at omega_th) at PM_min, so its PM is 45.004496 deg at N = 0, 0.004496 deg above PM_min, and
each tick costs 0.131840 deg: it falls below PM_min at N = 1 and its Ms passes Ms_max there too. Its N_max is 0 by
construction. A HIL or SIL run's plant is the card's nominal model (core section 6), so that corner is not the plant of the
run; decision 0046 item 2 requires every HIL run report to state N, the nominal rows' PM and Ms, and this shortfall.

Caveat: `sim/gz/sensor_replay` refuses logs that carry records 9 or 10 (rounded host step, command delay); it is not changed
here.
