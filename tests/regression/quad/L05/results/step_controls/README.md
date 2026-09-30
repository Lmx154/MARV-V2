The fine negative controls of the L5 T4 angle-mode steps against the predicate widened by Q (owner decision 19: "confirm
the fine negative controls (gains x 1.1, one tick of added delay) still fail the widened T4 predicate"). Raw output of
`step_controls.py` (the script, in this directory). The widened predicate is envelope +- (E + F + Q), with Q from
`t3/reference/attitude_t3_q.txt`. The script applies it through `tests/regression/quad/L05/gz/test_t4_steps.py`
`evaluate()` for the flown cases, and through the same rule for the counterfactual. The file header lists the inputs with
their SHA-256.

Files:
- `step_controls.py`: the script.
- `step_controls_tool.cpp`: `../step_cause/step_cause_tool.cpp` plus a `<delay_ticks>` argument to its `sim` use. With 0 it
  behaves as step_cause_tool; with 1 each rate execution's command reaches the plant one tick later, as `t3_test.cpp`
  `run_script`'s `delay_ticks`.
- `build_tool.sh`: builds that tool against the host-gz-l5 libraries.
- `controls.txt`: the raw output.

Command (repository root, after `uv sync --frozen`, gz-sim 8,
`cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5`; about 70 s):
`uv run python tests/regression/quad/L05/results/step_controls/step_controls.py --work <scratch dir> --out tests/regression/quad/L05/results/step_controls/controls.txt`

The gz logs and simulated traces go to `--work`; a rerun regenerates them (decision 0003).

Result (measured, `controls.txt`):
- **Baseline.** The flown roll and pitch steps pass the widened predicate. The largest stepped-axis excursion outside the
  envelope is 6.52e-3 rad (roll) and 5.94e-3 rad (pitch), against F + Q = 1.12e-2 and 1.03e-2.
- **(a) att_kp x 1.1, flown in gz.** It fails: 461 (roll) and 470 (pitch) violations, largest slack +3.08e-2 and
  +3.21e-2 rad. The quant counterfactual of the same case agrees (+3.13e-2, +3.27e-2). It is a gz negative control in
  `test_t4_steps.py`.
- **(a') rate kp and ki x 1.1 on all axes, flown in gz.** It does not fail: 0 violations, stepped axis at most 1.46e-3 and
  1.26e-3 rad outside the envelope, inside F alone. The J x tau box of the envelope covers a 10 % rate-gain change.
- **(b) One tick of added delay, quant counterfactual.** It does not fail: 0 violations. The stepped-axis slack is -4.71e-3
  (roll) and -4.40e-3 (pitch) rad, inside the widened band. In the unquantised design model the delay moves theta by at
  most 3.69e-4 rad, which is below F (2.92e-3) alone. The T4 angle steps therefore cannot see it, with or without Q. The
  fine delay control is enforced at T3 only.
