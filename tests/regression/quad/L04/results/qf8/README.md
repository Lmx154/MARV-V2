QF-8 crossover-vs-loop-rate curve for the L4 rate-loop gain rule (decision 0005 "QF-8 curve"; quad spec §6.2 QF-8). Raw
output of `tools/card/rate_qf8.py` (the script; it imports the gain generator `tools/card/rate.py` and runs its design rule
with `rate_loop_divisor` = 2ⁿ, n = 0..9). The file header lists the exact input values the rule consumes, each with its source file and entry (no file hashes, so an
unrelated edit to a shared input does not break the suite): `vehicles/uzh_neurobem_5in.yaml`, `design/budget.yaml`,
`design/scenario_values.yaml`, and `sensors/profiles/marv_v2_board_default.yaml` for the ODR range.

Command (from the repository root, after `uv sync --frozen`): `uv run python tools/card/rate_qf8.py --card vehicles/uzh_neurobem_5in.yaml --budget design/budget.yaml --scenario design/scenario_values.yaml --output tests/regression/quad/L04/results/qf8/qf8_curve.txt`

File: `qf8_curve.txt`. The QF-8 verdict is UNKNOWN: it needs the motor τ σ, which the card does not have.

The table is a fixed-input golden that equals the live design on 2026-09-30: the frozen test
`tests/regression/quad/L04/tools/test_qf8_curve.py` regenerates it byte for byte from the input values recorded in its own header, not from the live
card, budget or scenario register: `uv run python tools/card/rate_qf8.py --from-header tests/regression/quad/L04/results/qf8/qf8_curve.txt --card vehicles/uzh_neurobem_5in.yaml --output <file>`
(the card is read only for what the table does not consume). A later change to a live input therefore does not break the test and does not
change this file; it is regenerated only by a deliberate new golden (with a decision record, core §7.3) using the command above, inside the `marv-ci` image.
