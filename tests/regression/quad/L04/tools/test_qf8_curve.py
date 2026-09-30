"""QF-8 crossover-vs-loop-rate curve (tools/card/rate_qf8.py, decision 0005 "QF-8 curve").

The committed table is a fixed-input golden: the raw output of the script on the input values recorded in its own header
(equal to the live design on 2026-09-30). The tests regenerate it from those recorded values (rate_qf8.py
--from-header), not from the live card, budget or scenario register, so a legitimate change to a shared input does not
break them. They compare byte for byte, tie its 3.2 kHz row to rate.design() evaluated on the header's values, plant a
different motor tau in the header to show the comparison breaks, and add an unrelated budget entry to the live-file
mode to show it does not change the table.
"""

import math
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import rate  # noqa: E402
import rate_qf8  # noqa: E402
import schema  # noqa: E402

CLI = ROOT / "tools" / "card" / "rate_qf8.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
COMMITTED = ROOT / "tests" / "regression" / "quad" / "L04" / "results" / "qf8" / "qf8_curve.txt"
HEADER_END = "  ".join(rate_qf8.COLUMNS)


def run(*args):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True, check=False)


def regenerate(tmp, card=CARD, budget=BUDGET, scenario=SCENARIO, root=ROOT):
    out = tmp / "qf8_curve.txt"
    r = run(CLI, "--card", card, "--budget", budget, "--scenario", scenario, "--output", out, "--root", root)
    assert r.returncode == 0, r.stdout + r.stderr
    return out.read_bytes()


def regenerate_from_header(tmp, header=COMMITTED, card=CARD):
    out = tmp / "qf8_from_header.txt"
    r = run(CLI, "--from-header", header, "--card", card, "--output", out)
    assert r.returncode == 0, r.stdout + r.stderr
    return out.read_bytes()


def rows(data):
    lines = data.decode("utf-8").splitlines()
    return [ln.split("  ") for ln in lines[lines.index(HEADER_END) + 1:]]


def test_the_committed_table_is_the_regenerated_table_byte_for_byte(tmp_path):
    """A fixed-input golden: regenerated from the values in its own header."""
    assert regenerate_from_header(tmp_path) == COMMITTED.read_bytes()


def test_the_table_has_ten_rates_from_6400_down_to_12_point_5_hz_and_the_statements():
    data = COMMITTED.read_bytes()
    text = data.decode("utf-8")
    table = rows(data)
    assert [float(r[1]) for r in table] == [6400 / 2 ** n for n in range(10)]
    assert table[-1][1] == "12.5"
    assert "QF-8 verdict: UNKNOWN - needs the motor tau sigma (card sigma UNKNOWN)" in text
    assert "Spec gap: QF-8 ignores gyro filtering and aliasing; revisit at L6 (decision 0005)" in text
    assert "that each is an available ODR of the IMU is INFERRED (the profile records only the range)" in text


def test_the_3200_hz_row_equals_rate_design_on_the_headers_values():
    values, _ = rate_qf8.parse_header(COMMITTED.read_text(encoding="utf-8"))
    card, budget, scenario = rate_qf8.docs_from_header(values, schema.load_yaml(CARD))
    row = next(r for r in rows(COMMITTED.read_bytes()) if r[1] == "3200")
    n = int(row[0])
    scenario["rate_loop_divisor"]["value"] = 2 ** n
    r = rate.design(card, budget, scenario, CARD)
    assert r["period_us"] == 1e6 / 3200
    corners = [m[2] for m in r["margins"] if m[0] in rate_qf8.CORNERS]
    assert row[2] == rate_qf8._num(r["T"])
    assert row[3] == rate_qf8._num(r["omega_c"])
    assert row[4] == rate_qf8._num(min(corners)) and row[5] == rate_qf8._num(max(corners))
    assert row[7] == rate_qf8._num(math.degrees(r["pm_worst"]))
    assert row[10] == rate_qf8._num(r["tau_cl"])


def test_negative_control_a_different_motor_tau_changes_the_table(tmp_path):
    text = COMMITTED.read_text(encoding="utf-8")
    old = "  motor tau (s) = 0.033   ["
    assert text.count(old) == 1
    planted = tmp_path / "planted_header.txt"
    planted.write_text(text.replace(old, "  motor tau (s) = 0.03   ["), encoding="utf-8")
    other = tmp_path / "other"
    other.mkdir()
    assert rows(regenerate_from_header(other, planted)) != rows(COMMITTED.read_bytes())
    assert regenerate_from_header(other, planted) != COMMITTED.read_bytes()


def test_an_unrelated_new_budget_entry_does_not_change_the_table(tmp_path):
    root = tmp_path / "root"
    (root / "design").mkdir(parents=True)
    (root / "vehicles").mkdir()
    shutil.copytree(ROOT / "sensors", root / "sensors")
    shutil.copy(CARD, root / "vehicles" / CARD.name)
    shutil.copy(SCENARIO, root / "design" / SCENARIO.name)
    extra = (BUDGET.read_text(encoding="utf-8")
             + "\nunrelated_test_entry:\n  value: 12345\n  unit: \"1\"\n  method: design-budget\n  sigma: choice\n"
             "  rationale: a planted legitimate addition that no rate-loop rule consumes\n  used_by: [test]\n")
    (root / "design" / BUDGET.name).write_text(extra, encoding="utf-8")
    args = dict(card=root / "vehicles" / CARD.name, budget=root / "design" / BUDGET.name,
                scenario=root / "design" / SCENARIO.name, root=root)
    with_extra = regenerate(tmp_path, **args)
    plain = tmp_path / "plain"
    plain.mkdir()
    shutil.copy(BUDGET, root / "design" / BUDGET.name)
    assert regenerate(plain, **args) == with_extra
