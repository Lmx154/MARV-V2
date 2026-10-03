"""The stage (c) omega x J omega feed-forward evaluation (decision 0014 owner decisions 1-4; tools/sim/l6_ff_eval.py, raw
outputs in tests/regression/quad/L06/results/ff_eval/).

Re-runs the README's per-push case once (the card plant and the worst physical J corner: l6_ff_eval.main, module fixture)
and checks:
  - its raw output reproduces card_worst.txt byte for byte; control: a changed result renders differently;
  - the corner it runs is the one sweep.txt (the README's full command) names worst for the chosen form (PID+FF+lag), and
    every line of its terms, cross-check and results sections is a line of sweep.txt;
  - the model's cross-check against acro_cause: decision 0005's measured recovery table and cause.txt's m = 1 trace, within
    CROSS_CHECK_REL; control: the same model with the plant's w x J w removed fails it;
  - the PI law's (tools/card/rate.py) design-model P at the L4 sensor reproduces cause.txt's pitch P bit for bit (so the
    PI law is the one the acro_cause build flew);
  - linear_response is run_l4.script_response bit for bit for the L4 T3 fixture's stage (c) law with its chain low-pass;
    control: ki one ulp higher is not.
The variants' verdicts are reported, not asserted (owner decision 2).
"""

import copy
import math
import re
import sys
from pathlib import Path
from unittest import mock

import pytest

ROOT = Path(__file__).resolve().parents[5]
DIR = ROOT / "tests" / "regression" / "quad" / "L06" / "results" / "ff_eval"
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import l6_ff_eval as fe  # noqa: E402

# The worst physical J corner for the chosen form (PID+FF+lag), from sweep.txt's summary (checked below).
WORST_CORNER = 11
# The acceptance bar of the model cross-check: the lead's "within a few %" (stage (c) packet, 2026-10-01), labelled.
CROSS_CHECK_REL = 0.05
# Decision 0005's table prints |w| and the bound to two decimals, so its excess carries two half units of 0.01.
TABLE_HALF_UNIT = 0.005
SECTIONS = ("== predicate terms", "== cross-check", "== results")


@pytest.fixture(scope="module")
def run(tmp_path_factory, rate_lead_design):
    out = tmp_path_factory.mktemp("ff_eval") / "card_worst.txt"
    with mock.patch.object(fe.rl, "design", return_value=rate_lead_design):
        c, res, text = fe.main(["--out", str(out), "--corners", str(WORST_CORNER)])
    return c, res, text, out


def cross_check_findings(xc):
    """Every way a cross-check dict misses the acceptance bar (empty: it meets it)."""
    out = []
    for axis, (p, (peak, bound, count)) in zip(fe.AXES, xc["axes"]):
        if not abs(p["violations"] - count) <= CROSS_CHECK_REL * count:
            out.append(f"{axis}: outside {p['violations']} against {count}")
        gz = peak - bound
        if not abs(-p["worst margin"] - gz) <= CROSS_CHECK_REL * gz + 2 * TABLE_HALF_UNIT:
            out.append(f"{axis}: excess {-p['worst margin']} against {gz}")
    for axis, r, pk in zip(fe.AXES, xc["resid"], xc["peaks"]):
        if not r <= CROSS_CHECK_REL * pk:
            out.append(f"{axis}: trace residual {r} against peak {pk}")
    return out


def section_lines(text):
    lines, keep = [], False
    for ln in text.splitlines():
        if ln.startswith("== "):
            keep = ln.startswith(SECTIONS)
            continue
        if keep and ln.strip():
            lines.append(ln)
    return lines


def test_raw_output_reproduces_byte_for_byte(run):
    *_, out = run
    assert out.read_bytes() == (DIR / "card_worst.txt").read_bytes()


def test_control_a_changed_result_renders_differently(run):
    c, res, text, _ = run
    planted = dict(res)
    key = ("PID+FF+lag", 0, fe.RK_SUBSTEPS, True, "T3")
    planted[key] = copy.deepcopy(res[key])
    planted[key]["pred"][0]["violations"] += 1
    assert fe.render(c, planted) != text


def test_the_corner_run_is_the_sweeps_worst_and_its_rows_are_the_sweeps(run):
    _, _, text, _ = run
    sweep = (DIR / "sweep.txt").read_text()
    m = re.search(r"^PID\+FF\+lag\s+worst corner (\d+) ", sweep, re.M)
    assert m and int(m.group(1)) == WORST_CORNER
    missing = set(section_lines(text)) - set(sweep.splitlines())
    assert not missing, sorted(missing)


def test_model_cross_check_against_acro_cause(run):
    c, res, _, _ = run
    xc = fe.cross_check(c, res)
    assert len(xc["trace"]) > 0
    assert cross_check_findings(xc) == []


def test_control_without_the_coupling_fails_the_cross_check(run):
    c, res, _, _ = run
    assert cross_check_findings(fe.cross_check(c, res, fe.PI_UNCOUPLED))


def test_pi_law_is_the_acro_cause_builds(run):
    c, res, _, _ = run
    (mine, _), (gz, _) = fe.cross_check(c, res)["pitch_bound"]
    assert mine == gz


def test_linear_model_is_the_tests_script_response(run):
    c, *_ = run
    assert fe.linear_matches_script_response(c) == [True, True, True]


def test_control_one_ulp_of_ki_breaks_the_identity(run):
    c, *_ = run
    p, su = c["fixture"], c["su"]
    kp, ki, kd, tf, tau_ref = fe.fixture_law(c, 0)
    jt, tt = p[fe.INERTIA_KEYS[0]], p["motor_tau"]
    maps = (su.plant_map(jt, tt), fe.tick_map(c["su1"], jt, tt))
    sensor = fe.Sensor("the fixture's chain low-pass", 0, (su.lowpass,))
    mine = fe.linear_response(kp, math.nextafter(ki, math.inf), kd, tau_ref, tf, sensor, maps, c["sps"][0], c["stamps"],
                              c["plan"].divisor)
    ref = fe.run_l4.script_response(kp, ki, kd, tf, tau_ref, (su.divisor, su.lowpass, su.lowpass_error),
                                    su.tick_map(jt, tt), c["sps"][0], c["stamps"])
    assert mine != ref
