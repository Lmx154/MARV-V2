"""L11 vertical-loop gain generator (tools/card/vertical.py, flatten.py --out-vertical; decision 0020 sections 1, 4, 5).

The generator's own tests: the nominal run on the committed card and scenario register with a budget that carries
thrust_gain_robustness_band = 0.5 (Luis's ruling 5 of decision 0020; the budget entry itself lands with stage S3a-2, so
the tests add it to a temporary copy of the committed budget), the inner loop's Jury radius and rule 7 at the corners, the
sign and convergence of the lifted closed loop, the gravity-gradient instability with the loop disabled, three refusals,
and flatten --out-vertical. The corner margins and the tightness of the gains are the independent T3 design test's
(test_vertical_design.py).

Tolerances (derived, not tuned): a product of 2^K matrices formed by K squarings in double carries a relative error of at
most about 2^K n u (each squaring doubles the error and adds n u, n = 5 states, u = 2^-52), so 2^K n u bounds the rounding
of the step response and of the free gravity-gradient motion below.
"""

import copy
import math
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import attitude  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import rate  # noqa: E402
import schema  # noqa: E402
import vertical  # noqa: E402

FLATTEN = ROOT / "tools" / "card" / "flatten.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
WHERE = "vehicles/uzh_neurobem_5in.yaml"
IDS = ("vert_kp_v", "vert_ki_v", "vert_kp_z", "vert_loop_ratio")
CORNERS = ("nominal", *vertical.CORNER_NAMES)
EPS64 = 2.0 ** -52
N = attitude.STATE_N
# Planted refusal input: a band at which the low-gain corner has PM < PM_min at every crossover (r_lo = 0.3).
LARGE_BAND = 0.7


def run(*args):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True, check=False)


@pytest.fixture(scope="module")
def real(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("vertical")
    card, budget, scenario = (schema.load_yaml(p) for p in (CARD, BUDGET, SCENARIO))
    entries, report, res = vertical.vertical_entries(card, budget, scenario, CARD, WHERE)
    return {"dir": tmp, "budget_path": BUDGET, "card": card, "budget": budget, "scenario": scenario,
            "entries": dict(entries), "order": [n for n, _ in entries], "report": report, "res": res}


def closed(res, corner, k):
    """(A_cl, b_cl) of the lifted closed height loop at a corner: v_sp = k (z_ref - z), input z_ref."""
    _, jt, tau = next(c for c in res["corners"] if c[0] == corner)
    inner = res["inner"]
    a, b = vertical.vertical_model(inner["kv"], inner["ki"], res["T"], tau, 1 / jt, res["hover"]["gamma"],
                                   res["inputs"]["mass"])
    return [[a[i][j] - (k * b[i] if j == vertical.Z else 0.0) for j in range(N)] for i in range(N)], [k * x for x in b]


def doubled(a, b, k_squarings):
    """(A^(2^K), sum_{i < 2^K} A^i b) by K squarings of the affine map s -> A s + b."""
    for _ in range(k_squarings):
        b = [x + y for x, y in zip(attitude.matvec(a, b), b)]
        a = attitude.matmul(a, a)
    return a, b


# ---- the nominal run ------------------------------------------------------------------------------------------------


def test_the_four_ids_in_order_with_finite_positive_f32_exact_gains(real):
    assert real["order"] == list(IDS)
    for name, unit in zip(IDS[:3], ("N s/m", "N/m", "1/s")):
        e = real["entries"][name]
        v = e["value"]
        assert e["type"] == "f32" and e["unit"] == unit and e["sigma"] == schema.UNKNOWN, name
        assert e["method"].startswith("derived(") and "decision 0020" in e["method"], name
        assert math.isfinite(v) and v > 0 and rate.r32(v) == v, (name, v)
    n = real["entries"]["vert_loop_ratio"]
    assert (n["type"], n["value"], n["unit"], n["sigma"]) == ("i32", 1, "1", 0)
    res = real["res"]
    assert (real["entries"]["vert_kp_v"]["value"], real["entries"]["vert_ki_v"]["value"],
            real["entries"]["vert_kp_z"]["value"]) == (res["inner"]["kv"], res["inner"]["ki"], res["outer"]["k32"])


def test_the_inner_loop_is_jury_stable_and_rule_7_holds_at_every_corner(real):
    """The corner margins of both loops and the tightness of the gains are checked, against an independent model, by
    test_vertical_design.py (d1, d3, d4); what only the generator's own results show is kept here."""
    inner = real["res"]["inner"]
    assert [name for name, _ in inner["jury"]] == list(CORNERS)
    assert all(rad is not None and rad < 1 for _, rad in inner["jury"])
    assert all(c < 0.5 for _, c in inner["conditions"])


# ---- sign, convergence and the gravity gradient -------------------------------------------------------------------------


@pytest.mark.parametrize("corner", CORNERS)
def test_a_height_step_converges_to_the_reference_in_the_lifted_model_and_the_flipped_sign_diverges(real, corner):
    """z_ref = 1 m (down) from rest: with k_v, k_i, k_z > 0 the state after 2^K executions is the step's end, z = 1 within
    the rounding bound, K such that rho^(2^K) <= u (rho the closed loop's Jury radius). Control: -k_z fails the Jury
    test."""
    res = real["res"]
    k32 = res["outer"]["k32"]
    rho = dict((r[0], r[3]) for r in res["outer"]["rows"])[corner]
    k_sq = math.ceil(math.log2(math.log(EPS64) / math.log(rho)))
    a, b = closed(res, corner, k32)
    a_k, s = doubled(a, b, k_sq)
    assert max(abs(x) for row in a_k for x in row) <= 2.0 ** k_sq * N * EPS64
    assert abs(s[vertical.Z] - 1) <= 2.0 ** k_sq * N * EPS64, s
    assert s[1] == pytest.approx(0, abs=2.0 ** k_sq * N * EPS64)
    neg, _ = closed(res, corner, -k32)
    assert not attitude.jury(attitude.charpoly(neg))


def test_with_all_three_gains_zero_the_lifted_model_fails_jury_by_the_gravity_gradient_pole(real):
    """Loop disabled (k_v = k_i = k_z = 0): the Jury test fails, and z released from 1 m grows as cosh(sqrt(gamma) t)
    (the pole e^(sqrt(gamma) T)); with gamma = 0 it stays at 1 (control: the growth is the gradient's)."""
    res = real["res"]
    T, gamma, mass = res["T"], res["hover"]["gamma"], res["inputs"]["mass"]
    tau = res["inputs"]["tau"]
    loop = vertical.VerticalLoop(0.0, 0.0, T, tau, 1.0, gamma, mass, vertical.LOOP_RATIO)
    assert not loop.stable(0.0)
    sigma = math.sqrt(gamma)
    k_sq = math.ceil(math.log2(1 / (sigma * T)))
    bound = 2.0 ** k_sq * N * EPS64
    for g, expect in ((gamma, math.cosh(sigma * 2.0 ** k_sq * T)), (0.0, 1.0)):
        a, b = vertical.vertical_model(0.0, 0.0, T, tau, 1.0, g, mass)
        a_k, _ = doubled(a, [0.0] * N, k_sq)
        z = a_k[vertical.Z][vertical.Z]
        assert abs(z - expect) <= bound * expect, (g, z, expect)
    assert math.cosh(sigma * 2.0 ** k_sq * T) - 1 > bound * math.cosh(sigma * 2.0 ** k_sq * T)


# ---- refusals ---------------------------------------------------------------------------------------------------------


def test_refused_without_the_thrust_gain_band_in_the_budget(real):
    budget = copy.deepcopy(real["budget"])
    del budget["thrust_gain_robustness_band"]
    with pytest.raises(gpc.GenError) as e:
        vertical.design(real["card"], budget, real["scenario"], CARD)
    assert "vertical" in e.value.lines[0] and "thrust_gain_robustness_band" in e.value.lines[0]


def test_refused_when_the_motor_tau_is_unknown(real):
    card = copy.deepcopy(real["card"])
    card["rotors"]["motor_lag"]["tau"]["value"] = schema.UNKNOWN
    with pytest.raises(gpc.GenError) as e:
        vertical.design(card, real["budget"], real["scenario"], CARD)
    assert "vertical" in e.value.lines[0] and "rotors.motor_lag.tau" in e.value.lines[0]


def test_refused_when_the_thrust_gain_band_leaves_nothing_feasible(real):
    budget = copy.deepcopy(real["budget"])
    budget["thrust_gain_robustness_band"]["value"] = LARGE_BAND
    with pytest.raises(gpc.GenError) as e:
        vertical.design(real["card"], budget, real["scenario"], CARD)
    assert "vertical" in e.value.lines[0] and "no feasible velocity crossover" in e.value.lines[0]


# ---- flatten --out-vertical ---------------------------------------------------------------------------------------------


def test_flatten_out_vertical_writes_the_entries_and_the_report(real):
    d = real["dir"]
    r = run(FLATTEN, "--card", CARD, "--budget", real["budget_path"], "--out-card", d / "card.yaml",
            "--out-register", d / "register.yaml", "--scenario", SCENARIO, "--out-scenario", d / "scenario.yaml",
            "--out-vertical", d / "vertical.yaml", "--root", ROOT)
    assert r.returncode == 0, r.stderr
    doc = schema.load_yaml(d / "vertical.yaml")
    assert list(doc) == list(IDS)
    for name in IDS:
        assert doc[name] == real["entries"][name], name
    report = (d / "vertical_report.txt").read_text(encoding="utf-8").splitlines()
    assert report[0] == "MARV L11 vertical-loop derivation (tools/card/vertical.py, decision 0020)"
    assert report == real["report"]
