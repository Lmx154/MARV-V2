"""L4 rate-loop parameters from the vehicle card (tools/card/rate.py, flatten.py --out-rate; decision 0005 "Gain rule",
"QF-2", "kd").

The generator evaluates the loop by the closed-form factors of L(z). The oracle here is a different evaluation: the
plant is sampled from its state matrices (matrix exponential of [[A, B], [0, 0]] T, physical units, the true inertia and
the emitted gains, no normalisation), the loop is the state-space system [omega, motor torque, integrator] and its
frequency response comes from a complex linear solve; crossover and phase are found on a different grid with the phase
unwrapped by stepping.

Numeric tolerance TOL = 2^-30 rad (9.3e-10). Derivation: both evaluators bisect the crossover to adjacent doubles, and
the PM error of that is |dPM/dw| w 2^-52 (below 1e-14 rad); the solve's rounding is bounded by cond(zI - A_ol) eps with
cond below 1e3 near w = 10 rad/s (|z - 1| is 3e-3), that is 1e-13 rad. TOL leaves four decades of slack and stays below
the generator's delta_num (about 9e-9 rad), the margin it demands over PM_min.
"""

import cmath
import copy
import functools
import json
import math
import re
import subprocess
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import gen_plant_config as gpc  # noqa: E402
import mixer  # noqa: E402
import rate  # noqa: E402
import schema  # noqa: E402

FLATTEN = ROOT / "tools" / "card" / "flatten.py"
GEN = ROOT / "tools" / "gen" / "params_gen.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"

AXES = ("roll", "pitch", "yaw")
IDS = [f"rate_{q}_{a}" for q in ("kp", "ki", "kd", "tau_ref") for a in AXES]
OTHER_OUTPUTS = ("card.yaml", "register.yaml", "mixer.yaml", "scenario.yaml")
TOL = 2.0 ** -30
EPS32 = 2.0 ** -24  # float32 unit roundoff
EPS64 = 2.0 ** -52


def run(*args):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True, check=False)


def flatten(d, card=CARD, budget=BUDGET, scenario=SCENARIO, rate_out=True):
    args = [FLATTEN, "--card", card, "--budget", budget, "--out-card", d / "card.yaml",
            "--out-register", d / "register.yaml", "--out-mixer", d / "mixer.yaml", "--root", ROOT,
            "--scenario", scenario, "--out-scenario", d / "scenario.yaml"]
    if rate_out:
        args += ["--out-rate", d / "rate.yaml"]
    return run(*args)


def r32(x):
    return rate.r32(x)


# ---- the independent evaluation: sampled state-space plant, state-space loop, complex solve -----------------------


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def expm(m):
    """exp(m) by scaling and squaring around a 40-term Taylor series."""
    n = len(m)
    norm = max(sum(abs(x) for x in row) for row in m)
    s = max(0, math.ceil(math.log2(norm)) + 1) if norm > 0 else 0
    a = [[x / 2 ** s for x in row] for row in m]
    term = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    total = [row[:] for row in term]
    for k in range(1, 40):
        term = [[x / k for x in row] for row in matmul(term, a)]
        total = [[x + y for x, y in zip(r1, r2)] for r1, r2 in zip(total, term)]
    for _ in range(s):
        total = matmul(total, total)
    return total


@functools.lru_cache(maxsize=None)
def sampled_plant(j_true, tau, T):
    """(Ad, Bd) of omega' = m/J, m' = (u - m)/tau under zero-order-hold input u, sample period T."""
    aug = [[0.0, T / j_true, 0.0], [0.0, -T / tau, T / tau], [0.0, 0.0, 0.0]]
    e = expm(aug)
    return [[e[0][0], e[0][1]], [e[1][0], e[1][1]]], [e[0][2], e[1][2]]


def solve3(a, b):
    n = 3
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            m[r] = [x - f * y for x, y in zip(m[r], m[c])]
    x = [0j] * n
    for r in reversed(range(n)):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def loop_response(w, j_true, tau, kp, ki, T, extra_delay=0):
    """L(e^{jwT}) of the state-space loop [omega, m, xi]: u = kp e + xi, xi+ = xi + ki T e, y = omega, from e to y."""
    ad, bd = sampled_plant(j_true, tau, T)
    z = cmath.exp(1j * w * T)
    a = [[z - ad[0][0], -ad[0][1], -bd[0]], [-ad[1][0], z - ad[1][1], -bd[1]], [0, 0, z - 1]]
    x = solve3(a, [bd[0] * kp, bd[1] * kp, ki * T])
    return x[0] / z ** extra_delay


def phase_margin(j_true, tau, kp, ki, T, extra_delay=0):
    """(PM, crossover rad/s) of the state-space loop; the crossover must be unique on a ratio-1.02 grid from 1e-2 rad/s
    to Nyquist, and the phase is unwrapped by stepping from the first grid point."""
    ws = [1e-2]
    while ws[-1] * 1.02 < math.pi / T:
        ws.append(ws[-1] * 1.02)
    mags = [abs(loop_response(w, j_true, tau, kp, ki, T, extra_delay)) for w in ws]
    changes = [i for i in range(len(ws) - 1) if (mags[i] > 1) != (mags[i + 1] > 1)]
    assert len(changes) == 1 and mags[0] > 1, "crossover not unique"
    lo, hi = ws[changes[0]], ws[changes[0] + 1]
    for _ in range(200):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        if abs(loop_response(mid, j_true, tau, kp, ki, T, extra_delay)) > 1:
            lo = mid
        else:
            hi = mid
    wx = (lo + hi) / 2
    phase = cmath.phase(loop_response(ws[0], j_true, tau, kp, ki, T, extra_delay))
    if phase > 0:
        phase -= 2 * math.pi
    prev = phase
    for w in [x for x in ws[1:] if x < wx] + [wx]:
        p = cmath.phase(loop_response(w, j_true, tau, kp, ki, T, extra_delay))
        d = (p - cmath.phase(cmath.exp(1j * prev)) + math.pi) % (2 * math.pi) - math.pi
        prev += d
    return math.pi + prev, wx


def box_corners(j_a, tau, b_j, b_tau):
    return [(j_a * (1 + sj * b_j), tau * (1 + st * b_tau)) for sj in (-1, 1) for st in (-1, 1)]


def pm_worst(j_a, tau, kp, ki, T, b_j, b_tau, extra_delay=0):
    loops = [(j_a, tau)] + box_corners(j_a, tau, b_j, b_tau)
    return min(phase_margin(j, t, kp, ki, T, extra_delay)[0] for j, t in loops)


def simulate_t63(j_true, tau, kp, ki, T):
    """First sample time at which the closed-loop unit step response of the sampled state-space plant reaches 1 - 1/e."""
    ad, bd = sampled_plant(j_true, tau, T)
    w = m = xi = 0.0
    for n in range(2 ** 20):
        if w >= 1 - math.exp(-1):
            return n * T
        err = 1 - w
        u = kp * err + xi
        xi += ki * T * err
        w, m = ad[0][0] * w + ad[0][1] * m + bd[0] * u, ad[1][0] * w + ad[1][1] * m + bd[1] * u
    raise AssertionError("no rise")


# ---- fixtures -------------------------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def real(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("rate")
    r = flatten(tmp / "f")
    assert r.returncode == 0, r.stderr
    card, budget, scenario = (schema.load_yaml(p) for p in (CARD, BUDGET, SCENARIO))
    res = rate.design(card, budget, scenario, CARD)
    flat = schema.load_yaml(tmp / "f" / "rate.yaml")
    return {"res": res, "flat": flat, "dir": tmp / "f", "card": card, "budget": budget, "scenario": scenario}


def emitted(real, axis):
    f = real["flat"]
    return f[f"rate_kp_{axis}"]["value"], f[f"rate_ki_{axis}"]["value"]


def bands(real):
    i = real["res"]["inputs"]
    return i["b_J"], i["b_tau"]


# ---- structure of the gains -----------------------------------------------------------------------------------------


def test_the_twelve_ids_in_order_and_the_l04_manifest_lists_them(real):
    assert list(real["flat"]) == IDS
    ids = [i for i in (ROOT / "tests" / "regression" / "quad" / "L04" / "param_ids").read_text().split()
           if not i.startswith("#")]
    for name in IDS:
        assert ids.count(name) == 1, name


def test_ki_over_kp_is_the_crossover_over_a_and_kp_over_j_is_one_number_before_rounding(real):
    res = real["res"]
    pm_min = real["budget"]["PM_min"]["value"]
    a = math.sqrt((1 + math.sin(pm_min)) / (1 - math.sin(pm_min)))
    assert res["a"] == pytest.approx(a, rel=8 * EPS64)
    assert res["kappa_i"] / res["kappa_p"] == pytest.approx(res["omega_c"] / a, rel=8 * EPS64)
    for axis in AXES:
        v = res["axes"][axis]
        assert v["kp_double"] / v["J"] == pytest.approx(res["kappa_p"], rel=8 * EPS64)
        assert v["ki_double"] / v["J"] == pytest.approx(res["kappa_i"], rel=8 * EPS64)
        assert v["ki_double"] / v["kp_double"] == pytest.approx(res["omega_c"] / a, rel=8 * EPS64)


def test_after_rounding_the_gains_are_f32_within_one_half_ulp_and_kp_over_j_agrees_across_axes(real):
    res = real["res"]
    for axis in AXES:
        kp, ki = emitted(real, axis)
        v = res["axes"][axis]
        assert (kp, ki) == (v["kp"], v["ki"])
        assert kp == r32(kp) and ki == r32(ki)
        assert kp == pytest.approx(v["kp_double"], rel=EPS32) and ki == pytest.approx(v["ki_double"], rel=EPS32)
    j = [res["axes"][a]["J"] for a in AXES]
    kps = [emitted(real, a)[0] / j[i] for i, a in enumerate(AXES)]
    kis = [emitted(real, a)[1] / j[i] for i, a in enumerate(AXES)]
    assert max(kps) / min(kps) - 1 <= 2 * EPS32 * (1 + 1e-6)
    assert max(kis) / min(kis) - 1 <= 2 * EPS32 * (1 + 1e-6)


def test_kd_is_zero_derived_and_exact_and_the_other_sigmas_are_unknown(real):
    for a in AXES:
        e = real["flat"][f"rate_kd_{a}"]
        assert e["value"] == 0 and e["sigma"] == 0 and e["unit"] == "N m s^2/rad"
        assert e["method"].startswith("derived(") and "0005" in e["method"]
        for q, unit in (("kp", "N m s/rad"), ("ki", "N m/rad"), ("tau_ref", "s")):
            e = real["flat"][f"rate_{q}_{a}"]
            assert e["sigma"] == schema.UNKNOWN and e["unit"] == unit and e["type"] == "f32"
            assert e["method"].startswith("derived(") and "0005" in e["method"] and "0005" in e["source"]


def test_params_gen_takes_them_as_derived_f32_with_the_declared_sigma_kinds(real, tmp_path):
    g = run(GEN, "--card", real["dir"] / "card.yaml", "--card", real["dir"] / "mixer.yaml", "--card",
            real["dir"] / "scenario.yaml", "--card", real["dir"] / "rate.yaml", "--register",
            real["dir"] / "register.yaml", "--out-dir", tmp_path / "gen")
    assert g.returncode == 0, g.stderr
    params = json.loads((tmp_path / "gen" / "params_manifest.json").read_text())["params"]
    text = (tmp_path / "gen" / "param_defaults.cpp").read_text()
    for name in IDS:
        assert params[name]["type"] == "f32"
        block = re.search(rf"    // {name}\n(.*?)\n    // ", text, flags=re.S) or re.search(rf"    // {name}\n(.*)", text,
                                                                                            flags=re.S)
        assert "ParamMethod::Derived" in block.group(1).split("\n")[0], name
        want = "SigmaKind::Exact" if "_kd_" in name else "SigmaKind::Unknown"
        assert want in block.group(1).split("\n")[1], name


# ---- the margins, by an independent evaluation ----------------------------------------------------------------------


def test_worst_case_margin_on_the_emitted_f32_gains_meets_pm_min_and_agrees_with_the_generator(real):
    res = real["res"]
    b_j, b_tau = bands(real)
    pm_min = real["budget"]["PM_min"]["value"]
    tau, T = res["inputs"]["tau"], res["T"]
    for axis in AXES:
        kp, ki = emitted(real, axis)
        j = res["axes"][axis]["J"]
        got = pm_worst(j, tau, kp, ki, T, b_j, b_tau)
        assert got >= pm_min, (axis, got - pm_min)
        assert got >= pm_min + res["delta_num"] - TOL
        assert abs(got - res["axes"][axis]["pm_worst_f32"]) <= TOL, (axis, got, res["axes"][axis]["pm_worst_f32"])


def test_every_loop_of_the_box_agrees_with_the_generators_report_on_the_double_gains(real):
    res = real["res"]
    b_j, b_tau = bands(real)
    tau, T = res["inputs"]["tau"], res["T"]
    j = res["axes"]["roll"]["J"]
    kp, ki = res["axes"]["roll"]["kp_double"], res["axes"]["roll"]["ki_double"]
    loops = [(j, tau)] + box_corners(j, tau, b_j, b_tau)
    assert len(res["margins"]) == 5
    for (jt, tt), (name, pm, wx, _) in zip(loops, res["margins"]):
        got, gwx = phase_margin(jt, tt, kp, ki, T)
        assert abs(got - pm) <= TOL, name
        assert gwx == pytest.approx(wx, rel=1e-9), name
    assert res["pm_worst"] == min(m[1] for m in res["margins"])
    assert res["pm_worst"] >= real["budget"]["PM_min"]["value"]
    assert res["pm_nom"] == res["margins"][0][1]
    assert math.degrees(res["pm_worst"]) == pytest.approx(45.0, abs=1e-5)
    assert min(res["margins"], key=lambda m: m[1])[0] == "J-,tau+"


def test_the_generators_numbers_are_the_architects_scratch_values_within_their_stated_precision(real):
    res = real["res"]
    assert res["omega_c"] == pytest.approx(8.3, abs=0.1)
    assert math.degrees(res["pm_nom"]) == pytest.approx(52.0, abs=0.2)
    assert res["tau_cl"] == pytest.approx(0.161, abs=0.001)
    assert res["axes"]["roll"]["tau_max"] == pytest.approx(2.44, abs=0.01)


def test_the_monotonicity_condition_holds_with_room_and_is_reported(real):
    res = real["res"]
    lo = next(m for m in res["margins"] if m[0] == "J-,tau-")
    tau_lo = (1 - res["inputs"]["b_tau"]) * res["inputs"]["tau"]
    assert res["condition"] == lo[2] ** 2 * res["T"] * tau_lo < 0.5
    report = (real["dir"] / "rate_report.txt").read_text()
    assert repr(res["condition"]) in report and repr(res["delta_num"]) in report and repr(res["tau_cl"]) in report
    for token in ("0005", "kappa_p", "PM_nom", "PM_worst", "t63", "tau_max", "alpha_max", "tau_ref", "J-,tau+"):
        assert token in report


# ---- the corner reduction, on a grid ---------------------------------------------------------------------------------


@pytest.mark.parametrize("scale", [1.0, 0.5], ids=["box", "half-box"])
def test_pm_is_monotone_in_tau_and_has_no_interior_minimum_in_j_on_a_grid(real, scale):
    res = real["res"]
    b_j, b_tau = bands(real)[0] * scale, bands(real)[1] * scale
    tau, T = res["inputs"]["tau"], res["T"]
    j_a = res["axes"]["roll"]["J"]
    kp, ki = emitted(real, "roll")
    n = 9
    js = [j_a * (1 - b_j + 2 * b_j * i / (n - 1)) for i in range(n)]
    ts = [tau * (1 - b_tau + 2 * b_tau * i / (n - 1)) for i in range(n)]
    pm = [[phase_margin(j, t, kp, ki, T)[0] for t in ts] for j in js]
    for row in pm:
        for k in range(n - 1):
            assert row[k + 1] <= row[k] + TOL, "PM rises with tau"
    for k in range(n):
        col = [pm[i][k] for i in range(n)]
        for i in range(1, n - 1):
            assert not (col[i] < col[i - 1] - TOL and col[i] < col[i + 1] - TOL), f"interior minimum in J at tau[{k}]"
    corners = [pm[0][0], pm[0][-1], pm[-1][0], pm[-1][-1]]
    assert min(min(row) for row in pm) >= min(corners) - TOL
    assert min(min(row) for row in pm) <= min(corners) + TOL


# ---- negative controls ----------------------------------------------------------------------------------------------


def test_gains_times_1_1_break_the_margin(real):
    res = real["res"]
    b_j, b_tau = bands(real)
    pm_min = real["budget"]["PM_min"]["value"]
    for axis in AXES:
        kp, ki = emitted(real, axis)
        got = pm_worst(res["axes"][axis]["J"], res["inputs"]["tau"], 1.1 * kp, 1.1 * ki, res["T"], b_j, b_tau)
        assert got < pm_min - 1e-3, (axis, math.degrees(got))


def test_one_extra_sample_of_delay_breaks_the_margin(real):
    res = real["res"]
    b_j, b_tau = bands(real)
    pm_min = real["budget"]["PM_min"]["value"]
    for axis in AXES:
        kp, ki = emitted(real, axis)
        got = pm_worst(res["axes"][axis]["J"], res["inputs"]["tau"], kp, ki, res["T"], b_j, b_tau, extra_delay=1)
        assert got < pm_min - 1e-3, (axis, math.degrees(got))


def test_the_same_evaluator_passes_the_unperturbed_gains(real):
    """Positive control for the two controls above: without the perturbation the evaluator gives PM >= PM_min."""
    res = real["res"]
    b_j, b_tau = bands(real)
    kp, ki = emitted(real, "roll")
    assert pm_worst(res["axes"]["roll"]["J"], res["inputs"]["tau"], kp, ki, res["T"], b_j, b_tau) >= \
        real["budget"]["PM_min"]["value"]


def plant_card(tmp, old, new):
    text = CARD.read_text(encoding="utf-8")
    assert text.count(old) == 1, old
    path = tmp / "card.yaml"
    path.write_text(text.replace(old, new), encoding="utf-8")
    return path


def refused(tmp, **kwargs):
    d = tmp / "flat"
    r = flatten(d, **kwargs)
    assert r.returncode == 1, r.stdout + r.stderr
    assert "rate:" in r.stderr and "decision 0005" in r.stderr, r.stderr
    assert not d.exists() or not list(d.iterdir()), "something was written"
    return r


def test_a_card_with_a_much_larger_motor_tau_leaves_no_feasible_crossover_and_nothing_is_written(tmp_path):
    card = plant_card(tmp_path, "value: 0.033\n", "value: 33.0\n")
    r = refused(tmp_path, card=card)
    assert "no feasible crossover" in r.stderr


def test_a_budget_band_of_almost_one_leaves_no_feasible_crossover_and_nothing_is_written(tmp_path):
    text = BUDGET.read_text(encoding="utf-8")
    entry = text[text.index("inertia_robustness_band:"):]
    assert entry.count("value: 0.5\n") >= 1
    bad = tmp_path / "budget.yaml"
    bad.write_text(text.replace(entry, entry.replace("value: 0.5\n", "value: 0.999\n", 1), 1), encoding="utf-8")
    r = refused(tmp_path, budget=bad)
    assert "no feasible crossover" in r.stderr


def test_the_generator_refuses_by_exception_for_the_same_planted_inputs():
    card = schema.load_yaml(CARD)
    card["rotors"]["motor_lag"]["tau"]["value"] = 33.0
    with pytest.raises(gpc.GenError):
        rate.design(card, schema.load_yaml(BUDGET), schema.load_yaml(SCENARIO), CARD)
    budget = schema.load_yaml(BUDGET)
    budget["inertia_robustness_band"]["value"] = 0.999
    with pytest.raises(gpc.GenError):
        rate.design(schema.load_yaml(CARD), budget, schema.load_yaml(SCENARIO), CARD)


def test_out_rate_needs_the_scenario_register(tmp_path):
    r = run(FLATTEN, "--card", CARD, "--budget", BUDGET, "--out-card", tmp_path / "c", "--out-register", tmp_path / "r",
            "--out-rate", tmp_path / "rate.yaml")
    assert r.returncode == 2 and "--scenario" in r.stderr
    assert not (tmp_path / "c").exists()


# ---- tau_max, tau_ref ------------------------------------------------------------------------------------------------


def mixer_and_limits(real):
    m, _, idle = mixer.mixer_matrix(real["card"], CARD)
    rotors = real["card"]["rotors"]
    k = rotors["thrust_coeff"]["value"]
    return m, k * idle * idle, k * rotors["speed_range"]["value"][1] ** 2


def exact_gap(m, col, torque, f_min, f_max):
    """max_i lower_i - min_i upper_i of the collective's interval, in exact rationals; <= 0 is feasible."""
    lo = max((Fraction(f_min) - torque * Fraction(row[col])) / Fraction(row[0]) for row in m)
    hi = min((Fraction(f_max) - torque * Fraction(row[col])) / Fraction(row[0]) for row in m)
    return lo - hi


def best_slack(m, col, torque, f_min, f_max):
    """max over the collective c of the smallest slack of f_i = c M[i,thrust] + torque M[i,col] to [f_min, f_max], by
    golden-section search on the concave piecewise-linear function (a different route from the interval form)."""
    def slack(c):
        return min(min(c * r[0] + torque * r[col] - f_min, f_max - c * r[0] - torque * r[col]) for r in m)
    lo, hi = -1e3, 1e3
    inv = (math.sqrt(5) - 1) / 2
    for _ in range(200):
        x1, x2 = hi - inv * (hi - lo), lo + inv * (hi - lo)
        if slack(x1) < slack(x2):
            lo = x1
        else:
            hi = x2
    return slack((lo + hi) / 2)


@pytest.mark.parametrize("axis", AXES)
def test_tau_max_is_feasible_below_and_infeasible_above_by_2_pow_minus_40(real, axis):
    m, f_min, f_max = mixer_and_limits(real)
    col = {"roll": 1, "pitch": 2, "yaw": 3}[axis]
    t = real["res"]["axes"][axis]["tau_max"]
    assert t > 0
    up, down = Fraction(t) * (1 + Fraction(1, 2 ** 40)), Fraction(t) * (1 - Fraction(1, 2 ** 40))
    scale = Fraction(f_max) / Fraction(min(r[0] for r in m))
    for sign in (1, -1):
        assert exact_gap(m, col, sign * down, f_min, f_max) <= 0, sign
        assert exact_gap(m, col, sign * Fraction(t), f_min, f_max) <= scale * Fraction(1, 2 ** 50)
    assert any(exact_gap(m, col, sign * up, f_min, f_max) > 0 for sign in (1, -1))
    tau = real["res"]["axes"][axis]["tau_max"]
    for sign in (1, -1):
        assert best_slack(m, col, sign * tau * (1 - 2.0 ** -30), f_min, f_max) >= 0
    assert any(best_slack(m, col, sign * tau * (1 + 2.0 ** -30), f_min, f_max) < 0 for sign in (1, -1))


def test_tau_ref_is_both_terms_from_independent_recomputation(real):
    res = real["res"]
    b_j, b_tau = bands(real)
    tau, T = res["inputs"]["tau"], res["T"]
    m, f_min, f_max = mixer_and_limits(real)
    rate_max = {a: real["scenario"][f"rate_max_{a}"]["value"] for a in AXES}
    t63s = []
    for axis in AXES:
        kp, ki = emitted(real, axis)
        j = res["axes"][axis]["J"]
        t63s += [simulate_t63(jt, tt, kp, ki, T) for jt, tt in [(j, tau)] + box_corners(j, tau, b_j, b_tau)]
    tau_cl = max(t63s)
    # t63 lands on rate-loop samples. The test evaluates the emitted f32 gains, the generator its double gains, so the
    # two may pick adjacent samples: they must agree within one sample T.
    assert abs(tau_cl - res["tau_cl"]) <= T
    assert max(t for _, t in res["t63"]) == tau_cl
    for axis in AXES:
        v = res["axes"][axis]
        authority = rate_max[axis] / (v["tau_max"] / (v["J"] * (1 + b_j)))
        assert v["authority_term"] == pytest.approx(authority, rel=8 * EPS64)
        e = real["flat"][f"rate_tau_ref_{axis}"]["value"]
        assert e >= tau_cl and e >= authority
        assert e <= max(tau_cl, authority) * (1 + 2 * EPS32)
        assert e == r32(e)


def tau_ref_ok(tau_ref, terms):
    return all(tau_ref >= t for t in terms)


def test_planted_tau_ref_just_below_either_term_fails_the_predicate(real):
    res = real["res"]
    axis = "yaw"
    assert res["tau_cl"] > res["axes"][axis]["authority_term"], "tau_cl binds in the committed design"
    terms = [res["tau_cl"], res["axes"][axis]["authority_term"]]
    good = real["flat"][f"rate_tau_ref_{axis}"]["value"]
    assert tau_ref_ok(good, terms)
    assert not tau_ref_ok(math.nextafter(res["tau_cl"], 0.0), terms)
    assert not tau_ref_ok(res["axes"][axis]["authority_term"], terms)  # the authority term alone is not enough
    # a scenario whose yaw maximum rate is 4x makes the authority term bind: it is then the one that must be met
    scenario = copy.deepcopy(real["scenario"])
    scenario["rate_max_yaw"]["value"] = 4 * scenario["rate_max_yaw"]["value"]
    res2 = rate.design(real["card"], real["budget"], scenario, CARD)
    terms2 = [res2["tau_cl"], res2["axes"]["yaw"]["authority_term"]]
    assert terms2[1] > terms2[0] and tau_ref_ok(res2["axes"]["yaw"]["tau_ref"], terms2)
    assert not tau_ref_ok(math.nextafter(terms2[1], 0.0), terms2)
    assert not tau_ref_ok(terms2[0], terms2)  # without the authority term
    assert res2["axes"]["roll"]["tau_ref"] == res["axes"]["roll"]["tau_ref"]


# ---- flatten: the flag is optional, deterministic ---------------------------------------------------------------------


def test_without_out_rate_the_other_outputs_are_byte_identical_and_no_rate_file_exists(tmp_path):
    assert flatten(tmp_path / "none", rate_out=False).returncode == 0
    assert flatten(tmp_path / "with").returncode == 0
    assert sorted(p.name for p in (tmp_path / "none").iterdir()) == sorted(OTHER_OUTPUTS)
    assert sorted(p.name for p in (tmp_path / "with").iterdir()) == sorted(OTHER_OUTPUTS + ("rate.yaml",
                                                                                         "rate_report.txt"))
    for name in OTHER_OUTPUTS:
        assert (tmp_path / "none" / name).read_bytes() == (tmp_path / "with" / name).read_bytes(), name


def test_rate_outputs_are_byte_stable_across_two_runs(tmp_path):
    assert flatten(tmp_path / "a").returncode == 0
    assert flatten(tmp_path / "b").returncode == 0
    for name in ("rate.yaml", "rate_report.txt"):
        assert (tmp_path / "a" / name).read_bytes() == (tmp_path / "b" / name).read_bytes(), name
