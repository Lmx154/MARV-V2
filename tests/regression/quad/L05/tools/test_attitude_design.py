"""L5 attitude-loop parameters from the vehicle card (tools/card/attitude.py, flatten.py --out-attitude --sim7-u; decision
0006 E and owner decisions 7, 12, 17, 18): rule-property tests on the LIVE generated parameters. No gain value is pinned
(owner decision 7); every check asserts a rule, and each has a negative control that plants a violation of it.

The generator evaluates the lifted loop by F(z)/(z - 1) with the pole at z = 1 taken out analytically. The oracle here is a
different evaluation: the plant is sampled from its state matrices (matrix exponential of the augmented [omega, m, theta,
u] system, physical units, true inertia, the emitted f32 rate gains), the closed rate loop is the 5-state system
[omega, m, theta, I, e_prev], the N-step lifting is a stepping of that system (P by matrix powers, the input column by
simulating N steps from rest), the frequency response comes from a complex linear solve of (zI - P) x = Bs, and the phase
is unwrapped by stepping on a ratio-1.02 grid. Stability is the spectral radius by repeated squaring, not the Jury test.
The yaw release is stepped rate execution by rate execution, not by the lifted matrix.

Numeric tolerance TOL = 2^-30 rad (9.3e-10): both evaluators bisect the crossover to adjacent doubles (error of the PM
|dPM/dw| w 2^-52, below 1e-14 rad) and the solve's rounding is below 1e-12 rad (cond(zI - P) below 1e5 at w = 1e-2 rad/s,
1e-16 x 1e5 x 1e-2); TOL leaves three decades of slack and stays below the generator's delta_num (about 1e-7 rad).

The SIM-7 uncertainty file of these tests is fixtures/u.yaml, a labelled scenario test value with method scenario (see the
file). The product path refuses it (attitude.read_u accepts method scenario only with the test-only allow_scenario flag,
which flatten.py never passes); the tests call attitude.attitude_entries in process with the flag.
"""

import cmath
import functools
import json
import math
import re
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import attitude  # noqa: E402
import rate  # noqa: E402
import schema  # noqa: E402

FLATTEN = ROOT / "tools" / "card" / "flatten.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
FIXTURE_U = Path(__file__).resolve().parent / "fixtures" / "u.yaml"
L05 = ROOT / "tests" / "regression" / "quad" / "L05"

AXES = ("roll", "pitch", "yaw")
CORNER_NAMES = ("nominal", "J-,tau-", "J-,tau+", "J+,tau-", "J+,tau+")
ATT_IDS = ["att_kp", "att_yaw_weight", "att_loop_ratio", "att_yaw_alpha_min", "att_yaw_t_cross"]
TOL = 2.0 ** -30
EPS32 = 2.0 ** -24  # float32 unit roundoff
EPS64 = 2.0 ** -52
SQUARINGS = 16


def run(*args):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True, check=False)


def r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def f32_up(x):
    """The next float32 above x (x a float32 value)."""
    bits = struct.unpack("<I", struct.pack("<f", x))[0]
    return struct.unpack("<f", struct.pack("<I", bits + 1))[0]


def f32_down_rule(x):
    y = r32(x)
    return y if y <= x else struct.unpack("<f", struct.pack("<I", struct.unpack("<I", struct.pack("<f", y))[0] - 1))[0]


# ---- the independent evaluation ---------------------------------------------------------------------------------------


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
    """(Ad, Bd) of [omega, m, theta]: omega' = m/J, m' = (u - m)/tau, theta' = omega, under zero-order-hold u."""
    e = expm([[0.0, T / j_true, 0.0, 0.0], [0.0, -T / tau, 0.0, T / tau], [T, 0.0, 0.0, 0.0], [0.0] * 4])
    return tuple(tuple(e[i][:3]) for i in range(3)), tuple(e[i][3] for i in range(3))


@functools.lru_cache(maxsize=None)
def closed_step(j_true, tau, kp, ki, T):
    """(A, B) of x+ = A x + B r, x = [omega, m, theta, I, e_prev]: u = kp (r - omega) + I + ki T e_prev (I+), physical."""
    ad, bd = sampled_plant(j_true, tau, T)
    cu, cr = [-kp, 0.0, 0.0, 1.0, ki * T], kp
    a = [[(ad[i][c] if c < 3 else 0.0) + bd[i] * cu[c] for c in range(5)] for i in range(3)]
    a.append([0.0, 0.0, 0.0, 1.0, ki * T])
    a.append([-1.0, 0.0, 0.0, 0.0, 0.0])
    return a, [bd[0] * cr, bd[1] * cr, bd[2] * cr, 0.0, 1.0]


def step(a, b, x, r):
    return [sum(a[i][k] * x[k] for k in range(5)) + b[i] * r for i in range(5)]


@functools.lru_cache(maxsize=None)
def lifted(j_true, tau, kp, ki, T, n):
    """(P, Bs): x_(k+1)N = P x_kN + Bs r, P = A^N by matrix products, Bs by stepping N executions from rest with r = 1."""
    a, b = closed_step(j_true, tau, kp, ki, T)
    p = [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
    x = [0.0] * 5
    for _ in range(n):
        p = matmul(a, p)
        x = step(a, b, x, 1.0)
    return p, x


def csolve(a, b):
    n = len(b)
    m = [list(a[i]) + [b[i]] for i in range(n)]
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


def gn(p, bs, theta):
    """G_N(e^{j theta}) = [(zI - P)^-1 Bs]_theta."""
    z = cmath.exp(1j * theta)
    a = [[(z if i == j else 0) - p[i][j] for j in range(5)] for i in range(5)]
    return csolve(a, bs)[2]


def spectral_radius(p, bs, k):
    """rho of P - k Bs e_theta^T by repeated squaring of the normalised matrix (error factor below 1 + 1e-3 here)."""
    m = [row[:] for row in p]
    for i in range(5):
        m[i][2] -= k * bs[i]
    log_scale = 0.0
    for _ in range(SQUARINGS):
        m = matmul(m, m)
        s = max(sum(abs(x) for x in row) for row in m)
        m = [[x / s for x in row] for row in m]
        log_scale = 2 * log_scale + math.log(s)
    return math.exp(log_scale / 2 ** SQUARINGS)


def phase_margin(p, bs, k, t_a):
    """(PM, crossover rad/s) of L = k G_N; None when the crossover is not unique on a ratio-1.02 grid from 1e-2 rad/s to
    Nyquist, or the loop is unstable. The phase is unwrapped by stepping from the first grid point."""
    if spectral_radius(p, bs, k) >= 1:
        return None
    ws = [1e-2]
    while ws[-1] * 1.02 < math.pi / t_a:
        ws.append(ws[-1] * 1.02)
    mags = [abs(k * gn(p, bs, w * t_a)) for w in ws]
    changes = [i for i in range(len(ws) - 1) if (mags[i] > 1) != (mags[i + 1] > 1)]
    if len(changes) != 1 or not mags[0] > 1:
        return None
    lo, hi = ws[changes[0]], ws[changes[0] + 1]
    for _ in range(200):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        if abs(k * gn(p, bs, mid * t_a)) > 1:
            lo = mid
        else:
            hi = mid
    wx = (lo + hi) / 2
    prev = cmath.phase(k * gn(p, bs, ws[0] * t_a))
    for w in [x for x in ws[1:] if x < wx] + [wx]:
        ph = cmath.phase(k * gn(p, bs, w * t_a))
        prev += (ph - cmath.phase(cmath.exp(1j * prev)) + math.pi) % (2 * math.pi) - math.pi
    return math.pi + prev, wx


# ---- fixtures -------------------------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def real(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("attitude")
    r = run(FLATTEN, "--card", CARD, "--budget", BUDGET, "--out-card", tmp / "card.yaml", "--out-register",
            tmp / "register.yaml", "--out-mixer", tmp / "mixer.yaml", "--root", ROOT, "--scenario", SCENARIO,
            "--out-scenario", tmp / "scenario.yaml", "--out-rate", tmp / "rate.yaml")
    assert r.returncode == 0, r.stderr
    card, budget, scenario = (schema.load_yaml(p) for p in (CARD, BUDGET, SCENARIO))
    # The product path (flatten.py) refuses the scenario-method fixture, so the attitude generator is called in process
    # with the test-only flag; flatten.py runs the same attitude_entries on the committed measured file.
    u = attitude.read_u(FIXTURE_U, allow_scenario=True)
    entries, report, res = attitude.attitude_entries(card, budget, scenario, CARD, u, "fixtures/u.yaml")
    (tmp / "attitude_report.txt").write_text("\n".join(report) + "\n", encoding="utf-8")
    return {"res": res, "flat": dict(entries), "rate_flat": schema.load_yaml(tmp / "rate.yaml"),
            "dir": tmp, "card": card, "budget": budget, "u": u["value"], "oracle": {}, "q": {}}


def report_text(real):
    return (real["dir"] / "attitude_report.txt").read_text(encoding="utf-8")


def live(real, name):
    return real["flat"][name]["value"]


def corners(real):
    i = real["res"]["inputs"]
    return [("nominal", 1.0, i["tau"]), ("J-,tau-", 1 - i["b_J"], (1 - i["b_tau"]) * i["tau"]),
            ("J-,tau+", 1 - i["b_J"], (1 + i["b_tau"]) * i["tau"]),
            ("J+,tau-", 1 + i["b_J"], (1 - i["b_tau"]) * i["tau"]),
            ("J+,tau+", 1 + i["b_J"], (1 + i["b_tau"]) * i["tau"])]


def rate_gains(real, axis):
    f = real["rate_flat"]
    return f[f"rate_kp_{axis}"]["value"], f[f"rate_ki_{axis}"]["value"]


def inertia(real, axis):
    return real["res"]["rate"]["axes"][axis]["J"]


def oracle_loops(real, n):
    """[(axis, corner, P, Bs)] of the 15 loops on the emitted f32 rate gains, physical units."""
    T = real["res"]["T"]
    out = []
    for axis in AXES:
        kp, ki = rate_gains(real, axis)
        for name, jt, tau in corners(real):
            p, bs = lifted(inertia(real, axis) * jt, tau, kp, ki, T, n)
            out.append((axis, name, p, bs))
    return out


def oracle_worst(real, n, k, k_yaw=None):
    """(PM_worst, [(axis, corner, PM)]); PM_worst is -inf when a loop has no unique crossover or is unstable."""
    k_yaw = k if k_yaw is None else k_yaw
    t_a = n * real["res"]["T"]
    detail, worst = [], math.inf
    for axis, name, p, bs in oracle_loops(real, n):
        m = phase_margin(p, bs, k_yaw if axis == "yaw" else k, t_a)
        detail.append((axis, name, None if m is None else m[0]))
        worst = -math.inf if m is None or worst == -math.inf else min(worst, m[0])
    return worst, detail


def w32_of(real):
    return live(real, "att_yaw_weight")


def k_yaw_effective(k32, w32):
    return r32(k32 / w32) * w32


def oracle_live_worst(real, scale=1.0):
    k32, w32 = live(real, "att_kp") * scale, w32_of(real)
    return oracle_worst(real, live(real, "att_loop_ratio"), k32, k_yaw_effective(k32, w32))


# ---- the emitted parameters ---------------------------------------------------------------------------------------


def test_the_five_derived_ids_are_emitted_in_order_with_the_designers_values_and_the_l05_manifest_lists_them(real):
    f, res = real["flat"], real["res"]
    assert list(f) == ATT_IDS
    ids = [i for i in (L05 / "param_ids").read_text().split() if not i.startswith("#")]
    for name in ATT_IDS + ["angle_tilt_max", "yaw_deadband"]:
        assert ids.count(name) == 1, name
    assert f["att_kp"]["value"] == res["k32"] == r32(res["k32"]) and f["att_kp"]["type"] == "f32"
    assert f["att_yaw_weight"]["value"] == res["w32"] and f["att_yaw_weight"]["type"] == "f32"
    assert f["att_loop_ratio"]["value"] == res["N"] and f["att_loop_ratio"]["type"] == "i32"
    assert f["att_loop_ratio"]["sigma"] == 0
    assert f["att_yaw_alpha_min"]["value"] == res["alpha_min"] and f["att_yaw_t_cross"]["value"] == res["t_cross"]
    for name in ("att_kp", "att_yaw_weight", "att_yaw_alpha_min", "att_yaw_t_cross"):
        assert f[name]["sigma"] == schema.UNKNOWN and f[name]["method"].startswith("derived("), name
    assert [f[n]["unit"] for n in ATT_IDS] == ["1/s", "1", "1", "rad/s^2", "s"]


# ---- margins ----------------------------------------------------------------------------------------------------------


def margin_ok(real, scale=1.0):
    """PM_worst of the 15 loops >= PM_min - delta_num on the emitted f32 gains (yaw at its effective gain), by the oracle."""
    worst, _ = oracle_live_worst(real, scale)
    return worst >= real["res"]["inputs"]["PM_min"] - real["res"]["final"]["delta_num"]


def test_worst_corner_pm_of_every_axis_meets_pm_min_minus_delta_num_and_agrees_with_the_generator(real):
    res = real["res"]
    worst, detail = oracle_live_worst(real)
    assert margin_ok(real)
    assert len(detail) == 15 and [d[1] for d in detail] == list(CORNER_NAMES) * 3
    for got, (axis, name, pm, _, radius, _) in zip(detail, res["final"]["detail"]):
        assert (got[0], got[1]) == (axis, name)
        assert abs(got[2] - pm) <= TOL, (axis, name, got[2] - pm)
        assert radius < 1
    assert worst >= res["inputs"]["PM_min"] + res["final"]["delta_num"] - TOL
    assert abs(worst - res["final"]["pm_worst"]) <= TOL


def test_control_k_times_1_1_fails_the_margin_check(real):
    assert not margin_ok(real, scale=1.1)
    worst, _ = oracle_live_worst(real, 1.1)
    assert worst < real["res"]["inputs"]["PM_min"] - 1e-3


def yaw_compensation_holds(k32, w32, k_eff):
    """|k_eff - f32(k)| <= one f32 rounding of f32(k): f32(k/w) carries one rounding, the product is taken exactly."""
    return abs(k_eff - k32) <= EPS32 * k32


def test_the_yaw_effective_gain_equals_f32_k_within_the_f32_bound(real):
    k32, w32 = live(real, "att_kp"), w32_of(real)
    assert yaw_compensation_holds(k32, w32, k_yaw_effective(k32, w32))
    assert real["res"]["k_yaw_effective"] == k_yaw_effective(k32, w32)


def test_control_an_uncompensated_yaw_gain_fails_the_compensation_check(real):
    k32, w32 = live(real, "att_kp"), w32_of(real)
    assert not yaw_compensation_holds(k32, w32, w32 * k32)


# ---- tightness --------------------------------------------------------------------------------------------------------


def feasible_pair_end(real, k):
    """The generator's own feasibility test of a bracket end, by the oracle: all 15 loops at the double gain k (yaw too) have
    a unique crossover, are stable and have PM >= PM_min."""
    worst, _ = oracle_worst(real, live(real, "att_loop_ratio"), k, k)
    return worst >= real["res"]["inputs"]["PM_min"]


def tight(real, hi_scale=1.0):
    """The emitted k is tight against the generator's recorded final (feasible lo, infeasible hi) pair: hi, the next step
    above the feasible end, is infeasible by the oracle, and so is k (1 + (hi - k)/k) = hi."""
    lo, hi = real["res"]["final"]["bracket"]
    k = live(real, "att_kp")
    return feasible_pair_end(real, lo) and not feasible_pair_end(real, k * (1 + (hi * hi_scale - k) / k))


def test_k_times_one_plus_the_gap_to_the_recorded_infeasible_end_is_infeasible_and_the_pair_is_tight(real):
    f = real["res"]["final"]
    lo, hi = f["bracket"]
    k = live(real, "att_kp")
    assert tight(real)
    assert hi - lo == f["bracket_width"] and hi - lo <= 2 * (f32_up(k) - k), "the pair is resolved to about one f32 step"


def test_every_history_point_above_the_emitted_k_fails_the_guard_so_k_is_the_largest_passing_point(real):
    """The step-back is exactly the guard's: the emitted k is history[-1 - steps_back] and each later point of the history
    has PM_worst(f32 gains) < PM_min + delta_num (by the oracle), so nothing larger passed."""
    f = real["res"]["final"]
    steps = f["steps_back"]
    assert f["stepped_down"] == (steps > 0)
    assert r32(f["history"][len(f["history"]) - 1 - steps]) == live(real, "att_kp")
    n = live(real, "att_loop_ratio")
    for h in f["history"][len(f["history"]) - steps:]:
        k32 = r32(h)
        worst, _ = oracle_worst(real, n, k32, k_yaw_effective(k32, w32_of(real)))
        assert worst < real["res"]["inputs"]["PM_min"] + f["delta_num"] + TOL, h


def test_control_a_pair_end_moved_below_the_rule_is_not_tight(real):
    assert not tight(real, hi_scale=0.9)


# ---- the yaw weight ---------------------------------------------------------------------------------------------------


def report_alphas(real):
    """alpha_max of roll, pitch, yaw from the rate report the same flatten run wrote (repr of the doubles)."""
    text = (real["dir"] / "rate_report.txt").read_text(encoding="utf-8")
    out = {}
    for axis, val in re.findall(r"^axis (\w+)\n(?:  .*\n)*?  alpha_max \(rad/s\^2\)\s+(\S+)$", text, flags=re.M):
        out[axis] = float(val)
    assert set(out) == set(AXES)
    return out


def w_rule(alpha):
    return r32(min(1.0, alpha["yaw"] / min(alpha["roll"], alpha["pitch"])))


def test_w_is_the_clamped_ratio_of_the_yaw_alpha_to_the_smaller_tilt_alpha_of_the_rate_report(real):
    alpha = report_alphas(real)
    assert w32_of(real) == w_rule(alpha)
    assert 0 < w32_of(real) <= 1
    assert alpha["yaw"] / min(alpha["roll"], alpha["pitch"]) == pytest.approx(real["res"]["w_raw"], rel=8 * EPS64)


def test_control_planted_weight_rules_do_not_reproduce_the_emitted_w(real):
    alpha = report_alphas(real)
    live_w = w32_of(real)
    assert live_w != r32(alpha["yaw"] / max(alpha["roll"], alpha["pitch"]))
    assert live_w != r32(alpha["yaw"] / alpha["roll"])
    assert live_w != r32(min(1.0, min(alpha["roll"], alpha["pitch"]) / alpha["yaw"]))
    assert live_w != 1.0


# ---- the release quantities --------------------------------------------------------------------------------------------


def alpha_rule(real):
    """alpha_max,yaw from the rate report's tau_max,yaw, the card's J_yaw and the budget's band."""
    text = (real["dir"] / "rate_report.txt").read_text(encoding="utf-8")
    block = re.search(r"^axis yaw\n(?:  .*\n)*", text, flags=re.M).group(0)
    tau_max = float(re.search(r"tau_max \(N m\)\s+(\S+)", block).group(1))
    return tau_max / (inertia(real, "yaw") * (1 + real["budget"]["inertia_robustness_band"]["value"]))


def alpha_ok(real, value, rule=None):
    """value is the largest f32 at or below the rule, and the rule equals the rate report's alpha_max,yaw."""
    rule = alpha_rule(real) if rule is None else rule
    return value == r32(value) and value <= rule < f32_up(value)


def test_att_yaw_alpha_min_is_the_rule_rounded_down_and_equals_the_rate_reports_alpha_max_yaw_rounded_down(real):
    rule = alpha_rule(real)
    assert rule == pytest.approx(report_alphas(real)["yaw"], rel=8 * EPS64)
    assert alpha_ok(real, live(real, "att_yaw_alpha_min"), rule)
    assert live(real, "att_yaw_alpha_min") == f32_down_rule(report_alphas(real)["yaw"])


def test_control_planted_alpha_min_values_fail_the_rule(real):
    v = live(real, "att_yaw_alpha_min")
    rule = alpha_rule(real)
    assert not alpha_ok(real, f32_up(v), rule), "rounded up past the rule"
    assert not alpha_ok(real, 0.99 * v, rule), "not f32, not the largest"
    j, b = inertia(real, "yaw"), real["budget"]["inertia_robustness_band"]["value"]
    assert not alpha_ok(real, r32(rule * (1 + b)), rule), "J without the band: the wrong side of the rule"
    assert not alpha_ok(real, f32_down_rule(rule * (1 + b)), rule)
    assert j > 0


def crossing_time(real, j_true, tau, n):
    """First attitude execution (every n rate executions, k >= 1) at which the yaw rate of the physical closed loop, released
    from 1 rad/s with r = 0, is <= 0, stepped rate execution by rate execution: its time (s)."""
    kp, ki = rate_gains(real, "yaw")
    T = real["res"]["T"]
    a, b = closed_step(j_true, tau, kp, ki, T)
    x = [1.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(1, attitude.BRAKE_SCAN_EXECUTIONS * n + 1):
        x = step(a, b, x, 0.0)
        if i % n == 0 and x[0] <= 0:
            return i * T
    raise AssertionError("no crossing")


def grid_points(real, m=5):
    i = real["res"]["inputs"]
    j_a = inertia(real, "yaw")
    return [(j_a * (1 + i["b_J"] * (2 * a / (m - 1) - 1)), i["tau"] * (1 + i["b_tau"] * (2 * c / (m - 1) - 1)))
            for a in range(m) for c in range(m)]


def t_cross_ok(real, value, points):
    return all(value >= crossing_time(real, j, tau, live(real, "att_loop_ratio")) * (1 - 4 * EPS64)
               for j, tau in points)


def test_t_cross_is_the_latest_corner_release_crossing_rounded_up_and_bounds_a_5x5_box_grid(real):
    n = live(real, "att_loop_ratio")
    t = live(real, "att_yaw_t_cross")
    j_a = inertia(real, "yaw")
    corner_times = [crossing_time(real, j_a * jt, tau, n) for _, jt, tau in corners(real)]
    latest = max(corner_times)
    assert t == r32(t) and t >= latest * (1 - 4 * EPS64)
    assert t - latest < EPS32 * 2 * latest, "rounded up by at most one f32 step"
    assert t_cross_ok(real, t, [(j_a * jt, tau) for _, jt, tau in corners(real)])
    assert t_cross_ok(real, t, grid_points(real))
    for (name, k, tm), ct in zip(real["res"]["crossings"], corner_times):
        assert tm == pytest.approx(ct, rel=8 * EPS64), name
    assert real["res"]["t_cross"] == t


def test_control_a_smaller_planted_t_cross_fails(real):
    n = live(real, "att_loop_ratio")
    t = live(real, "att_yaw_t_cross")
    j_a = inertia(real, "yaw")
    pts = [(j_a * jt, tau) for _, jt, tau in corners(real)]
    t_a = n * real["res"]["T"]
    assert not t_cross_ok(real, t - t_a, pts), "one attitude period early"
    assert not t_cross_ok(real, 0.5 * t, pts)
    nominal = crossing_time(real, j_a, real["res"]["inputs"]["tau"], n)
    assert nominal < t and not t_cross_ok(real, nominal, pts), "nominal alone is not the latest corner"


# ---- SIM-7 -----------------------------------------------------------------------------------------------------------


def q_of(real, n):
    """q(N) = PM_worst(f32(k_ref), N) by the oracle, or None; k_ref is the generator's N = 1 rule (its f32 value)."""
    if n not in real["q"]:
        k = real["res"]["ref"]["k32"]
        worst, _ = oracle_worst(real, n, k, k_yaw_effective(k, w32_of(real)))
        real["q"][n] = None if worst == -math.inf else worst
    return real["q"][n]


def sim7_ratio(qs, u):
    """N* from the rule: qs is [(N, q or None)] for N = 1, 2, 4, ...; Delta_i = |q_i - q_(i-1)|, i* the largest i with
    Delta_j < U for every j <= i, the scan stopping at the first failure or a None."""
    n_star = 1
    for (_, q0), (n, q1) in zip(qs, qs[1:]):
        if q0 is None or q1 is None or abs(q1 - q0) >= u:
            break
        n_star = n
    return n_star


def oracle_ratio(real, u):
    qs, i = [(1, q_of(real, 1))], 0
    while True:
        i += 1
        qs.append((2 ** i, q_of(real, 2 ** i)))
        if qs[-1][1] is None or abs(qs[-1][1] - qs[-2][1]) >= u:
            return sim7_ratio(qs, u)


def table_obeys_rule(table, u, n_star):
    """The generator's table: Delta_i = |q_i - q_(i-1)|, each accepted row has Delta < U, the scan ends at the first row with
    Delta >= U (or without a q), and N* is the last accepted N."""
    accepted = 1
    for prev, row in zip(table, table[1:]):
        if row["q"] is None:
            return row is table[-1] and accepted == n_star
        if abs(row["delta"] - abs(row["q"] - prev["q"])) > 1e-15:
            return False
        if row["delta"] < u:
            accepted = row["N"]
            if row is table[-1]:
                return False
        else:
            return row is table[-1] and accepted == n_star
    return accepted == n_star


def test_the_sim7_table_satisfies_its_own_rule_and_the_oracles_q_reproduce_it(real):
    table, u, n = real["res"]["table"], real["u"], live(real, "att_loop_ratio")
    assert table_obeys_rule(table, u, n)
    assert [row["N"] for row in table] == [2 ** i for i in range(len(table))]
    for row in table:
        assert row["q"] is not None and abs(q_of(real, row["N"]) - row["q"]) <= TOL, row["N"]
    text = report_text(real)
    for row in table:
        assert re.search(rf"^  {row['N']} +\S+ +{math.degrees(row['q']):.6f}", text, flags=re.M), row
    assert "N* = " in text and str(n) in text


def test_the_sim7_ratio_recomputed_by_the_oracle_is_the_emitted_att_loop_ratio(real):
    assert oracle_ratio(real, real["u"]) == live(real, "att_loop_ratio")


def test_control_planted_uncertainties_and_tables_move_the_ratio_or_break_the_rule(real):
    n = live(real, "att_loop_ratio")
    assert oracle_ratio(real, real["u"] / 4) != n, "a smaller U lowers the rate"
    assert oracle_ratio(real, real["u"] * 4) != n, "a larger U raises it"
    table = [dict(row) for row in real["res"]["table"]]
    table[1]["q"] += real["u"]
    assert not table_obeys_rule(table, real["u"], n)
    assert not table_obeys_rule(real["res"]["table"], real["u"], 2 * n)
    assert table_obeys_rule(real["res"]["table"], real["u"], n)


def test_the_report_prints_the_sim7_halving_table_with_u_its_source_and_the_final_n(real):
    text = report_text(real)
    for token in ("SIM-7", "q(N) = PM_worst(k_ref)", "Delta deg", "verdict", "source file", "U rule", "N* ",
                  "att_yaw_alpha_min", "att_yaw_t_cross", "delta_num", "stepped down", "Jury radius", "yaw effective gain"):
        assert token in text, token
    assert repr(real["u"]) in text


# ---- the product path refuses a scenario-method U ------------------------------------------------------------------------


def test_control_read_u_without_the_test_flag_and_flatten_refuse_a_scenario_method_u_file(tmp_path):
    with pytest.raises(attitude.gpc.GenError, match="method must be measured"):
        attitude.read_u(FIXTURE_U)
    assert attitude.read_u(FIXTURE_U, allow_scenario=True)["value"] == 0.001
    r = run(FLATTEN, "--card", CARD, "--budget", BUDGET, "--out-card", tmp_path / "card.yaml", "--out-register",
            tmp_path / "register.yaml", "--root", ROOT, "--scenario", SCENARIO, "--out-scenario",
            tmp_path / "scenario.yaml", "--out-attitude", tmp_path / "attitude.yaml", "--sim7-u", FIXTURE_U)
    assert r.returncode == 1 and "method must be measured" in r.stderr, r.stdout + r.stderr
    assert not (tmp_path / "attitude.yaml").exists() and not (tmp_path / "card.yaml").exists()


def test_the_committed_measured_u_file_is_accepted_by_the_product_path():
    u = attitude.read_u(ROOT / "design" / "measured" / "sim7_u" / "u.yaml")
    assert u["method"] == "measured" and u["value"] > 0
