"""L6 stage (c) rate-loop gain rule (tools/card/rate_lead.py; decision 0014 "the design", owner decisions 3, 6 and 8): the
cross-check that the rule extends rate.py (N = 1, H = 1, latency 0), N* on the floors with its control, the noise model's
linearity and the T_ff control, the physical J corners, the f32 guard with its control, the floors over the
configuration set (rule step 13) with its two controls, and the closed-loop stability of every loop of that set (decision 0014
fifth round item 4) with the gain-margin bracket as its control.

Every acceptance test has a control that breaks it (core 7.2). Tolerances are derived on their lines. The committed-card
design runs once (module fixture, about 45 s of plain Python).
"""

import cmath
import math
import multiprocessing
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import attitude_lead as al  # noqa: E402
import gen_imu_config as gic  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import rate  # noqa: E402
import rate_lead as rl  # noqa: E402
import schema  # noqa: E402
import test_rate_lead_checks as checks  # noqa: E402  (this directory; pytest's default import mode puts it on sys.path)

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"

# Cross-check tolerance: test_rate_design.py's TOL (2^-30 rad), also test_gyro_chain_design.py's cross-check 1. Both
# evaluators are exact to double rounding (observed 3e-14 rad and 2e-14 relative on the gains); 2^-30 leaves four decades.
TOL = 2.0 ** -30
# The T_ff control's factor (the packet's labelled value): 0.9 T_ff must break the budget.
T_FF_CONTROL = 0.9
# The noise linearity control's factor: the IMU noise density x 1.1.
DENSITY_CONTROL = 1.1
# The f32 guard control: a gain perturbed by 1e-3 relative in the direction that crosses the floors (up: the design sits
# on the sup boundary of the crossover, so more gain moves the binding corner past PM_min and Ms_max).
GUARD_CONTROL = 1e-3
# The configuration-set controls (rule step 13): the packet's labelled gain factor on kp, ki and kd of every axis, and the
# planted PM's distance below PM_min (rad; any positive value is a violation, this one is far above the evaluation's rounding).
GAIN_SET_CONTROL = 1.1
PLANTED_BELOW = 1e-6
# The stability control: the gain scaled by the loop's gain margin divided and multiplied by this labelled factor (the
# LIMIT_FACTOR of test_attitude_lead.py), which must be stable below and unstable above.
GAIN_MARGIN_CONTROL = 1.1


def documents():
    return rl.load(CARD, BUDGET, SCENARIO, PROFILE)


@pytest.fixture(scope="module")
def lead(rate_lead_design):
    return rate_lead_design


@pytest.fixture(scope="module")
def l4():
    card, budget, scenario, _ = documents()
    return rate.design(card, budget, scenario, CARD)


# ---- the cross-check: N = 1, H = 1, latency 0 is rate.py ------------------------------------------------------------


def test_n1_without_chain_or_latency_reproduces_rate_py(l4):
    inp = l4["inputs"]
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    divisor = schema.load_yaml(SCENARIO)["rate_loop_divisor"]["value"]
    model = rl.Model(l4["T"] / divisor, divisor, 0, None, corners, l4["a"], inp["inertia"])
    sup = rl.sup_crossover(model, 1.0, inp["PM_min"])
    # The same sup rule, decision for decision: rate.py's scan bracket, bisection count and final width, bit for bit.
    assert sup["scan_bracket"] == l4["scan_bracket"]
    assert sup["bisections"] == l4["bisections"]
    assert sup["width"] == l4["bracket_width"]
    # rate.py's chosen point is on this rule's history (its guard, PM_f32 >= PM_min + delta_num on the LTI loop, stepped
    # down one point); the guards differ (rule step 8: the firmware loop, owner decision 6), the sup rule does not.
    assert l4["stepped_down"] and sup["history"][-2] == l4["omega_c"]
    # Gains and margins at rate.py's point.
    g = model.gains(l4["omega_c"], 1.0)
    assert g[2] == 0.0
    assert abs(g[0] / l4["kappa_p"] - 1) <= TOL and abs(g[1] / l4["kappa_i"] - 1) <= TOL
    for axis, (kp, ki, kd, _) in zip(rate.AXES, rl.f32_axes(model, g)):
        assert (kp, ki, kd) == (l4["axes"][axis]["kp"], l4["axes"][axis]["ki"], 0.0)
    mine = model.margins(g)
    assert len(mine) == len(l4["margins"]) == 5
    for (name, pm, wx, xs), (rname, rpm, rwx, _) in zip(mine, l4["margins"]):
        assert name == rname and len(xs) == 1
        assert abs(pm - rpm) <= TOL, (name, pm - rpm)
        assert abs(wx / rwx - 1) <= TOL, (name, wx / rwx - 1)
    # The step response of rule step 11 reduces to rate.py's rise_time sample for sample.
    for (name, jt, tau), (rname, t63) in zip(corners, l4["t63"]):
        assert name == rname and rl.rise_time(model, g, jt, tau) == t63


def test_cross_check_control_one_sample_of_latency_breaks_it(l4):
    inp = l4["inputs"]
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    model = rl.Model(l4["T"] / 2, 2, 1, None, corners, l4["a"], inp["inertia"])
    g = model.gains(l4["omega_c"], 1.0)
    # A pure delay moves the phase, not |L| (K changes by 1e-12 only, through the aliasing sum): the margins are the control.
    for (_, pm, _, _), (_, rpm, _, _) in zip(model.margins(g), l4["margins"]):
        assert abs(pm - rpm) > 1000 * TOL


# ---- N* -------------------------------------------------------------------------------------------------------------


def test_n_star_sits_on_the_floors_and_one_step_up_violates(lead):
    i, model = lead["inputs"], lead["model"]
    pm_min, ms_max = i["PM_min"], i["Ms_max"]
    # Root tolerance of PM = PM_min: the sup bracket's infeasible end hi has PM below PM_min, so PM(w_c) - PM_min is at
    # most |dPM/dw| (hi - w_c) (w_c the guarded point, at or below the feasible end). The slope by a central difference of
    # step 2^-20 w_c (rate.py's DERIVATIVE_LOG2_STEP); x 2 covers curvature over a bracket of 1e-7 relative.
    n, w = lead["n_star"], lead["omega_c"]
    hi = lead["sup"]["omega"] + lead["sup"]["width"]
    step = w * 2.0 ** rate.DERIVATIVE_LOG2_STEP
    slope = abs(rl.worst(model.margins(model.gains(w + step, n)))[1]
                - rl.worst(model.margins(model.gains(w - step, n)))[1]) / (2 * step)
    gap = lead["pm_worst"][1] - pm_min
    assert 0 <= gap <= 2 * slope * (hi - w), (gap, slope, hi - w)
    assert lead["ms_worst"][1] <= ms_max
    assert lead["noise_floor_max"] <= lead["budget_n"]
    # test_rate_lead_checks.py evaluates the rule's gains at these values without the search: they must be its output.
    assert (n, w, lead["t_ff"]) == (checks.N_STAR, checks.OMEGA_C, checks.T_FF)
    # The fixture ran the N search in worker processes (default procs): its N* record is the serial evaluation's, exactly.
    assert rl.evaluate_n(model, lead["noise"], lead["n_lo"]["n"], pm_min, lead["ops"]) == lead["n_lo"]
    # The bisection bracket is within the labelled step, so N* (1 + step) is past the boundary: the control.
    assert lead["n_hi"]["n"] / lead["n_lo"]["n"] <= 1 + rl.N_REL_STEP
    up = rl.evaluate_n(model, lead["noise"], n * (1 + rl.N_REL_STEP), pm_min, lead["ops"])
    assert up["ms"] > ms_max or up["floor"] > lead["budget_n"]
    # The rule's monotonicity premise held on the grid (design() refuses otherwise), and the grid reaches an octave of N
    # past the bracket (N_MONOTONE_EXTRA points of 2^(1/4)), so past 2 N*.
    grid = lead["grid"]
    assert all(b["ms"] >= a["ms"] and b["floor"] >= a["floor"] for a, b in zip(grid, grid[1:]))
    assert grid[-1]["n"] >= 2 * n


# ---- noise ----------------------------------------------------------------------------------------------------------


def test_noise_is_linear_in_the_density_and_t_ff_is_the_smallest_within_budget(lead):
    i, noise = lead["inputs"], lead["noise"]
    kd32, tf32 = [a[2] for a in lead["axes32"]], lead["axes32"][0][3]
    sc = schema.load_yaml(SCENARIO)
    num, den = sc["tick_period_num_us"]["value"], sc["tick_period_den"]["value"]
    si, _, _ = gic.imu_config(PROFILE)
    assert i["sigma_d"] == gic.derived_report(si, num, den)[0]["gyro"][4]
    scaled = dict(si, gyro_noise_density=si["gyro_noise_density"] * DENSITY_CONTROL)
    sigma = gic.derived_report(scaled, num, den)[0]["gyro"][4]
    w0, fw = lead["noise_max_w0"], (lead["dts"], -rl.LIBM_EXP_ULPS)   # the firmware model T_ff is set on (rule step 9)
    base = noise.rms(None, w0, kd32, tf32, lead["t_ff"], fw=fw)
    up = noise.rms(None, w0, kd32, tf32, lead["t_ff"], sigma_d=sigma, fw=fw)
    for key in ("d", "ff", "combined"):
        for a, b in zip(base[key], up[key]):
            assert b == pytest.approx(DENSITY_CONTROL * a, rel=1e-12)   # a product of doubles: rounding only

    def combined(t_ff):
        return max(max(noise.rms(sp, w, kd32, tf32, t_ff, fw=fw)["combined"]) for sp, w in lead["ops"])

    assert combined(lead["t_ff"]) <= lead["budget_n"]
    assert combined(T_FF_CONTROL * lead["t_ff"]) > lead["budget_n"]


# ---- the physical J corners -----------------------------------------------------------------------------------------


def test_physical_corners_are_physical_and_keep_the_common_scale_ones():
    card, budget, _, _ = documents()
    j0 = card["inertia_diag"]["value"]
    b = budget["inertia_robustness_band"]["value"]
    tol = rl.CORNER_TOL_ULPS * sys.float_info.epsilon * sum(j0)
    vs = rl.physical_corners(j0, b)
    pts = [v["J"] for v in vs]
    for p in pts:
        assert rl.triangle_ok(p, tol), p
        assert all((1 - b) * j - tol <= x <= (1 + b) * j + tol for x, j in zip(p, j0)), p
    bad = tuple(f * j for f, j in zip((1 - b, 1 - b, 1 + b), j0))     # (0.5, 0.5, 1.5) J
    assert not rl.triangle_ok(bad, tol) and bad not in pts
    lo, hi = tuple((1 - b) * j for j in j0), tuple((1 + b) * j for j in j0)
    assert lo in pts and hi in pts
    assert {v["J"] for v in vs if v["common_scale"]} == {lo, hi}
    # Every box vertex that satisfies the inequalities is kept; the other box vertices are not.
    for signs in rl.SIGN_PATTERNS:
        corner = tuple((1 + s * b) * j for s, j in zip(signs, j0))
        assert (corner in pts) == rl.triangle_ok(corner, tol), corner


# ---- the f32 guard --------------------------------------------------------------------------------------------------


def test_f32_guard_passes_and_a_gain_beyond_it_fails(lead):
    i, model = lead["inputs"], lead["model"]
    axes32 = lead["axes32"]
    assert all(rate.r32(x) == x for a in axes32 for x in a)
    assert lead["dts"] == [312, 313]     # the SIL's stamps at 625/4 us, D = 2: the executions alternate (rule step 8)
    rows = lead["guard"]
    assert len(rows) == len(axes32)
    assert rl.guard_passes(rows, i["PM_min"], i["Ms_max"])
    for idx in range(len(axes32)):
        bad = [list(a) for a in axes32]
        bad[idx][0] *= 1 + GUARD_CONTROL
        assert not rl.guard_passes(rl.guard(model, [tuple(a) for a in bad], lead["dts"]), i["PM_min"], i["Ms_max"])


# ---- the configuration set (rule step 13) -----------------------------------------------------------------------------


def test_the_f32_design_holds_the_floors_over_the_configuration_set_and_planted_violations_break_it(lead):
    """Controls (core 7.2): (a) the packet's planted gain x 1.1, which must fail the set assertion; it fails at step 1''s
    point too, so on its own it cannot show that the other configurations are looked at. Hence (b): a PM below PM_min
    planted at one configuration other than step 1''s point (notches bypassed, latency 0, the stage (c) T4 configuration)
    must fail the verdict while step 1''s own rows still pass."""
    i, cset, model, chain = lead["inputs"], lead["set"], lead["model"], lead["chain"]
    pm_min, ms_max = i["PM_min"], i["Ms_max"]
    rows = cset["rows"]
    # The set evaluated: the resolved level's halving, step 1''s point first with step 8's guard rows, the bypassed
    # configurations and both latencies among them.
    assert [r[0] for r in rows] == [c[0] for c in rl.configuration_set(chain, model.latency, cset["level"] + 1)]
    assert rows[0][0] == (0.0, model.latency) and rows[0][2] == lead["guard"]
    assert {k[1] for k, _, _ in rows} == {model.latency, 0} and any(k[0] is None for k, _, _ in rows)
    # Resolution (core 7.5): the last halving left the worst PM and the worst Ms unchanged, the ones before did not.
    levels = cset["levels"]
    assert [lv[0] for lv in levels] == list(range(cset["level"] + 2))
    last, prev = levels[-1], levels[-2]
    assert (last[2][0], last[3][0]) == (prev[2][0], prev[3][0])
    assert all((a[2][0], a[3][0]) != (b[2][0], b[3][0]) for a, b in zip(levels[:-2], levels[1:-1]))
    assert rl.set_passes(rows, pm_min, ms_max)
    (pm, pl, pa, pc), (ms, ml, ma, mc) = rl.set_worst(rows)
    print(f"set: {len(rows)} configurations, resolved at level {cset['level']}; worst PM {math.degrees(pm)!r} deg at {pl} "
          f"{pa} {pc}; worst Ms {ms!r} at {ml} {ma} {mc}")
    # (a) The planted gain: the set assertion fails.
    bad = [tuple(x * GAIN_SET_CONTROL for x in a[:3]) + (a[3],) for a in lead["axes32"]]
    planted = rl.configuration_rows(model, chain, bad, lead["dts"], rl.cpu_quota.usable_cpus(), CARD)
    assert not rl.set_passes(planted["rows"], pm_min, ms_max)
    # (b) A planted PM at one other configuration: the verdict fails, step 1''s rows alone still pass.
    j = next(n for n, r in enumerate(rows) if r[0] == (None, 0))
    g = [list(r) for r in rows[j][2]]
    g[0][0] = pm_min - PLANTED_BELOW
    planted = rows[:j] + [(rows[j][0], rows[j][1], [tuple(r) for r in g])] + rows[j + 1:]
    assert not rl.set_passes(planted, pm_min, ms_max)
    assert rl.guard_passes(rows[0][2], pm_min, ms_max)


# ---- closed-loop stability over the configuration set ------------------------------------------------------------------
#
# Model.margins finds the |L| = 1 crossings of the frequency response only (no encirclement count, no closed-loop poles). Here
# the closed rate loop of every loop of the set is proven stable on the lifted state space of attitude_lead.LeadLoop.step (one
# rate execution of the tick-rate loop: ZOH motor-lag plant, latency line, chain biquads, firmware PID law; the matrix built by
# stepping unit vectors, as LeadLoop.__init__ does), with the angle state (THETA, which feeds nothing) removed: r = 0 is the
# closed rate loop. Two models: the design model (alpha in double, T = D T_s) and the firmware model of rule step 8 (per
# execution dt_n and f32 alpha_n of rate_lead.firmware_phases at the three expf variants -+LIBM_EXP_ULPS and 0; the 2-periodic
# law's monodromy M = A_1 A_0, stable iff rho(M) < 1).
# Proof, and the tolerance: attitude_lead's certified Stein certificate (stein_candidate, stein_margin, positive_margin; rule
# step 5'). Its margin is a lower bound on the smallest eigenvalue of P - A^T P A and of T^T P T, with the rounding of every
# product bounded by Higham's gamma_n |X||Y| and every bound rounded up by SLACK; a margin > 0 proves rho(A) < 1 for the double
# matrix, so the assertion has no tolerance of its own. The radius rho_K (attitude_lead.squaring_radius) is reported and compared
# with 1 exactly: its excess over the true radius is about ln(C)/2^K, below 1e-6 for C < e^16 (RADIUS_TOL of test_attitude_lead.py),
# against a distance from 1 of 7e-4 or more here (the largest radius is printed); the proof is the certificate.

_STABILITY = {}


def lifted(loop):
    """The lifted rate-loop matrix of one execution (r = 0): LeadLoop.step on unit vectors, THETA removed."""
    n = loop.n
    cols = [loop.step([1.0 if i == k else 0.0 for i in range(n)], 0.0) for k in range(n)]
    a = [[cols[k][r] for k in range(n)] for r in range(n)]
    assert all(a[r][al.THETA] == (1.0 if r == al.THETA else 0.0) for r in range(n)), "theta feeds another state"
    keep = [i for i in range(n) if i != al.THETA]
    return [[a[r][k] for k in keep] for r in keep]


def certificate(a):
    """The Stein certificate's margin of the matrix a: > 0 proves rho(a) < 1; -inf without a candidate."""
    p = al.stein_candidate(a)
    return -math.inf if p is None else min(al.stein_margin(a, p), al.positive_margin(p))


def gain_margin(model, gains, jt, tau):
    """1/|L| at the first angle above the highest gain crossover where the unwrapped arg L of rate_lead's design loop reaches
    -pi (the phase unwrapped from low frequency as gyro_chain_design.crossovers does, the angle bisected), or None."""
    lp = model.loop(gains, jt, tau)
    th = gcd._THETA
    vals = [lp(t) for t in th]
    phase = [cmath.phase(-vals[0]) - math.pi]
    for i in range(1, len(th)):
        phase.append(phase[-1] + cmath.phase(vals[i] / vals[i - 1]))
    start = max(x[0] for x in gcd.crossovers(lp))
    for i in range(1, len(th)):
        if th[i] > start and phase[i] < -math.pi <= phase[i - 1]:
            lo, hi, base = th[i - 1], th[i], phase[i - 1]
            for _ in range(gcd.MAX_BISECTIONS):
                mid = (lo + hi) / 2
                if mid <= lo or mid >= hi:
                    break
                if base + cmath.phase(lp(mid) / vals[i - 1]) >= -math.pi:
                    lo = mid
                else:
                    hi = mid
            return 1 / abs(lp((lo + hi) / 2))
    return None


def stability_job(job):
    """The record of one loop (configuration, axis, corner) of the set: both models' certificates and radii, the gain margin
    and the certificates and radii at the gains scaled by GM / GAIN_MARGIN_CONTROL and GM x GAIN_MARGIN_CONTROL."""
    c = _STABILITY
    cfg_i, axis_i, corner_i = job
    key, label, lat, stages = c["cfgs"][cfg_i]
    m = c["model"]
    inner = al.Inner(label, c["axes32"], m.t_s, m.divisor, lat, [s for s in stages if s != gcd.IDENTITY])
    _, jt, tau = m.corners[corner_i]
    kp, ki, kd, tf = c["axes32"][axis_i]
    j = m.inertia[axis_i]

    def loop_at(f):
        return al.LeadLoop((f * kp / j, f * ki / j, f * kd / j, tf), jt, tau, inner, grid=False)

    loop = loop_at(1.0)
    a = lifted(loop)
    out = {"key": key, "axis": rl.AXES[axis_i], "corner": m.corners[corner_i][0], "design": (certificate(a), al.squaring_radius(a)[0])}
    firmware = []
    for ulps in (-rl.LIBM_EXP_ULPS, 0, rl.LIBM_EXP_ULPS):
        mats = []
        for dt, alpha in rl.firmware_phases(c["dts"], tf, ulps):
            loop.t, loop.alpha = dt, alpha
            mats.append(lifted(loop))
        mono = al._matmul(mats[1], mats[0])
        firmware.append((certificate(mono), math.sqrt(al.squaring_radius(mono)[0])))
    out["firmware"] = firmware
    gm = gain_margin(rl.Model(m.t_s, m.divisor, lat, stages, m.corners, m.a, m.inertia), (kp / j, ki / j, kd / j, tf), jt, tau)
    out["gm"] = gm
    if gm is not None:
        out["bracket"] = []
        for f in (gm / GAIN_MARGIN_CONTROL, gm * GAIN_MARGIN_CONTROL):
            scaled = lifted(loop_at(f))
            out["bracket"].append((certificate(scaled), al.squaring_radius(scaled)[0]))
    return out


@pytest.fixture(scope="module")
def stability(lead):
    m = lead["model"]
    cfgs = rl.configuration_set(lead["chain"], m.latency, lead["set"]["level"] + 1)
    assert [cfg[0] for cfg in cfgs] == [row[0] for row in lead["set"]["rows"]]
    assert len(lead["dts"]) == 2   # the monodromy below is A_1 A_0 of the 2-periodic law (rule step 8)
    _STABILITY.update(cfgs=cfgs, model=m, axes32=lead["axes32"], dts=lead["dts"])
    jobs = [(c, a, k) for c in range(len(cfgs)) for a in range(len(rl.AXES)) for k in range(len(m.corners))]
    with multiprocessing.get_context("fork").Pool(rl.cpu_quota.usable_cpus()) as pool:
        records = pool.map(stability_job, jobs, chunksize=1)
    return cfgs, records


def test_every_rate_loop_of_the_set_is_certified_stable_in_the_design_and_the_firmware_model(lead, stability):
    cfgs, records = stability
    assert len(records) == len(cfgs) * len(rl.AXES) * len(lead["model"].corners) == 120
    for r in records:
        assert r["design"][0] > 0 and r["design"][1] < 1, (r["key"], r["axis"], r["corner"], r["design"])
        for cert, rho in r["firmware"]:
            assert cert > 0 and rho < 1, (r["key"], r["axis"], r["corner"], r["firmware"])
    worst = max([r["design"][1] for r in records] + [rho for r in records for _, rho in r["firmware"]])
    print(f"{len(records)} loops certified (design and firmware models, 3 expf variants); largest radius {worst!r}")


def test_control_gains_at_the_gain_margin_over_1_1_are_stable_and_at_1_1_times_it_are_not(stability):
    """Controls (core 7.2): the certificate and the radius separate stable from unstable on the very state spaces above. Every
    loop has a -pi crossing above its crossover (so a gain margin), certified and below 1 at GM / 1.1, and neither at GM x 1.1."""
    _, records = stability
    for r in records:
        assert r["gm"] is not None and r["gm"] > 1, (r["key"], r["axis"], r["corner"])
        (cert_lo, rho_lo), (cert_hi, rho_hi) = r["bracket"]
        assert cert_lo > 0 and rho_lo < 1, (r["key"], r["axis"], r["corner"], r["bracket"])
        assert not cert_hi > 0 and rho_hi > 1, (r["key"], r["axis"], r["corner"], r["bracket"])
    print(f"min gain margin {min(r['gm'] for r in records)!r}")
