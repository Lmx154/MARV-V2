"""T4 attitude-loop chirp in Gazebo on truth gyro and truth attitude (quad spec 4 L5 pass bar, T4; QF-3: phase margin >=
PM_min; decision 0006 F "T4 attitude chirp" and E "U and the circularity", "Frozen check"; owner decision 20; decision 0005
"T4 chirp margins").

Skipped only as conftest.py says; the gz CI step fails on a skip. Every run is truth-fed, perfect-model
(tools/sim/run_l5.py): not a validation run.

Runs, per axis, scenarios/quad/L05/chirp_<axis>.yaml (tools/sim/run_l5.py run_chirp; one gz process each). The scenario holds the
torque-envelope amplitude A_env (decision 0006 F, re-derived here from the live build: run_l5.chirp_design), the band and the
window; the chirp is added to the rate setpoint at every rate execution, sticks 0, heading locked.
  yaw:          (m = 1, A_env), (m = 2, A_env), (m = 1, A_env/2), (m = 1, A_env, D/2).
  roll, pitch:  (m = 1, A_env/2^k) for k = 1..5 (owner decision 20: at A_env the vehicle tumbles, peak attitude error 2, and there
                is no crossover); then, at the amplitude of the k with the smallest PM, (m = 2) and (D/2).
  control:      (m = 1, att_kp x c) at the margin run's amplitude.

Measurement. PM(run) = run_l5.chirp_margin: indirect closed-loop identification from the log alone (run_l5 section "T4 attitude
chirp": y the attitude error variable the law multiplies by its gain, recomputed from the TRUTH quaternion; G_m = Y/D,
L = C G_m / (1 - C G_m) at the attitude period T_a, C the implemented f32 gain, crossover |L| = 1 inside the band, PM = pi +
arg L). The margin and its terms are run_l5.u_terms: yaw as at L4 (PM of the (m = 1, A_env) run, E_H = |PM(m = 1) - PM(m = 2)|,
U_A = |PM(A_env) - PM(A_env/2)|); roll and pitch (owner decision 20) PM = the minimum over the k that give a unique crossover,
U_A = the spread (max - min) of those PMs, E_H = |PM(m = 1) - PM(m = 2)| at the minimum's amplitude. U_d is run_l5.float_input_term
of the margin run (the L4 rule on the attitude period). U = E_H + U_A + U_d on every axis.

Predicate per axis. PASS iff no host step read in any window is stale (0003 item 11) and
  PM - U >= PM_min   and   min_box PM_design - U <= PM <= max_box PM_design + U,
PM_design at nominal and the four tau x J corners from tools/card/attitude.py design() (the build's gains are checked equal to
the design's), PM_min the budget's. The plateau's spread is inside U_A, so the decision's "min - U_A >= PM_min" is implied.

Duration (core 7.5). The scenario's duration_s is the converged one: |PM(D) - PM(D/2)| < E_H + U_A, both at D, at the margin run.

Negative control (the metric control of core 7.2), on every axis. att_kp x c through a SIL override, c the smallest power of two
for which the design-model PM at nominal tau and J (attitude.Loop.margin on the axis's effective gain) is below PM_min - U of the
axis's own margin. Its measured PM (C with the control's gain, the margin run's amplitude) must fail the predicate with that U.

SIM-7 fixed point (decision 0006 E, "Frozen check"). U_att = min over axes of U on these runs; the SIM-7 halving rule of
tools/card/attitude.py design() with U_att gives N1. If N1 is the live att_loop_ratio the fixed point closes. Otherwise the
chirps are re-measured at N1 WITHOUT regenerating the product (run_l5.run_chirp design_u: att_loop_ratio and att_kp flown through
sil_override from attitude.design() on U_att) and U_att(N1) gives N2; a two-cycle (N2 is the live ratio and the live ratio is
the higher rate, the smaller N) takes the higher rate and closes; N2 = N1 means the product must be regenerated (fails).
Control: a planted U moves N*, and the same rule must fail on it.

Cross-check (reported, not asserted): the measured nominal PM against the design nominal PM.
"""

import dataclasses
import math
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import attitude  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import run_l5  # noqa: E402
import run_scenario  # noqa: E402
import schema  # noqa: E402

from conftest import CARD, PLUGIN_DIR, SCEN  # noqa: E402

l4 = run_l5.l4
AXES = l5s.AXES
GYRO_VALID_BIT = 6  # marv_sil.h MARV_IMU_GYRO_VALID: the enum position of GyroValid
HALF = 0.5  # the amplitude of the U_A run, A/2
FIRST_CONTROL_C = 2
MARGIN_SCALE = {run_l5.plateau_name(k): 2.0 ** -k for k in run_l5.PLATEAU_K}


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


def deg(x):
    return None if x is None else math.degrees(x)


def verdict(pm, u_terms, design_pms, pm_min):
    """The predicate of the module docstring on one measured PM (rad); pm None (no crossover in the band) fails."""
    e_h, u_a, u_d = u_terms
    u = e_h + u_a + u_d
    lo, hi = min(design_pms) - u, max(design_pms) + u
    if pm is None:
        return {"passed": False, "reason": "no unique crossover in the band", "E_H + U_A + U_d (deg)": deg(u)}
    slack = pm - u - pm_min
    in_range = lo <= pm <= hi
    return {"passed": slack >= 0 and in_range, "PM (deg)": deg(pm), "E_H (deg)": deg(e_h), "U_A (deg)": deg(u_a),
            "U_d (deg)": deg(u_d), "PM - E_H - U_A - U_d - PM_min (deg)": deg(slack), "range (deg)": [deg(lo), deg(hi)],
            "in range": in_range}


def control_c(result, axis, limit):
    """The smallest power of two c >= 2 whose effective gain c k gives a nominal design-model PM below `limit` (rad), with
    that PM and crossover (rad/s); attitude.Loop.margin on the axis's nominal loop at N*."""
    n, t_a, k32, w32 = result["N"], result["T_a"], result["k32"], result["w32"]
    loop = next(lp for a, name, lp in attitude.build_loops(result["rate"], n) if a == axis and name == "nominal")
    c = FIRST_CONTROL_C
    while True:
        theta, pm, why = loop.margin(attitude.axis_k(axis, l4.r32(c * k32), w32))
        if theta is None:
            raise AssertionError(f"gains x {c}: the design model has no unique crossover ({why}) before PM < limit")
        if pm < limit:
            return c, pm, theta / t_a
        c *= 2


def sim7_n(u_value, result):
    """N* of the SIM-7 halving rule of attitude.design() with uncertainty u_value (rad), on the live card."""
    card, budget = schema.load_yaml(CARD), schema.load_yaml(ROOT / "design" / "budget.yaml")
    register = schema.load_yaml(ROOT / "design" / "scenario_values.yaml")
    u = {"value": u_value, "method": "measured", "source": "this run", "rule": "min over axes of E_H + U_A + U_d"}
    return attitude.design(card, budget, register, str(CARD), u, rate_result=result["rate"])["N"]


def fixed_point_closes(live, n1, n2):
    """The recorded rule (decision 0006 E step 3): N1 = SIM-7(U_att at the live N) equals the live N, or N2 =
    SIM-7(U_att at N1) is the live N and the live N is the higher rate (a two-cycle takes the higher rate)."""
    return n1 == live or (n2 == live and live < n1)


def write_u(path, value):
    path.write_text(yaml.safe_dump({"U": value, "unit": "rad", "method": "measured", "source": "this run",
                                    "rule": "min over axes of E_H + U_A + U_d of the run (test_t4_chirp.py)"}),
                    encoding="utf-8")
    return path


# ---- fixtures -------------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class Chirp:
    axis: str
    runs: dict  # name -> run_l5.ChirpRun
    scales: dict  # name -> amplitude scale of A_env
    margins: dict  # name -> identify() result
    pms: dict
    terms: dict  # run_l5.u_terms
    u_d: float
    convergence: float
    evaluation: dict
    wall_s: float

    @property
    def e_h(self):
        return self.terms["E_H"]

    @property
    def u_a(self):
        return self.terms["U_A"]

    @property
    def pm(self):
        return self.terms["pm"]

    @property
    def margin_run(self):
        return self.terms["margin_run"]

    @property
    def u(self):
        return self.e_h + self.u_a + self.u_d


_CACHE = {}


def measure(tmp_path_factory, axis, design_u=None):
    """The runs of one axis and their margins (cached: the fixed-point test needs every axis). `design_u` runs at the N of
    that SIM-7 uncertainty file without regenerating the product (run_l5.run_chirp)."""
    key = (axis, str(design_u))
    if key in _CACHE:
        return _CACHE[key]
    out = tmp_path_factory.mktemp(f"chirp_{axis}")
    scenario = SCEN / f"chirp_{axis}.yaml"
    runs, scales, margins = {}, {}, {}

    def go(name, m, scale, halved=False):
        runs[name] = run_l5.run_chirp(CARD, scenario, m, out, PLUGIN_DIR, amp_scale=scale, halved=halved, design_u=design_u)
        scales[name] = scale
        margins[name] = run_l5.chirp_margin(runs[name])

    if axis in run_l5.ROLL_PITCH:
        for k in run_l5.PLATEAU_K:
            go(run_l5.plateau_name(k), 1, MARGIN_SCALE[run_l5.plateau_name(k)])
        valid = {n: r["pm"] for n, r in margins.items() if r["pm"] is not None}
        low = min(valid, key=valid.get)
        go("m2", 2, scales[low])
        go("half_duration", 1, scales[low], True)
    else:
        go("m1", 1, 1.0)
        go("m2", 2, 1.0)
        go("half_amplitude", 1, HALF)
        go("half_duration", 1, 1.0, True)
    pms = {name: r["pm"] for name, r in margins.items()}
    terms = run_l5.u_terms(axis, pms)
    assert pms["m2"] is not None and pms["half_duration"] is not None, {n: r["reason"] for n, r in margins.items()}
    m_run = terms["margin_run"]
    design = runs[m_run].design
    float_term = run_l5.float_input_term(runs[m_run], margins[m_run])
    conv = abs(terms["pm"] - pms["half_duration"])
    ev = verdict(terms["pm"], (terms["E_H"], terms["U_A"], float_term["U_d"]), list(design["design_pm"][axis].values()),
                 design["pm_min"])
    ev.update({
        "margin run": m_run, "amplitude scale of A_env": scales[m_run], "U_d terms": float_term,
        "PM per run (deg)": {n: deg(v) for n, v in pms.items()},
        "crossover per run (rad/s)": {n: r["crossover"] for n, r in margins.items()},
        "no crossover": {n: r["reason"] for n, r in margins.items() if r["pm"] is None},
        "|PM(D) - PM(D/2)| (deg)": deg(conv), "E_H + U_A (deg)": deg(terms["E_H"] + terms["U_A"]),
        "design PM (deg)": {n: deg(v) for n, v in design["design_pm"][axis].items()},
        "measured - design nominal (deg)": deg(terms["pm"] - design["design_pm"][axis]["nominal"]),
        "peak |y| per run (rad)": {n: max(abs(v) for v in s.y) for n, s in runs.items()},
        "stale reads in the windows": {n: len(s.window_stale) for n, s in runs.items()},
        "window executions with a motor at a DShot end": {n: s.dshot_end_executions for n, s in runs.items()},
        "att_loop_ratio flown": design["N"],
    })
    wall = sum(s.step.run.wall_s for s in runs.values())
    run_l5.write_report([s.step for s in runs.values()], dict(ev, gz_wall_s=wall),
                        str(out / f"chirp_{axis}_{run_l5.NAME_LABEL}_sequence.report.txt"))
    _CACHE[key] = Chirp(axis, runs, scales, margins, pms, terms, float_term["U_d"], conv, ev, wall)
    return _CACHE[key]


@pytest.fixture(scope="module", params=AXES)
def axis(request):
    return request.param


@pytest.fixture(scope="module")
def chirp(tmp_path_factory, axis):
    return measure(tmp_path_factory, axis)


@pytest.fixture(scope="module")
def control(tmp_path_factory, axis, chirp):
    s1 = chirp.runs[chirp.margin_run]
    result = s1.design["result"]
    c, pm_design, wc_design = control_c(result, axis, s1.design["pm_min"] - chirp.u)
    gain = l4.r32(c * result["k32"])
    out = tmp_path_factory.mktemp(f"chirp_{axis}_control")
    s = run_l5.run_chirp(CARD, SCEN / f"chirp_{axis}.yaml", 1, out, PLUGIN_DIR, amp_scale=chirp.scales[chirp.margin_run],
                         overrides={"att_kp": (l4.F32, repr(gain))})
    r = run_l5.chirp_margin(s)
    assert r["pm"] is not None, r["reason"]
    float_term = run_l5.float_input_term(s, r)
    design = s1.design
    ev = verdict(r["pm"], (chirp.e_h, chirp.u_a, float_term["U_d"]), list(design["design_pm"][axis].values()),
                 design["pm_min"])
    ev.update({"U_d terms": float_term, "c": c, "control att_kp": gain, "design PM at nominal (deg)": deg(pm_design),
               "design crossover (rad/s)": wc_design, "measured crossover (rad/s)": r["crossover"],
               "stale reads in the window": len(s.window_stale)})
    run_l5.write_report([s.step], dict(ev, gz_wall_s=s.step.run.wall_s))
    return {"run": s, "margin": r, "evaluation": ev, "c": c, "gain": gain}


@pytest.fixture(scope="module")
def u_att(tmp_path_factory):
    """U_att = min over axes of U on these runs at the live N, with the axis it is at."""
    per_axis = {a: measure(tmp_path_factory, a).u for a in AXES}
    a = min(per_axis, key=per_axis.get)
    return per_axis[a], a, per_axis


@pytest.fixture(scope="module")
def design_result(tmp_path_factory):
    """attitude.design() on the committed SIM-7 file at the live build (the seed table and the rate result)."""
    return run_l5.attitude_design(str(CARD), str(ROOT))


# ---- the pass bar ---------------------------------------------------------------------------------------------------

def test_chirp_margin_meets_qf3(chirp, axis, capsys):
    ev = chirp.evaluation
    _say(capsys, f"{axis:<5} chirp {'PASS' if ev['passed'] else 'FAIL'}  PM {ev['PM (deg)']:.6f} deg (run {ev['margin run']}), "
                 f"E_H {ev['E_H (deg)']:.3e}, U_A {ev['U_A (deg)']:.6f}, U_d {ev['U_d (deg)']:.6f}, slack "
                 f"{ev['PM - E_H - U_A - U_d - PM_min (deg)']:.6f} deg, range [{ev['range (deg)'][0]:.6f}, "
                 f"{ev['range (deg)'][1]:.6f}], measured - design nominal "
                 f"{ev['measured - design nominal (deg)']:+.6f} deg, gz wall {chirp.wall_s:.2f} s",
         f"    runs (deg): {ev['PM per run (deg)']}", f"    no crossover: {ev['no crossover']}",
         f"    peak |y| (rad): {ev['peak |y| per run (rad)']}")
    for s in chirp.runs.values():
        assert run_scenario.complete_trailer(s.step.run.log, s.step.run.iterations, s.step.run.m)
        assert s.window_stale == [], f"m = {s.step.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
        assert "truth_fed_perfect_model" in Path(s.step.run.log_path).name
    assert ev["passed"], ev


def test_duration_is_converged(chirp, axis):
    c = l5s.values(l5s.load(SCEN / f"chirp_{axis}.yaml"))["script"]["chirp"]
    assert chirp.runs[chirp.margin_run].dur_us == round(c["duration_s"] * 1e6)
    assert chirp.runs["half_duration"].dur_us * 2 == chirp.runs[chirp.margin_run].dur_us
    assert chirp.convergence < chirp.e_h + chirp.u_a, chirp.evaluation


def test_chirp_runs_on_the_live_design_and_truth_sources(chirp, axis, live_params):
    design = chirp.runs[chirp.margin_run].design
    c = l5s.values(l5s.load(SCEN / f"chirp_{axis}.yaml"))["script"]["chirp"]
    a_env = l4.r32(design["axes"][axis]["amplitude_rad_s"])
    assert l4.r32(c["amp_rad_s"]) == a_env, "the scenario amplitude is stale"
    assert (l4.r32(c["w_lo_rad_s"]), l4.r32(c["w_hi_rad_s"])) == tuple(l4.r32(w) for w in design["band"])
    assert design["N"] == live_params["att_loop_ratio"]
    for name, s in chirp.runs.items():
        assert s.step.harness_overrides == {}
        assert s.k == attitude.axis_k(axis, live_params["att_kp"], live_params["att_yaw_weight"])
        assert s.amp == l4.r32(a_env * chirp.scales[name]), name
        text = Path(s.step.run.world_path).read_text(encoding="utf-8")
        assert text.count("<gyro_source>truth</gyro_source>") == 1
        assert text.count("<attitude_source>truth</attitude_source>") == 1
        assert s.stamps[0] == s.step.plan.stamp_us(s.step.plan.origin)
        assert any(s.d) and s.d[-1] == 0.0, "the window must cover the chirp and its end"
        assert s.step.overrides["l5_chirp_amp_rad_s"] == (l4.F32, repr(s.amp)), name


# ---- negative control -----------------------------------------------------------------------------------------------

def test_control_gain_times_c_fails_the_predicate(control, chirp, axis, capsys):
    ev = control["evaluation"]
    _say(capsys, f"{axis:<5} control att_kp x {control['c']}: measured PM {ev.get('PM (deg)')} deg (design "
                 f"{ev['design PM at nominal (deg)']:.6f}), crossover {ev['measured crossover (rad/s)']} rad/s, "
                 f"{'PASS' if ev['passed'] else 'FAIL'} (must fail), gz wall {control['run'].step.run.wall_s:.2f} s")
    s = control["run"]
    assert s.step.harness_overrides == {"att_kp": (l4.F32, repr(control["gain"]))}
    assert s.k == attitude.axis_k(axis, control["gain"], s.design["result"]["w32"])
    assert run_scenario.complete_trailer(s.step.run.log, s.step.run.iterations, s.step.run.m)
    assert s.window_stale == [], "the control must fail on the margin, not on a stale read"
    assert not ev["passed"], ev


# ---- the SIM-7 fixed point ------------------------------------------------------------------------------------------

def test_sim7_fixed_point(u_att, design_result, live_params, tmp_path_factory, capsys):
    u1, at, per_axis = u_att
    result = design_result
    live = live_params["att_loop_ratio"]
    n1 = sim7_n(u1, result)
    lines = [f"SIM-7 fixed point: U_att(N = {live}) {u1:.9e} rad ({deg(u1):.6f} deg) at {at}, per axis {per_axis}; "
             f"N1 = SIM-7(U_att) = {n1}, live att_loop_ratio = {live}; seed U^0 = {result['U']['value']:.9e} rad",
             f"    halving table at the seed: {result['table']}"]
    n2 = None
    if n1 != live:
        u_file = write_u(tmp_path_factory.mktemp("sim7") / "u_n1.yaml", u1)
        at_n1 = {a: measure(tmp_path_factory, a, design_u=u_file).u for a in AXES}
        u2 = min(at_n1.values())
        n2 = sim7_n(u2, result)
        lines.append(f"    re-measured at N = {n1} (overrides, product not regenerated): U_att {u2:.9e} rad per axis {at_n1}; "
                     f"N2 = SIM-7(U_att) = {n2}; two-cycle: {n2 == live and live < n1}")
    _say(capsys, *lines)
    assert fixed_point_closes(live, n1, n2), (u1, n1, n2, live)


def test_sim7_fixed_point_control_planted_u_moves_n_star(design_result, live_params):
    result = design_result
    live = live_params["att_loop_ratio"]
    first = next(row for row in result["table"] if row["delta"] is not None)
    planted = first["delta"] * HALF
    n1 = sim7_n(planted, result)
    assert n1 != live, "the planted U does not move N*"
    assert not fixed_point_closes(live, n1, None)
    assert not fixed_point_closes(live, n1, n1), "N2 = N1 must not close: the product would have to be regenerated"
    assert fixed_point_closes(live, live, None) and fixed_point_closes(live, 2 * live, live)
    assert not fixed_point_closes(2 * live, live, 2 * live), "a two-cycle whose live rate is the lower one must not close"
