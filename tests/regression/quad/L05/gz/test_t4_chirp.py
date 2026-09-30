"""T4 attitude-loop chirp in Gazebo on truth gyro and truth attitude (quad spec 4 L5 pass bar, T4; QF-3: phase margin >=
PM_min; decision 0006 F "T4 attitude chirp" and E "U and the circularity", "Frozen check"; decision 0005 "T4 chirp margins").

Skipped only as conftest.py says; the gz CI step fails on a skip. Every run is truth-fed, perfect-model
(tools/sim/run_l5.py): not a validation run.

Runs, per axis, scenarios/quad/L05/chirp_<axis>.yaml (tools/sim/run_l5.py run_chirp; one gz process each):
  (m = 1, A, D), (m = 2, A, D), (m = 1, A/2, D), (m = 1, A, D/2), and the negative control (m = 1, A, D, att_kp x c).
The band, the amplitude A and the stamps are the scenario's (derived values, re-derived here from the live build:
run_l5.chirp_design); the chirp is added to the rate setpoint at every rate execution, sticks 0, heading locked.

Measurement. PM(run) = run_l5.chirp_margin: indirect closed-loop identification from the log alone (run_l5 section "T4
attitude chirp": y the attitude error variable the law multiplies by its gain, recomputed from the TRUTH quaternion; G_m = Y/D,
L = C G_m / (1 - C G_m) at the attitude period T_a, C the implemented f32 gain, crossover |L| = 1 inside the band, PM = pi +
arg L). E_H = |PM(m = 1) - PM(m = 2)| (decision 0003 E), U_A = |PM(A) - PM(A/2)|, U_d = run_l5.float_input_term of the
(m = 1, A) run (the L4 rule on the attitude period).

Predicate per axis. PASS iff no host step read in any window is stale (0003 item 11) and, with U = E_H + U_A + U_d,
  PM(m = 1, A) - U >= PM_min   and   min_box PM_design - U <= PM(m = 1, A) <= max_box PM_design + U,
PM_design at nominal and the four tau x J corners from tools/card/attitude.py design() (the build's gains are checked equal
to the design's), PM_min the budget's.

Duration (core 7.5). The scenario's duration_s is the converged one: |PM(D) - PM(D/2)| < E_H + U_A, both at D.

Negative control (the metric control of core 7.2). att_kp x c through a SIL override, c the smallest power of two for which the
design-model PM at nominal tau and J (attitude.Loop.margin on the axis's effective gain) is below PM_min - U of the axis's own
nominal runs. Its measured PM (C with the control's gain) must fail the predicate evaluated with that U.

SIM-7 fixed point (decision 0006 E, "Frozen check"). U_att = min over axes of (E_H + U_A + U_d) on these runs; the SIM-7 halving
rule of tools/card/attitude.py design() with U_att must give the live att_loop_ratio. Control: a planted U (half of the first
halving difference of the live table) moves N*, and the same assertion must fail on it.

Cross-check (reported, not asserted): the measured nominal PM against the design nominal PM.
"""

import dataclasses
import math
import sys
from pathlib import Path

import pytest

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
RUNS = (("m1", 1, 1.0, False), ("m2", 2, 1.0, False), ("half_amplitude", 1, HALF, False),
        ("half_duration", 1, 1.0, True))


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
        theta, pm, why = loop.margin(attitude.axis_k(axis, run_l5.l4.r32(c * k32), w32))
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


def fixed_point_holds(u_value, live_ratio, result):
    return sim7_n(u_value, result) == live_ratio


# ---- fixtures -------------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class Chirp:
    axis: str
    runs: dict  # name -> run_l5.ChirpRun
    margins: dict  # name -> identify() result
    e_h: float
    u_a: float
    u_d: float
    convergence: float
    evaluation: dict
    wall_s: float

    @property
    def u(self):
        return self.e_h + self.u_a + self.u_d


_CACHE = {}


def measure(tmp_path_factory, axis):
    """The four runs of one axis and their margins (cached: the fixed-point test needs every axis)."""
    if axis in _CACHE:
        return _CACHE[axis]
    out = tmp_path_factory.mktemp(f"chirp_{axis}")
    scenario = SCEN / f"chirp_{axis}.yaml"
    runs = {name: run_l5.run_chirp(CARD, scenario, m, out, PLUGIN_DIR, amp_scale=scale, halved=halved)
            for name, m, scale, halved in RUNS}
    margins = {name: run_l5.chirp_margin(s) for name, s in runs.items()}
    pms = {name: r["pm"] for name, r in margins.items()}
    assert all(v is not None for v in pms.values()), {n: r["reason"] for n, r in margins.items()}
    e_h, u_a = abs(pms["m1"] - pms["m2"]), abs(pms["m1"] - pms["half_amplitude"])
    conv = abs(pms["m1"] - pms["half_duration"])
    design = runs["m1"].design
    float_term = run_l5.float_input_term(runs["m1"], margins["m1"])
    ev = verdict(pms["m1"], (e_h, u_a, float_term["U_d"]), list(design["design_pm"][axis].values()), design["pm_min"])
    ev.update({
        "U_d terms": float_term, "PM per run (deg)": {n: deg(v) for n, v in pms.items()},
        "crossover per run (rad/s)": {n: r["crossover"] for n, r in margins.items()},
        "|PM(D) - PM(D/2)| (deg)": deg(conv), "E_H + U_A (deg)": deg(e_h + u_a),
        "design PM (deg)": {n: deg(v) for n, v in design["design_pm"][axis].items()},
        "measured - design nominal (deg)": deg(pms["m1"] - design["design_pm"][axis]["nominal"]),
        "peak |y| per run (rad)": {n: max(abs(v) for v in s.y) for n, s in runs.items()},
        "stale reads in the windows": {n: len(s.window_stale) for n, s in runs.items()},
        "window executions with a motor at a DShot end": {n: s.dshot_end_executions for n, s in runs.items()},
    })
    wall = sum(s.step.run.wall_s for s in runs.values())
    run_l5.write_report([s.step for s in runs.values()], dict(ev, gz_wall_s=wall),
                        str(out / f"chirp_{axis}_{run_l5.NAME_LABEL}_sequence.report.txt"))
    _CACHE[axis] = Chirp(axis, runs, margins, e_h, u_a, float_term["U_d"], conv, ev, wall)
    return _CACHE[axis]


@pytest.fixture(scope="module", params=AXES)
def axis(request):
    return request.param


@pytest.fixture(scope="module")
def chirp(tmp_path_factory, axis):
    return measure(tmp_path_factory, axis)


@pytest.fixture(scope="module")
def control(tmp_path_factory, axis, chirp):
    s1 = chirp.runs["m1"]
    result = s1.design["result"]
    c, pm_design, wc_design = control_c(result, axis, s1.design["pm_min"] - chirp.u)
    gain = run_l5.l4.r32(c * result["k32"])
    over = {"att_kp": (l4.F32, repr(gain))}
    out = tmp_path_factory.mktemp(f"chirp_{axis}_control")
    s = run_l5.run_chirp(CARD, SCEN / f"chirp_{axis}.yaml", 1, out, PLUGIN_DIR, overrides=over)
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
    """U_att = min over axes of E_H + U_A + U_d on these runs, with the axis it is at."""
    per_axis = {a: measure(tmp_path_factory, a).u for a in AXES}
    a = min(per_axis, key=per_axis.get)
    return per_axis[a], a, per_axis


# ---- the pass bar ---------------------------------------------------------------------------------------------------

def test_chirp_margin_meets_qf3(chirp, axis, capsys):
    ev = chirp.evaluation
    _say(capsys, f"{axis:<5} chirp {'PASS' if ev['passed'] else 'FAIL'}  PM {ev['PM (deg)']:.6f} deg, "
                 f"E_H {ev['E_H (deg)']:.3e}, U_A {ev['U_A (deg)']:.6f}, U_d {ev['U_d (deg)']:.6f}, slack "
                 f"{ev['PM - E_H - U_A - U_d - PM_min (deg)']:.6f} deg, range [{ev['range (deg)'][0]:.6f}, "
                 f"{ev['range (deg)'][1]:.6f}], measured - design nominal "
                 f"{ev['measured - design nominal (deg)']:+.6f} deg, peak |y| {ev['peak |y| per run (rad)']['m1']:.4f} rad, "
                 f"gz wall {chirp.wall_s:.2f} s", f"    runs (deg): {ev['PM per run (deg)']}")
    for s in chirp.runs.values():
        assert run_scenario.complete_trailer(s.step.run.log, s.step.run.iterations, s.step.run.m)
        assert s.window_stale == [], f"m = {s.step.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
        assert "truth_fed_perfect_model" in Path(s.step.run.log_path).name
    assert ev["passed"], ev


def test_duration_is_converged(chirp, axis):
    c = l5s.values(l5s.load(SCEN / f"chirp_{axis}.yaml"))["script"]["chirp"]
    assert chirp.runs["m1"].dur_us == round(c["duration_s"] * 1e6)
    assert chirp.runs["half_duration"].dur_us * 2 == chirp.runs["m1"].dur_us
    assert chirp.convergence < chirp.e_h + chirp.u_a, chirp.evaluation


def test_chirp_runs_on_the_live_design_and_truth_sources(chirp, axis, live_params):
    s1 = chirp.runs["m1"]
    design = s1.design
    c = l5s.values(l5s.load(SCEN / f"chirp_{axis}.yaml"))["script"]["chirp"]
    assert run_l5.l4.r32(c["amp_rad_s"]) == run_l5.l4.r32(design["axes"][axis]["amplitude_rad_s"]), "the scenario amplitude is stale"
    assert (run_l5.l4.r32(c["w_lo_rad_s"]), run_l5.l4.r32(c["w_hi_rad_s"])) == tuple(run_l5.l4.r32(w) for w in design["band"])
    assert design["N"] == live_params["att_loop_ratio"]
    for name, s in chirp.runs.items():
        assert s.step.harness_overrides == {}
        assert s.k == attitude.axis_k(axis, live_params["att_kp"], live_params["att_yaw_weight"])
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

def test_sim7_fixed_point(u_att, chirp, live_params, capsys):
    u, at, per_axis = u_att
    result = chirp.runs["m1"].design["result"]
    n = sim7_n(u, result)
    _say(capsys, f"SIM-7 fixed point: U_att {u:.9e} rad ({deg(u):.6f} deg) at {at}, per axis {per_axis}; "
                 f"N*(U_att) = {n}, live att_loop_ratio = {live_params['att_loop_ratio']}; U^0 (seed file) = "
                 f"{result['U']['value']:.9e} rad", f"    halving table: {result['table']}")
    assert fixed_point_holds(u, live_params["att_loop_ratio"], result), (u, n, live_params["att_loop_ratio"])


def test_sim7_fixed_point_control_planted_u_moves_n_star(chirp, live_params):
    result = chirp.runs["m1"].design["result"]
    first = next(row for row in result["table"] if row["delta"] is not None)
    planted = first["delta"] * HALF
    assert sim7_n(planted, result) != live_params["att_loop_ratio"], "the planted U does not move N*"
    assert not fixed_point_holds(planted, live_params["att_loop_ratio"], result)
