"""The L5 T4 chirp's plateau admission (tools/sim/run_l5.py section "the plateau admission"; owner decision of 2026-10-02,
decision 0014 third round item 2, amending decision 0006 decision 20's plateau). No Gazebo: on the committed L5 T3 fixtures
(tests/regression/quad/L05/t3/reference: attitude_t3_inputs.txt, the product parameter values, and attitude_t3_q_inputs.txt,
the mixer side) and the committed chirp_roll / chirp_pitch scenarios. It checks
  - the shift: run_l5.chirp_admission's PM_nl(k) and PM_lin, on roll and pitch, equal a fresh run of the stage (c) chirp
    diagnosis's method (a1_model.py, 2026-10-02: reference_run below, its run() with the axis as a parameter) on the same
    inputs; the derived tolerance is 0, because the port performs the reference's binary64 operations in the same order on
    the same values in one process. Control: the reference one float32 step of the amplitude away gives a different PM_nl.
    The quantiser's inputs run_l5 derives from the build's table, the plan and the card equal the committed q fixture.
  - the threshold: it is |PM(att_kp x 1.1) - PM(att_kp)| of the fine control, f32(att_kp f32(1.1)) as t3_test.cpp scales it,
    on the nominal loop of the chirp's design reference (run_l5.t4_lead_reference: its PM table and its loops), exactly.
    Control: the same change at the J+,tau+ corner is a different number (the rule's "at the same corner").
  - the rule: a planted run with |shift| >= threshold is excluded and one just below is admitted; an excluded run's PM does
    not enter the plateau (u_terms with the admitted runs); fewer than run_l5.PLATEAU_MIN_RUNS admitted runs raise.
rate_lead.design and the stage (c) attitude design over the configuration set run once per pytest session (conftest.py);
the run_l5 call that derives the latter is checked to receive the inputs the session's design was computed from.
"""

import math
import sys
from pathlib import Path
from unittest import mock

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import attitude  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import run_l5  # noqa: E402
import schema  # noqa: E402

rm = run_l5._recovery_model()
oracle = rm.oracle
l4 = run_l5.l4

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
REF = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference"
SCEN = ROOT / "scenarios" / "quad" / "L05"
GAIN_CONTROL = 1.1  # the fine control "gains x 1.1" (core 7.2; quad spec 4 L5 T3; t3_test.cpp kGainScale)
CONTROL_CORNER = "J+,tau+"


def fixture_params():
    """The product parameter table of the L5 T3 fixtures: attitude_t3_inputs.txt and the firmware side of
    attitude_t3_q_inputs.txt."""
    p = oracle.read_inputs(REF / "attitude_t3_inputs.txt")
    q = oracle.read_q_inputs(REF / "attitude_t3_q_inputs.txt")
    p.update({k: q[k] for k in oracle.Q_FW_KEYS if k != "l5_thrust_n"})
    return p, q


def reference_run(su, qf, axis, amp, w_lo, w_hi, dur_us, n_exec, linear):
    """a1_model.py run() of the stage (c) chirp diagnosis (2026-10-02), at the nominal corner (jt = taut = 1), the axis index
    in place of its roll index 0: the design model's PM (rad) or None."""
    idx = ("roll", "pitch", "yaw").index(axis)
    p, cfg = su.p, su.cfg
    n_ticks = su.divisor * su.ratio
    plants = [oracle.Axis(p[oracle.INERTIA[n]] * 1.0, p['motor_tau'] * 1.0, su.tick_s) for n in oracle.AXES]
    kp = [p[f'rate_kp_{a}'] for a in oracle.AXES]
    ki = [p[f'rate_ki_{a}'] for a in oracle.AXES]
    kd = [p[f'rate_kd_{a}'] for a in oracle.AXES]
    b0, b1, b2, a1, a2 = su.lpf
    qz = rm.Quant3(qf) if not linear else None
    chain, y_tick = [None] * 3, [0.0] * 3
    integral, e_prev, u = [0.0] * 3, [0.0] * 3, [0.0] * 3
    y_prev, d_f = [0.0] * 3, [0.0] * 3
    q = (1.0, 0.0, 0.0, 0.0)
    q_sp = q
    ys, ds = [], []
    for j in range(n_exec * n_ticks):
        for i, pl in enumerate(plants):
            x = pl.w
            x1, x2, yy1, yy2 = chain[i] if j > 0 else (x, x, x, x)
            yv = b0 * x + b1 * x1 + b2 * x2 - a1 * yy1 - a2 * yy2
            chain[i] = (x, x1, yv, yy1)
            y_tick[i] = yv
        if j % n_ticks == 0:
            a = j // n_ticks
            t_us = su.stamp_us(j)
            dd = l4.chirp_value(t_us, amp, w_lo, w_hi, 0, dur_us)
            yerr = run_l5.attitude_error(q, cfg.w)[idx]
            if linear:
                yerr = plants[idx].th
                r_hold = [0.0, 0.0, 0.0]
                r_hold[idx] = -cfg.kp * yerr
            else:
                r_hold = rm.law(cfg, q, q_sp)
            ys.append(yerr)
            ds.append(dd)
            r_hold[idx] += dd
            if a == 0:
                u = [0.0] * 3
                y_prev = list(y_tick)
            else:
                dt = su.dt_exec(a)
                yv = list(y_tick)
                req = [0.0] * 3
                for i in range(3):
                    alpha = su.alpha(dt, i)[0]
                    integral[i] += ki[i] * e_prev[i] * dt
                    e = r_hold[i] - yv[i]
                    d_raw = -kd[i] * (yv[i] - y_prev[i]) / dt
                    d_f[i] = d_f[i] + alpha * (d_raw - d_f[i]) if alpha != 1.0 else d_raw
                    req[i] = kp[i] * e + integral[i] + d_f[i]
                    e_prev[i] = e
                    y_prev[i] = yv[i]
                u = qz(req) if qz is not None else req
            if a == 0 and qz is not None:
                u = qz([0.0, 0.0, 0.0])
        d3 = [pl.step(u[i]) for i, pl in enumerate(plants)]
        q = oracle.qmul(q, oracle.quat_exp(d3))
        nq = math.sqrt(sum(c * c for c in q))
        q = tuple(c / nq for c in q)
    return l4.identify(ys, ds, su.t_a, cfg.kp, 0.0, (w_lo, w_hi))["pm"]


def session_attitude_design(shared):
    """attitude_lead.design as run_l5.attitude_lead_design calls it, answering with the session's design (conftest.py) after
    checking the call's inputs are the ones that design was computed from: the card, budget and register documents, the inner
    loop's label, gains, tick, divisor, latency and stages, and the rate design."""
    def design(card, budget, scenario, card_path, inner, rate_result=None, procs=None):
        want = shared["inner"]
        assert card_path == str(CARD)
        assert (card, budget, scenario) == tuple(schema.load_yaml(p) for p in (CARD, BUDGET, SCENARIO))
        assert (inner.label, inner.axes32, inner.t_s, inner.divisor, inner.latency, inner.stages) == (
            want.label, want.axes32, want.t_s, want.divisor, want.latency, want.stages)
        assert [inner.configs(lv) for lv in range(2)] == [want.configs(lv) for lv in range(2)]
        assert rate_result == shared["rate"]
        return shared
    return design


@pytest.fixture(scope="module")
def design(rate_lead_design, attitude_lead_design):
    """chirp_design_lead's "result" quantities and the T4 reference, on the session's rate_lead.design and attitude design."""
    with mock.patch.object(l4, "_lead_design", return_value=rate_lead_design), \
            mock.patch.object(run_l5.attitude_lead, "design", session_attitude_design(attitude_lead_design)):
        _, res = run_l5.attitude_lead_design(str(CARD), str(ROOT))
        ref = run_l5.t4_lead_reference(str(CARD), str(ROOT))
    return {"rate": res["rate"], "inner": ref["inner"], "k32": res["k32"], "w32": res["w32"]}, ref


@pytest.fixture(scope="module")
def admission(design):
    params, _ = fixture_params()
    return {axis: run_l5.chirp_admission(CARD, SCEN / f"chirp_{axis}.yaml", params=params, result=design[0])
            for axis in run_l5.ROLL_PITCH}


def chirp_inputs(axis):
    params, qf = fixture_params()
    doc = l5s.load(SCEN / f"chirp_{axis}.yaml")
    p = run_l5.plan(doc, params, CARD)
    c = p.chirp
    return (oracle.Setup(params), qf, l4.r32(c["amp_rad_s"]), l4.r32(c["w_lo_rad_s"]), l4.r32(c["w_hi_rad_s"]), c["dur_us"],
            p.end + 1, p.thrust_n)


@pytest.mark.parametrize("axis", run_l5.ROLL_PITCH)
def test_shift_is_the_diagnosis_method_on_the_same_inputs(admission, axis):
    su, qf, a_env, w_lo, w_hi, dur_us, n_exec, thrust = chirp_inputs(axis)
    params, _ = fixture_params()
    assert run_l5.quant_inputs(CARD, params, thrust) == qf, "the derived quantiser inputs are not the committed q fixture"
    ad = admission[axis]
    first = ad["runs"][run_l5.plateau_name(run_l5.PLATEAU_K[0])]["amp"]
    lin = reference_run(su, qf, axis, first, w_lo, w_hi, dur_us, n_exec, True)
    assert ad["pm_linear"] == lin
    for k in run_l5.PLATEAU_K:
        r = ad["runs"][run_l5.plateau_name(k)]
        amp = l4.r32(a_env * 2.0 ** -k)
        assert r["amp"] == amp
        pm = reference_run(su, qf, axis, amp, w_lo, w_hi, dur_us, n_exec, False)
        assert r["pm"] == pm, (axis, k)
        assert r["shift"] == (None if pm is None else pm - lin), (axis, k)


def test_control_one_f32_step_of_the_amplitude_moves_the_nonlinear_pm(admission):
    """The exact comparison above resolves a one-ulp input change: the reference at the next float32 amplitude above a run's
    gives a different PM_nl."""
    su, qf, a_env, w_lo, w_hi, dur_us, n_exec, _ = chirp_inputs("roll")
    k = run_l5.PLATEAU_K[-1]
    amp = l4.r32(a_env * 2.0 ** -k)
    pm = reference_run(su, qf, "roll", amp + oracle.ulp32(amp), w_lo, w_hi, dur_us, n_exec, False)
    assert pm != admission["roll"]["runs"][run_l5.plateau_name(k)]["pm"]


@pytest.mark.parametrize("axis", run_l5.ROLL_PITCH)
def test_threshold_is_the_fine_controls_pm_change_at_the_nominal_corner(design, admission, axis):
    result, ref = design
    fine_k = l4.r32(result["k32"] * l4.r32(GAIN_CONTROL))
    fine = {(a, name): lp.margin(attitude.axis_k(a, fine_k, result["w32"]))[1] for a, name, lp in ref["loops"]
            if a == axis and name in ("nominal", CONTROL_CORNER)}
    pm = ref["design_pm"][axis]["nominal"]
    ad = admission[axis]
    assert ad["threshold"] == abs(fine[axis, "nominal"] - pm) > 0
    assert (ad["pm_design"], ad["pm_fine"]) == (pm, fine[axis, "nominal"])
    corner = abs(fine[axis, CONTROL_CORNER] - ref["design_pm"][axis][CONTROL_CORNER])
    assert corner != ad["threshold"], "control: another corner's change must differ"


@pytest.mark.parametrize("axis", run_l5.ROLL_PITCH)
def test_admission_rule_and_planted_runs(admission, axis):
    ad = admission[axis]
    thr = ad["threshold"]
    shifts = {n: r["shift"] for n, r in ad["runs"].items()}
    assert ad["admitted"] == run_l5.plateau_admission(shifts, thr)
    for n, s in shifts.items():
        assert (n in ad["admitted"]) == (s is not None and abs(s) < thr), n
    names = [run_l5.plateau_name(k) for k in run_l5.PLATEAU_K]
    planted = n = names[len(names) // 2]
    for s in (thr, -thr, 2 * thr, None):
        assert n not in run_l5.plateau_admission(dict(shifts, **{n: s}), thr), f"control: shift {s!r} must be excluded"
    assert n in run_l5.plateau_admission(dict(shifts, **{n: math.nextafter(thr, 0.0)}), thr)
    # An excluded run's PM, planted as the lowest, does not enter the plateau.
    admitted = [x for x in names if x != planted][:run_l5.PLATEAU_MIN_RUNS]
    pm = {x: ad["runs"][x]["pm"] or ad["pm_linear"] for x in names}
    pm[planted] = min(pm.values()) / 2
    pm["m2"] = pm[admitted[0]]
    t = run_l5.u_terms(axis, pm, admitted)
    assert t["margin_run"] != planted and t["pm"] == min(pm[x] for x in admitted)
    assert run_l5.u_terms(axis, pm)["margin_run"] == planted, "control: without the admission the planted run is the margin"
    with pytest.raises(run_l5.PlanError):
        run_l5.u_terms(axis, pm, admitted[:run_l5.PLATEAU_MIN_RUNS - 1])
