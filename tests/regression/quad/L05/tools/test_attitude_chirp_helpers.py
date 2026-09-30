"""Pure-Python tests of the L5 attitude-chirp identification helpers of tools/sim/run_l5.py (decision 0006 F "T4 attitude chirp",
E "U and the circularity"; no Gazebo): attitude_error, steady_torque_peak, the identification on a synthetic closed loop at
tick resolution, the scenario files and the SIM-7 halving rule with a planted U. Each has a control that plants a violation.

Synthetic loop. The design model of tools/card/attitude.py (rate_model: the exact ZOH step of the closed rate loop and the
angle) stepped one rate execution at a time, the attitude gain k applied to the angle at every N-th execution and held, the
chirp added at every rate execution: the structure of the l5_attitude_scripted composition on the design plant.
"""

import cmath
import functools
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

l4 = run_l5.l4
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05"
AXES = l5s.AXES
REL_TOL = 1e-5  # scenario test value: the residual of the synthetic loop after the settling time below (e^-14 of its slowest pole)
SETTLE_S = 12.0  # scenario test value: 15 time constants of the slowest design pole (0.82 s, tools/card/attitude.py Jury radius)
HALF = 0.5
TWO = 2


@functools.lru_cache(maxsize=None)
def design():
    return run_l5.attitude_design(str(CARD), str(ROOT))


def model(axis="roll", corner="nominal"):
    res = design()
    rm = res["rate"]
    loops = dict((name, (jt, tau)) for name, jt, tau in l4.rate.corner_list(rm["inputs"]["tau"], rm["inputs"]["b_tau"],
                                                                             rm["inputs"]["b_J"]))
    jt, tau = loops[corner]
    kp, ki = attitude.axis_gains(rm, axis)
    a, b = attitude.rate_model(kp, ki, rm["T"], tau, jt)
    return res, rm, jt, tau, kp, ki, a, b


def simulate(axis, k, amp_of_tick, n_ticks, record_from=0):
    """Rate executions 0 .. n_ticks - 1 of the synthetic loop: (theta at each attitude execution, peak |J_a u| over the
    executions from record_from, list of d at the attitude executions)."""
    res, rm, jt, tau, kp, ki, a, b = model(axis)
    n = res["N"]
    j_a = rm["axes"][axis]["J"]
    s = [0.0] * attitude.STATE_N
    hold = 0.0
    theta, d_att, peak = [], [], 0.0
    for i in range(n_ticks):
        if i % n == 0:
            hold = -k * s[attitude.THETA]
            theta.append(s[attitude.THETA])
            d_att.append(amp_of_tick(i))
        r = hold + amp_of_tick(i)
        if i >= record_from:
            peak = max(peak, abs(j_a * (kp * (r - s[1]) + s[3] + ki * rm["T"] * s[4])))
        s = [sum(a[row][c] * s[c] for c in range(attitude.STATE_N)) + b[row] * r for row in range(attitude.STATE_N)]
    return theta, peak, d_att


# ---- attitude_error -------------------------------------------------------------------------------------------------

def quat(axis, angle):
    v = [0.0, 0.0, 0.0]
    v[AXES.index(axis)] = math.sin(angle / 2)
    return (math.cos(angle / 2), *v)


def test_attitude_error_is_the_rotation_on_each_axis_and_the_weighted_yaw():
    w32 = design()["w32"]
    angle = 0.3  # scenario test value: a rotation of 0.3 rad about one axis
    assert run_l5.attitude_error((1.0, 0.0, 0.0, 0.0), w32) == (0.0, 0.0, 0.0)
    for axis in ("roll", "pitch"):
        y = run_l5.attitude_error(quat(axis, angle), w32)
        assert y[AXES.index(axis)] == pytest.approx(TWO * math.sin(angle / 2), abs=1e-15)
        assert all(abs(v) < 1e-15 for i, v in enumerate(y) if i != AXES.index(axis))
    y = run_l5.attitude_error(quat("yaw", angle), w32)
    assert y[2] == pytest.approx(TWO * math.sin(w32 * angle / 2) / w32, abs=1e-15)
    assert y[2] == pytest.approx(angle, rel=1e-3)
    assert run_l5.attitude_error(tuple(-v for v in quat("yaw", angle)), w32) == pytest.approx(y, abs=1e-15)


def test_attitude_error_control_the_yaw_weight_and_the_sign_matter():
    w32 = design()["w32"]
    q = quat("yaw", 1.0)  # scenario test value: a large yaw, where the weight's curvature shows
    assert abs(run_l5.attitude_error(q, w32)[2] - run_l5.attitude_error(q, TWO * w32)[2]) > 1e-3
    assert run_l5.attitude_error(quat("roll", 0.3), w32)[0] > 0 > run_l5.attitude_error(quat("roll", -0.3), w32)[0]


# ---- steady_torque_peak against the stepped loop --------------------------------------------------------------------

def stepped_peak(axis, k, omega):
    res, rm, *_ = model(axis)
    t = rm["T"]
    ticks = round((SETTLE_S + 2 * math.pi / omega) / t)
    _, peak, _ = simulate(axis, k, lambda i: math.sin(omega * i * t), ticks, record_from=round(SETTLE_S / t))
    return peak


@pytest.mark.parametrize("axis", ["roll", "yaw"])
def test_steady_torque_peak_matches_the_stepped_loop(axis):
    res, rm, jt, tau, *_ = model(axis)
    k = attitude.axis_k(axis, res["k32"], res["w32"])
    omega = 6.0  # scenario test value: inside the band, near the torque peak
    fast = run_l5.steady_torque_peak(rm, axis, jt, tau, k, res["N"], omega)
    assert stepped_peak(axis, k, omega) == pytest.approx(fast, rel=REL_TOL)


def test_steady_torque_peak_control_a_planted_gain_is_detected():
    res, rm, jt, tau, *_ = model("roll")
    omega = 6.0
    fast = run_l5.steady_torque_peak(rm, "roll", jt, tau, res["k32"], res["N"], omega)
    assert abs(stepped_peak("roll", TWO * res["k32"], omega) - fast) > 1e-2 * fast


# ---- identification on the synthetic loop ---------------------------------------------------------------------------

DURATION_S = 8.0  # scenario test value: the chirp duration of the scenarios
TAIL_S = 15.0  # scenario test value: their tail


def synthetic(axis="roll", amp=1.0):
    res, rm, *_ = model(axis)
    t, n = rm["T"], res["N"]
    lo, hi = 1.4393843256287513, 11.123489020532052  # the scenarios' band (scenario values, re-derived in the gz test)
    dur_us = round(DURATION_S * 1e6)
    k = attitude.axis_k(axis, res["k32"], res["w32"])
    ticks = round((DURATION_S + TAIL_S) / t)
    theta, _, d_att = simulate(axis, k, lambda i: l4.chirp_value(round(i * t * 1e6), amp, lo, hi, 0, dur_us), ticks)
    return res, rm, theta, d_att, (lo, hi), k


def test_identification_recovers_the_design_margin_on_the_synthetic_loop():
    res, rm, theta, d, band, k = synthetic()
    t_a = res["T_a"]
    m = l4.identify(theta, d, t_a, k, 0.0, band)
    nominal = next(pm for a, name, pm, *_ in res["final"]["detail"] if a == "roll" and name == "nominal")
    omega_c = next(th for a, name, _, th, *_ in res["final"]["detail"] if a == "roll" and name == "nominal") / t_a
    bound = omega_c * rm["T"] * (res["N"] - 1)  # twice the half rate period between the chirp's and the sample's instants
    assert m["pm"] is not None, m["reason"]
    assert abs(m["pm"] - nominal) <= bound, (math.degrees(m["pm"]), math.degrees(nominal), bound)
    assert m["crossover"] == pytest.approx(omega_c, rel=1e-2)


def test_identification_control_a_wrong_gain_is_detected():
    res, rm, theta, d, band, k = synthetic()
    nominal = next(pm for a, name, pm, *_ in res["final"]["detail"] if a == "roll" and name == "nominal")
    m = l4.identify(theta, d, res["T_a"], k * TWO, 0.0, band)
    omega_c = next(th for a, name, _, th, *_ in res["final"]["detail"] if a == "roll" and name == "nominal") / res["T_a"]
    assert m["pm"] is None or abs(m["pm"] - nominal) > omega_c * rm["T"] * (res["N"] - 1)


# ---- scenarios ------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("axis", AXES)
def test_chirp_scenarios_load_and_hold_the_window(axis):
    vals = l5s.values(l5s.load(SCEN / f"chirp_{axis}.yaml"))
    c = vals["script"]["chirp"]
    assert c["axis"] == axis and 0 < c["w_lo_rad_s"] < c["w_hi_rad_s"] and c["amp_rad_s"] > 0
    assert vals["script"]["segments"] == [] and vals["m_sequence"] == [TWO, 1]
    t_a = float(design()["T_a"])
    assert vals["script"]["end_attitude_execution"] * t_a > c["duration_s"] + c["start_attitude_execution"] * t_a


def test_scenario_control_a_chirp_without_its_band_is_refused(tmp_path):
    text = (SCEN / "chirp_roll.yaml").read_text(encoding="utf-8").replace("w_hi_rad_s:\n      value: ", "w_hi_rad_s:\n      value: 0.0 #")
    path = tmp_path / "bad.yaml"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(Exception):
        l5s.load(path)


# ---- SIM-7 with a planted U ------------------------------------------------------------------------------------------

def sim7_n(u_value):
    rm = design()["rate"]
    import schema
    u = {"value": u_value, "method": "measured", "source": "test", "rule": "test"}
    return attitude.design(schema.load_yaml(CARD), schema.load_yaml(ROOT / "design" / "budget.yaml"),
                           schema.load_yaml(ROOT / "design" / "scenario_values.yaml"), str(CARD), u, rate_result=rm)["N"]


def test_sim7_n_star_follows_u_and_a_planted_u_moves_it():
    res = design()
    rows = [r for r in res["table"] if r["delta"] is not None]
    live = sim7_n(res["U"]["value"])
    assert live == res["N"]
    assert sim7_n(rows[0]["delta"] * HALF) == 1, "U below the first halving difference gives N* = 1"
    assert sim7_n(rows[1]["delta"] * TWO) > live, "U above the second halving difference raises N*"
