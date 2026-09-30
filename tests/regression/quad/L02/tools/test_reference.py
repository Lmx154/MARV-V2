"""Analytic references of the L2 T4 scenarios (tools/sim/reference.py, docs/decisions/0003 items 7, 9, 11). L02 tests.

Every metric test has a negative control: a wrong-sign height gradient, a DShot command off by one, a planted drift.
Test values are labelled: the fine-solver step counts (2^13 and 2^14 for the direct RK4 cross-checks), the central
difference half-width 1e-3 s, the brute-force tick-held solver's 1 and 2 RK4 substeps per tick (a Richardson pair), the synthetic axisymmetric spin (a = 2 rad/s about x, w_z = 5 rad/s, the rotation
scenario's x and z rates) with I_x = I_y = 0.0025 and I_z = 0.0043 kg m^2 (the card's I_x and I_z).
"""

import math
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_plant_config as gpc  # noqa: E402
import hover as hover_mod  # noqa: E402
import reference as ref  # noqa: E402
import schema  # noqa: E402
import t4_rule  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L02"
US = Fraction(1, 10 ** 6)  # unit conversion, microseconds to seconds
FINE_N = 1 << 13
DELTA = 1e-3
RK4_SCALE = 2 ** ref.RK4_ORDER - 1

CARD_V = ref.load_card(CARD)
TICK_S = float(Fraction(625, 4) * US)  # the scenarios' tick period, correctly rounded: bitwise the plugin's tick_period_s


def scenario(name):
    doc = schema.load_yaml(SCEN / f"{name}.yaml")
    ticks = doc["duration_ticks"]["value"]
    period = Fraction(doc["tick_period_num_us"]["value"], doc["tick_period_den"]["value"]) * US
    return {
        "phi": doc["site_latitude_rad"]["value"], "h0": doc["site_height_m"]["value"], "ticks": ticks,
        "T": float(ticks * period), "rates": doc["initial_state"]["body_rates_frd_rad_s"]["value"],
    }


FF = scenario("free_fall")
HV = scenario("hover")
RT = scenario("rotation")
D_LO, D_HI = hover_mod.bracket_for_card(gpc.plant_config(CARD), HV["phi"], HV["h0"])[2:]


def direct_rk4(accel, t_end, n):
    """Independent fixed-step RK4 of p'' = accel(t, p) on the full state, no splitting; returns (p, v)."""
    h = t_end / n
    p = v = 0.0
    for i in range(n):
        t = i * h
        k1p, k1v = v, accel(t, p)
        k2p, k2v = v + 0.5 * h * k1v, accel(t + 0.5 * h, p + 0.5 * h * k1p)
        k3p, k3v = v + 0.5 * h * k2v, accel(t + 0.5 * h, p + 0.5 * h * k2p)
        k4p, k4v = v + h * k3v, accel(t + h, p + h * k3p)
        p += h * (k1p + 2.0 * k2p + 2.0 * k3p + k4p) / 6.0
        v += h * (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0
    return p, v


def const_g(value):
    return lambda phi, h: value


def test_scenario_times_are_the_committed_ones():
    assert FF["T"] == 1.0 and HV["T"] == 2.0 and RT["T"] == 2.0
    assert (FF["ticks"], HV["ticks"], RT["ticks"]) == (6400, 12800, 12800)
    assert (D_LO, D_HI) == (765, 766)


def test_card_values_agree_with_the_plant_configuration():
    cfg = gpc.plant_config(CARD)
    assert (CARD_V.mass_kg, CARD_V.thrust_coeff, CARD_V.tau_s) == (cfg["mass_kg"], cfg["thrust_coeff"], cfg["motor_tau_s"])
    assert (CARD_V.omega_min, CARD_V.omega_max) == (cfg["omega_min_rad_s"], cfg["omega_max_rad_s"])
    assert CARD_V.inertia_diag == (0.0025, 0.0021, 0.0043)


def test_gravity_is_the_hover_module_not_a_copy():
    assert ref.free_fall.__defaults__[0] is hover_mod.normal_gravity
    assert ref.hover is hover_mod  # the module object itself
    assert ref.hover_state.__defaults__[0] is hover_mod.normal_gravity
    assert ref.hover_state_tick_held.__defaults__[0] is hover_mod.normal_gravity


def test_esc_map_endpoints():
    assert ref.esc_omega(CARD_V, 48) == CARD_V.omega_min
    assert ref.esc_omega(CARD_V, 2047) == CARD_V.omega_max
    assert ref.esc_omega(CARD_V, 0) == 0.0


# ---- free fall -------------------------------------------------------------------------------------------------

def test_free_fall_with_constant_gravity_is_the_closed_form():
    g0 = hover_mod.normal_gravity(FF["phi"], FF["h0"])
    for t in (0.25, 0.5, FF["T"]):
        r = ref.free_fall(t, FF["phi"], FF["h0"], const_g(g0))
        assert abs(r.p - g0 * t * t / 2.0) <= r.F_p and abs(r.v - g0 * t) <= r.F_v and r.a == g0
        assert r.trunc_p == 0.0 and r.trunc_v == 0.0  # delta = 0: the reference is the closed form


def test_free_fall_at_zero_time_is_the_initial_state():
    r = ref.free_fall(0.0, FF["phi"], FF["h0"])
    assert (r.p, r.v, r.n_steps) == (0.0, 0.0, 0)
    assert r.a == hover_mod.normal_gravity(FF["phi"], FF["h0"])


def test_free_fall_matches_an_independent_direct_rk4_of_the_full_ode():
    phi, h0 = FF["phi"], FF["h0"]
    r = ref.free_fall(FF["T"], phi, h0)
    acc = lambda t, p: hover_mod.normal_gravity(phi, h0 - p)  # noqa: E731
    p1, v1 = direct_rk4(acc, FF["T"], FINE_N)
    p2, v2 = direct_rk4(acc, FF["T"], 2 * FINE_N)
    tol_p = abs(p2 - p1) / RK4_SCALE + r.F_p
    tol_v = abs(v2 - v1) / RK4_SCALE + r.F_v
    assert abs(r.p - p2) <= tol_p and abs(r.v - v2) <= tol_v


def test_free_fall_gradient_matters_more_than_the_floor():
    phi, h0, T = FF["phi"], FF["h0"], FF["T"]
    g0 = hover_mod.normal_gravity(phi, h0)
    r = ref.free_fall(T, phi, h0)
    F_p = t4_rule.floor(FF["ticks"], r.Q_p, ref.FREE_FALL_ROUNDINGS)
    F_v = t4_rule.floor(FF["ticks"], r.Q_v, ref.FREE_FALL_ROUNDINGS)
    assert r.p - g0 * T * T / 2.0 > 1e4 * F_p  # falling: g grows with depth, so the fall is longer
    assert r.v - g0 * T > 1e4 * F_v


def test_negative_control_wrong_sign_gradient_is_distinguished_from_the_reference():
    phi, h0, T = FF["phi"], FF["h0"], FF["T"]
    g0 = hover_mod.normal_gravity(phi, h0)
    wrong = lambda ph, h: 2.0 * hover_mod.normal_gravity(ph, h0) - hover_mod.normal_gravity(ph, h)  # noqa: E731
    r = ref.free_fall(T, phi, h0)
    w = ref.free_fall(T, phi, h0, wrong)
    F_p = t4_rule.floor(FF["ticks"], r.Q_p, ref.FREE_FALL_ROUNDINGS)
    assert w.p - g0 * T * T / 2.0 < -1e4 * F_p  # the wrong-sign fall is shorter than the constant-g one
    assert abs(w.p - r.p) > 1e4 * F_p and abs(w.p - r.p) > r.F_p + w.F_p
    # the constant-g closed form fails against the true reference under the item 9 tolerance
    v = t4_rule.evaluate([r.p] * 3, g0 * T * T / 2.0, F_p, r.F_p)
    assert not v.passed


def test_free_fall_reference_error_is_below_the_floor_by_halving_the_solver():
    phi, h0, T = FF["phi"], FF["h0"], FF["T"]
    r = ref.free_fall(T, phi, h0)
    F_p = t4_rule.floor(FF["ticks"], r.Q_p, ref.FREE_FALL_ROUNDINGS)
    F_v = t4_rule.floor(FF["ticks"], r.Q_v, ref.FREE_FALL_ROUNDINGS)
    assert r.F_p <= F_p and r.F_v <= F_v
    assert r.trunc_p <= t4_rule.U * r.Q_p and r.trunc_v <= t4_rule.U * r.Q_v
    print(f"free_fall T={T}: n={r.n_steps} trunc_p={r.trunc_p:.3g} F_ref_p={r.F_p:.3g} F_p={F_p:.3g} "
          f"F_ref_v={r.F_v:.3g} F_v={F_v:.3g}")


def test_free_fall_halving_the_solver_changes_the_result_by_the_reported_estimate():
    phi, h0, T = FF["phi"], FF["h0"], FF["T"]
    g0 = hover_mod.normal_gravity(phi, h0)
    n = ref.free_fall(T, phi, h0).n_steps
    rhs = lambda t, d: hover_mod.normal_gravity(phi, h0 - g0 * t * t / 2.0 - d) - g0  # noqa: E731
    coarse = ref._rk4(rhs, T, n // 2)
    fine = ref._rk4(rhs, T, n)
    assert abs(fine[0] - coarse[0]) / RK4_SCALE == pytest.approx(ref.free_fall(T, phi, h0).trunc_p, rel=1e-9)


def test_gravity_height_gradient_matches_a_central_difference():
    phi, h = FF["phi"], FF["h0"]
    fd = (hover_mod.normal_gravity(phi, h + 1.0) - hover_mod.normal_gravity(phi, h - 1.0)) / 2.0
    assert ref.gravity_height_gradient(phi, h) == pytest.approx(fd, rel=1e-6)
    assert ref.gravity_height_gradient(phi, h) < 0.0
    assert ref.gravity_height_gradient(phi, h, const_g(9.8)) == 0.0


# ---- hover -----------------------------------------------------------------------------------------------------

def hover_at(t, dshot, gravity=hover_mod.normal_gravity):
    return ref.hover_state(t, CARD_V, dshot, HV["phi"], HV["h0"], gravity)


def test_hover_with_constant_gravity_matches_a_direct_rk4_of_the_lagged_thrust():
    g0 = hover_mod.normal_gravity(HV["phi"], HV["h0"])
    for d in (D_LO, D_HI):
        acc = lambda t, p, d=d: g0 - ref.thrust_accel(CARD_V, d, t)  # noqa: E731
        for t in (0.05, 0.5, HV["T"]):
            r = hover_at(t, d, const_g(g0))
            p1, v1 = direct_rk4(acc, t, FINE_N)
            p2, v2 = direct_rk4(acc, t, 2 * FINE_N)
            assert abs(r.p - p2) <= abs(p2 - p1) / RK4_SCALE + r.F_p
            assert abs(r.v - v2) <= abs(v2 - v1) / RK4_SCALE + r.F_v


def test_hover_matches_a_direct_rk4_with_the_height_gradient():
    phi, h0 = HV["phi"], HV["h0"]
    acc = lambda t, p: hover_mod.normal_gravity(phi, h0 - p) - ref.thrust_accel(CARD_V, D_LO, t)  # noqa: E731
    r = hover_at(HV["T"], D_LO)
    p1, v1 = direct_rk4(acc, HV["T"], FINE_N)
    p2, v2 = direct_rk4(acc, HV["T"], 2 * FINE_N)
    assert abs(r.p - p2) <= abs(p2 - p1) / RK4_SCALE + r.F_p
    assert abs(r.v - v2) <= abs(v2 - v1) / RK4_SCALE + r.F_v


def five_point(f, t, delta):
    return (-f(t + 2.0 * delta) + 8.0 * f(t + delta) - 8.0 * f(t - delta) + f(t - 2.0 * delta)) / (12.0 * delta)


def fd_of_v(dshot, t, gravity=hover_mod.normal_gravity):
    v = lambda x: hover_at(x, dshot, gravity).v  # noqa: E731
    fd1 = five_point(v, t, DELTA)
    fd2 = five_point(v, t, DELTA / 2.0)
    r = hover_at(t, dshot, gravity)
    # tolerance: Richardson estimate of the stencil (order 4), the stencil's amplification of the reference error in v
    # (sum |coefficients| = 18 / (12 delta)), and the reference's own error in a
    tol = abs(fd1 - fd2) * 2 ** 4 / (2 ** 4 - 1) + 18.0 / (12.0 * DELTA) * r.F_v + r.F_a
    return fd2, tol


def test_hover_acceleration_equals_the_finite_difference_of_the_velocity():
    for d in (D_LO, D_HI):
        for t in (0.05, 0.2, 1.0, HV["T"] - 2.0 * DELTA):
            fd, tol = fd_of_v(d, t)
            assert abs(hover_at(t, d).a - fd) <= tol, (d, t)


def test_negative_control_wrong_dshot_breaks_the_acceleration_check():
    for t in (0.05, 0.2, 1.0):
        fd, tol = fd_of_v(D_LO, t)
        for wrong in (D_LO - 1, D_LO + 1):
            assert abs(hover_at(t, wrong).a - fd) > 100.0 * tol, (wrong, t)


def test_negative_control_planted_velocity_drift_breaks_the_acceleration_check():
    drift = 1e-6  # m/s^2, planted
    fd, tol = fd_of_v(D_LO, 1.0)
    assert abs(hover_at(1.0, D_LO).a - (fd + drift)) > tol


def test_hover_gradient_matters_at_the_check_time():
    g0 = hover_mod.normal_gravity(HV["phi"], HV["h0"])
    r = hover_at(HV["T"], D_LO)
    c = hover_at(HV["T"], D_LO, const_g(g0))
    F_p = t4_rule.floor(HV["ticks"], r.Q_p, ref.HOVER_ROUNDINGS)
    assert abs(r.p - c.p) > 1e2 * F_p


def test_hover_settling_time_and_floor():
    t_s, f_a, a_T = ref.hover_settling(CARD_V, D_LO)
    assert f_a == (1 + ref.HOVER_ROUNDINGS) * t4_rule.U * a_T
    assert 2.0 * a_T * math.exp(-t_s / CARD_V.tau_s) == pytest.approx(f_a, rel=1e-9)
    assert t_s == pytest.approx(CARD_V.tau_s * math.log(2.0 * a_T / f_a), rel=1e-15)
    assert 0.0 < t_s < HV["T"]  # the committed check time is late enough
    assert ref.hover_settling(CARD_V, D_LO, HV["ticks"])[0] < t_s  # a larger floor settles sooner
    print(f"hover D_lo: t_s={t_s:.4g} s (n_a=1) F_a={f_a:.3g} a_T={a_T:.6g}; "
          f"n_a={HV['ticks']}: t_s={ref.hover_settling(CARD_V, D_LO, HV['ticks'])[0]:.4g} s")


def test_hover_acceleration_has_settled_to_the_floor_by_t_s_and_not_before():
    t_s, f_a, a_T = ref.hover_settling(CARD_V, D_LO)
    g0 = hover_mod.normal_gravity(HV["phi"], HV["h0"])

    def transient(t):
        r = hover_at(t, D_LO)
        return abs(r.a - (hover_mod.normal_gravity(HV["phi"], HV["h0"] - r.p) - a_T))

    assert transient(t_s) <= f_a * (1.0 + 1e-6)  # (1 - e)^2 = 1 - 2e + e^2: the e^2 term is below F_a / 2
    assert transient(HV["T"]) <= f_a
    assert transient(t_s / 4.0) > 1e3 * f_a  # negative control: well before t_s the transient is far above the floor
    assert g0 > 0.0


def test_hover_reference_error_is_below_the_floor():
    for d in (D_LO, D_HI):
        r = hover_at(HV["T"], d)
        F_p = t4_rule.floor(HV["ticks"], r.Q_p, ref.HOVER_ROUNDINGS)
        F_v = t4_rule.floor(HV["ticks"], r.Q_v, ref.HOVER_ROUNDINGS)
        F_a = t4_rule.floor(HV["ticks"], r.Q_a, ref.HOVER_ROUNDINGS)
        assert r.F_p <= F_p and r.F_v <= F_v and r.F_a <= F_a
        print(f"hover D={d} T={HV['T']}: n={r.n_steps} F_ref_p={r.F_p:.3g} F_p={F_p:.3g} F_ref_v={r.F_v:.3g} "
              f"F_v={F_v:.3g} F_ref_a={r.F_a:.3g} F_a={F_a:.3g}")


def test_bracket_signs_for_the_committed_card():
    a_lo = hover_at(HV["T"], D_LO).a
    a_hi = hover_at(HV["T"], D_HI).a
    assert a_lo > 0.0 > a_hi
    assert ref.bracket_sign_ok(a_lo, a_hi)
    bound = ref.bracket_bound(CARD_V, D_LO, D_HI)
    assert bound > 0.0 and ref.bracket_magnitude_ok(a_lo, CARD_V, D_LO, D_HI)
    assert ref.bracket_magnitude_ok(a_hi, CARD_V, D_LO, D_HI)
    # the two settled accelerations differ by one command step, up to g's height gradient over the two heights
    r_lo, r_hi = hover_at(HV["T"], D_LO), hover_at(HV["T"], D_HI)
    grad = abs(ref.gravity_height_gradient(HV["phi"], HV["h0"] - r_lo.p))
    assert abs((a_lo - a_hi) - bound) <= grad * abs(r_lo.p - r_hi.p) * 1.01 + r_lo.F_a + r_hi.F_a
    assert abs((a_lo - a_hi) - bound) > 100.0 * (r_lo.F_a + r_hi.F_a)  # and the gradient term is not negligible


def test_negative_control_shifted_bracket_fails_the_signs():
    for lo, hi in ((D_LO - 1, D_HI - 1), (D_LO + 1, D_HI + 1)):
        assert not ref.bracket_sign_ok(hover_at(HV["T"], lo).a, hover_at(HV["T"], hi).a)


def test_negative_control_command_off_by_one_breaks_the_magnitude_bound():
    for wrong in (D_LO - 1, D_HI + 1):
        a = hover_at(HV["T"], wrong).a
        assert not ref.bracket_magnitude_ok(a, CARD_V, D_LO, D_HI), wrong


def test_bracket_sign_and_bound_functions_on_plain_numbers():
    assert ref.bracket_sign_ok(1e-3, -1e-3)
    assert not ref.bracket_sign_ok(0.0, -1e-3) and not ref.bracket_sign_ok(1e-3, 0.0)
    assert not ref.bracket_sign_ok(-1e-3, 1e-3) and not ref.bracket_sign_ok(1e-3, 1e-3)


# ---- tick-held hover (0003 item 7, last paragraph) ----------------------------------------------------------------

def tick_at(n, dshot, gravity=hover_mod.normal_gravity, hold=1):
    return ref.hover_state_tick_held(n, TICK_S, CARD_V, dshot, HV["phi"], HV["h0"], gravity, hold)


def brute_tick_held(n, dshot, gravity, sub, hold=1):
    """Brute-force per-tick simulation, independent of the reference's closed forms: the thrust of tick j from
    w = w_D (1 - exp(-(j + hold) h / tau)) (exp, where the reference uses expm1 and prefix sums), the full ODE
    p'' = g(phi, h0 - p) - a_j integrated by RK4 with `sub` substeps inside each tick (so the RK4 never straddles a thrust
    step), and p, v accumulated exactly (Fractions of the float increments), so the summation adds no roundoff."""
    phi, h0, h = HV["phi"], HV["h0"], TICK_S
    w_d = ref.esc_omega(CARD_V, dshot)
    p_x = v_x = Fraction(0)
    hs = h / sub
    for j in range(n):
        w = w_d * (1.0 - math.exp(-((j + hold) * h) / CARD_V.tau_s))
        a_j = ref.N_ROTORS * CARD_V.thrust_coeff * w * w / CARD_V.mass_kg
        for _ in range(sub):
            p, v = float(p_x), float(v_x)
            acc = lambda pp: gravity(phi, h0 - pp) - a_j  # noqa: E731
            k1p, k1v = v, acc(p)
            k2p, k2v = v + 0.5 * hs * k1v, acc(p + 0.5 * hs * k1p)
            k3p, k3v = v + 0.5 * hs * k2v, acc(p + 0.5 * hs * k2p)
            k4p, k4v = v + hs * k3v, acc(p + hs * k3p)
            p_x += Fraction(hs * (k1p + 2.0 * k2p + 2.0 * k3p + k4p) / 6.0)
            v_x += Fraction(hs * (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0)
    return float(p_x), float(v_x)


def test_tick_held_matches_a_brute_force_per_tick_simulation_with_constant_gravity():
    g0 = hover_mod.normal_gravity(HV["phi"], HV["h0"])
    for d in (D_LO, D_HI):
        for n in (1, 7, 640, HV["ticks"]):
            r = tick_at(n, d, const_g(g0))
            p, v = brute_tick_held(n, d, const_g(g0), 1)
            assert r.trunc_p == 0.0 and r.trunc_v == 0.0
            assert abs(r.p - p) <= r.F_p and abs(r.v - v) <= r.F_v, (d, n, r.p - p, r.F_p, r.v - v, r.F_v)


def test_tick_held_matches_a_brute_force_per_tick_simulation_with_the_height_gradient():
    for d in (D_LO, D_HI):
        for n in (640, HV["ticks"]):
            r = tick_at(n, d)
            p1, v1 = brute_tick_held(n, d, hover_mod.normal_gravity, 1)
            p2, v2 = brute_tick_held(n, d, hover_mod.normal_gravity, 2)
            tol_p = abs(p2 - p1) / RK4_SCALE + r.F_p
            tol_v = abs(v2 - v1) / RK4_SCALE + r.F_v
            assert abs(r.p - p2) <= tol_p and abs(r.v - v2) <= tol_v, (d, n, r.p - p2, tol_p, r.v - v2, tol_v)
            print(f"tick-held D={d} n={n}: p={r.p:.15g} v={r.v:.15g} F_ref_p={r.F_p:.3g} F_ref_v={r.F_v:.3g} "
                  f"brute-force |dp|={abs(r.p - p2):.3g} |dv|={abs(r.v - v2):.3g} solver n={r.n_steps}")


def test_negative_control_brute_force_with_a_wrong_command_is_not_the_tick_held_reference():
    r = tick_at(HV["ticks"], D_LO)
    p, v = brute_tick_held(HV["ticks"], D_LO + 1, hover_mod.normal_gravity, 1)
    assert abs(r.v - v) > 1e3 * r.F_v and abs(r.p - p) > 1e3 * r.F_p


def test_tick_held_reference_error_is_below_the_floor():
    for d in (D_LO, D_HI):
        r = tick_at(HV["ticks"], d)
        F_p = t4_rule.floor(HV["ticks"], r.Q_p, ref.HOVER_ROUNDINGS)
        F_v = t4_rule.floor(HV["ticks"], r.Q_v, ref.HOVER_ROUNDINGS)
        assert r.F_p <= F_p and r.F_v <= F_v
        assert r.trunc_p <= t4_rule.U * r.Q_p and r.trunc_v <= t4_rule.U * r.Q_v
        print(f"tick-held D={d} T={HV['T']}: n={r.n_steps} trunc_p={r.trunc_p:.3g} F_ref_p={r.F_p:.3g} F_p={F_p:.3g} "
              f"trunc_v={r.trunc_v:.3g} F_ref_v={r.F_v:.3g} F_v={F_v:.3g}")


def test_tick_held_at_zero_ticks_and_a_fractional_tick_count():
    r = tick_at(0, D_LO)
    assert (r.p, r.v, r.n_steps) == (0.0, 0.0, 0)
    assert r.a == hover_mod.normal_gravity(HV["phi"], HV["h0"]) - ref.tick_held_thrust_accel(CARD_V, D_LO, 0, TICK_S)
    with pytest.raises(ref.ReferenceError):
        tick_at(1.5, D_LO)


def test_tick_held_thrust_is_the_end_of_step_lag():
    """The plant holds w(t_{j+1}) over tick j: the first tick already carries the thrust of one tick of spin-up."""
    x = TICK_S / CARD_V.tau_s
    w1 = ref.esc_omega(CARD_V, D_LO) * (1.0 - math.exp(-x))
    assert ref.tick_held_omega(CARD_V, D_LO, 0, TICK_S) == pytest.approx(w1, rel=1e-14)
    assert ref.tick_held_omega(CARD_V, D_LO, 0, TICK_S, hold=0) == 0.0
    assert ref.tick_held_omega(CARD_V, D_LO, 5, TICK_S, hold=0) == ref.tick_held_omega(CARD_V, D_LO, 4, TICK_S)


def test_tick_held_differs_from_the_continuous_reference_by_half_a_tick_of_thrust():
    """v_cont - v_tick = (right Riemann sum - integral) of the thrust acceleration a(s) = a_T (1 - e^-(s/tau))^2. By the
    Euler-Maclaurin series that is (h/2)(a(t) - a(0)) + (h^2/12)(a1(t) - a1(0)) - (h^4/720)(a3(t) - a3(0)) + ... with a1,
    a3 the first and third derivatives; a(0) = a1(0) = 0 and a3(0) = -6 a_T / tau^3. So the difference is (1/2) a_T h
    (1 + O(E)), E = e^-(t/tau), plus the h^2 and h^4 terms. The height gradient adds a coupled term: the two positions differ
    by at most a_T h t, which moves g by |dg/dh| a_T h t and v by at most |dg/dh| a_T h t^2. Tolerance: the series terms
    beyond (1/2) a_T h (the h^4 one doubled for the next), the gradient bound, and both references' floors."""
    h, tau = TICK_S, CARD_V.tau_s
    for d in (D_LO, D_HI):
        a_T = ref.hover_a_T(CARD_V, d)
        for n in (1280, HV["ticks"]):
            t = n * h
            e = math.exp(-t / tau)
            cont, tick = hover_at(t, d), tick_at(n, d)
            grad = abs(ref.gravity_height_gradient(HV["phi"], HV["h0"] - tick.p))
            predicted = h / 2.0 * a_T * (1.0 - e) ** 2 + h * h / 12.0 * 2.0 * a_T * (1.0 - e) * e / tau
            tol = 2.0 * h ** 4 / 720.0 * 6.0 * a_T / tau ** 3 + grad * a_T * h * t * t + cont.F_v + tick.F_v
            assert abs((cont.v - tick.v) - predicted) <= tol, (d, n, (cont.v - tick.v) - predicted, tol)
            print(f"tick-held D={d} n={n}: v_cont - v_tick = {cont.v - tick.v:.9e}, series {predicted:.9e}, "
                  f"a_T h / 2 = {h / 2.0 * a_T:.9e}, diff to series {(cont.v - tick.v) - predicted:.3e} <= tol {tol:.3e}")
    # the settled limit: E ~ 1e-26 at T, so the difference is (1/2) a_T h up to the gradient term and the floors
    for d in (D_LO, D_HI):
        a_T = ref.hover_a_T(CARD_V, d)
        cont, tick = hover_at(HV["T"], d), tick_at(HV["ticks"], d)
        grad = abs(ref.gravity_height_gradient(HV["phi"], HV["h0"] - tick.p))
        assert abs((cont.v - tick.v) - h / 2.0 * a_T) <= grad * a_T * h * HV["T"] ** 2 + cont.F_v + tick.F_v
        assert abs((cont.v - tick.v) - h / 2.0 * a_T) < 1e-4 * (cont.v - tick.v)


def test_negative_control_start_of_tick_hold_is_not_the_tick_held_reference():
    """Holding w(t_j) over tick j instead of w(t_{j+1}) moves v by h (a(t) - a(0)) = a_T h (1 + O(E)): more than F."""
    d, n = D_LO, HV["ticks"]
    end, start = tick_at(n, d), tick_at(n, d, hold=0)
    F_v = t4_rule.floor(n, end.Q_v, ref.HOVER_ROUNDINGS)
    F_p = t4_rule.floor(n, end.Q_p, ref.HOVER_ROUNDINGS)
    assert abs(end.v - start.v) > 1e6 * (F_v + end.F_v + start.F_v)
    assert abs(end.p - start.p) > 1e6 * (F_p + end.F_p + start.F_p)
    a_T = ref.hover_a_T(CARD_V, d)
    grad = abs(ref.gravity_height_gradient(HV["phi"], HV["h0"] - end.p))
    assert abs((start.v - end.v) - TICK_S * a_T) <= grad * a_T * TICK_S * 2.0 * HV["T"] ** 2 + F_v + end.F_v + start.F_v


def test_tick_held_acceleration_at_the_check_time_is_the_continuous_one_to_the_floor():
    """a over the tick that starts at T carries w(T + h) instead of w(T): 2 a_T (1 - E) E h / tau, plus the p shift."""
    h, tau = TICK_S, CARD_V.tau_s
    for d in (D_LO, D_HI):
        a_T = ref.hover_a_T(CARD_V, d)
        cont, tick = hover_at(HV["T"], d), tick_at(HV["ticks"], d)
        grad = abs(ref.gravity_height_gradient(HV["phi"], HV["h0"] - cont.p))
        shift = 2.0 * a_T * math.exp(-HV["T"] / tau) * h / tau + grad * abs(cont.p - tick.p)
        assert abs(cont.a - tick.a) <= shift + cont.F_a + tick.F_a
        print(f"a_D at T, D={d}: continuous {cont.a:.12e}, tick-held {tick.a:.12e}, diff {cont.a - tick.a:.3e} "
              f"(p shift term {grad * abs(cont.p - tick.p):.3e}, F_a {cont.F_a:.3g})")


# ---- torque-free rotation --------------------------------------------------------------------------------------

IX, IZ = CARD_V.inertia_diag[0], CARD_V.inertia_diag[2]
SPIN_A, SPIN_Z = RT["rates"][0], RT["rates"][2]
OMEGA_PREC = (IZ - IX) * SPIN_Z / IX


def axisymmetric(t):
    """Exact torque-free solution for I = diag(Ix, Ix, Iz): w = (a cos Wt, a sin Wt, w_z), W = (Iz - Ix) w_z / Ix."""
    return (SPIN_A * math.cos(OMEGA_PREC * t), SPIN_A * math.sin(OMEGA_PREC * t), SPIN_Z)


def test_axisymmetric_solution_satisfies_eulers_equations():
    inertia = (IX, IX, IZ)
    for t in (0.0, 0.3, 1.7):
        w = axisymmetric(t)
        wdot = (-SPIN_A * OMEGA_PREC * math.sin(OMEGA_PREC * t), SPIN_A * OMEGA_PREC * math.cos(OMEGA_PREC * t), 0.0)
        # I w' = (I w) x w
        l = [i * x for i, x in zip(inertia, w)]
        cross = (l[1] * w[2] - l[2] * w[1], l[2] * w[0] - l[0] * w[2], l[0] * w[1] - l[1] * w[0])
        for i in range(3):
            assert inertia[i] * wdot[i] == pytest.approx(cross[i], abs=1e-14)


def test_rotation_invariants_of_the_closed_form_are_constant_within_the_floor():
    inertia = (IX, IX, IZ)
    first = ref.rotation_reference(axisymmetric(0.0), inertia)
    assert first.L == math.sqrt((IX * SPIN_A) ** 2 + (IZ * SPIN_Z) ** 2) or first.L == pytest.approx(
        math.hypot(IX * SPIN_A, IZ * SPIN_Z), rel=1e-15)
    assert first.KE == pytest.approx(0.5 * (IX * SPIN_A ** 2 + IZ * SPIN_Z ** 2), rel=1e-15)
    n = RT["ticks"]
    f_l, f_ke = ref.rotation_floors(n, first)
    period = RT["T"] / n
    dev_l = dev_ke = 0.0
    for j in range(n + 1):
        l_norm, ke = ref.rotation_invariants(axisymmetric(j * period), inertia)
        dev_l = max(dev_l, abs(l_norm - first.L))
        dev_ke = max(dev_ke, abs(ke - first.KE))
    assert dev_l <= f_l and dev_ke <= f_ke
    assert first.F_ref_L <= f_l and first.F_ref_KE <= f_ke
    print(f"rotation T={RT['T']}: N={n} |L|={first.L:.6g} KE={first.KE:.6g} F_ref_L={first.F_ref_L:.3g} "
          f"F_L={f_l:.3g} F_ref_KE={first.F_ref_KE:.3g} F_KE={f_ke:.3g}; synthetic max dev {dev_l:.3g} {dev_ke:.3g}")


def test_negative_control_planted_drift_breaks_the_rotation_invariants():
    inertia = (IX, IX, IZ)
    first = ref.rotation_reference(axisymmetric(0.0), inertia)
    n = RT["ticks"]
    f_l, f_ke = ref.rotation_floors(n, first)
    eps = 1e-9  # planted relative drift of the spin over the run
    w = axisymmetric(RT["T"])
    drifted = (w[0], w[1], w[2] * (1.0 + eps))
    l_norm, ke = ref.rotation_invariants(drifted, inertia)
    assert abs(l_norm - first.L) > f_l and abs(ke - first.KE) > f_ke
    # a wrong inertia (I_z swapped for I_y of the card) breaks the invariants of the true motion
    l_bad, ke_bad = ref.rotation_invariants(axisymmetric(RT["T"]), (IX, IX, CARD_V.inertia_diag[1]))
    assert abs(l_bad - first.L) > f_l and abs(ke_bad - first.KE) > f_ke


def test_rotation_invariants_through_the_t4_rule():
    inertia = (IX, IX, IZ)
    first = ref.rotation_reference(axisymmetric(0.0), inertia)
    f_l, _ = ref.rotation_floors(RT["ticks"], first)
    q = [ref.rotation_invariants(axisymmetric(RT["T"]), inertia)[0]] * 3  # the same value at every step size
    assert t4_rule.evaluate(q, first.L, f_l, first.F_ref_L).branch == t4_rule.BRANCH_I
    w = axisymmetric(RT["T"])
    bad = [ref.rotation_invariants((w[0], w[1], w[2] * (1.0 + 1e-9)), inertia)[0]] * 3
    assert not t4_rule.evaluate(bad, first.L, f_l, first.F_ref_L).passed


def test_rotation_invariants_of_the_committed_initial_rates():
    r = ref.rotation_reference(RT["rates"], CARD_V.inertia_diag)
    n = RT["ticks"]
    f_l, f_ke = ref.rotation_floors(n, r)
    assert r.F_ref_L <= f_l and r.F_ref_KE <= f_ke and r.F_ref_L > 0.0
    lx, ly, lz = (i * w for i, w in zip(CARD_V.inertia_diag, RT["rates"]))
    assert r.L == pytest.approx(math.sqrt(lx * lx + ly * ly + lz * lz), rel=1e-15)
    print(f"rotation committed rates: |L|={r.L:.6g} KE={r.KE:.6g} F_ref_L={r.F_ref_L:.3g} F_L={f_l:.3g} "
          f"F_ref_KE={r.F_ref_KE:.3g} F_KE={f_ke:.3g}")
