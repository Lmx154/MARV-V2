"""The T4 chirp identification and its design quantities (tools/sim/run_l4.py section "T4 chirp margins"), without gz.
L04 tests, host-only.

Synthetic loop. The design model of tools/card/rate.py (J w' = m, tau m' = v - m, the input held over each rate-loop
period T, the gyro sampled at the period's start) is simulated here in binary64 from its exact zero-order-hold solution,
in physical units with the axis's J and the emitted float32 gains, under the firmware's law at a constant dt = T
(u_n = kp e_n + I_n, I_n = I_(n-1) + ki e_(n-1) T, e = -y: setpoint and reference 0) with the chirp added at the plant
input, v = u + d. It is linear and time-invariant, so its phase margin is known without the estimator:
rate.loop_crossover evaluates L(e^{jwT}) from its closed-form factors. run_l4.identify on the simulated (y, d) must
recover that margin within TOL_ID, at nominal and at the corner (J-, tau+) whose margin is PM_min.

Scenario test values (each with its reason):
  AMPLITUDE = 0.5 N m      any value serves (the loop is linear); about the roll amplitude of the Gazebo chirp.
  DURATION_S = 8 s, TAIL_S = 5 s, T0_US = 0   the committed chirp scenarios' duration and tail, from stamp 0.
  TOL_ID = 2^-20 rad (9.5e-7 rad, 5.5e-5 deg)  above the estimator's error on these loops by more than five decades
      (below 1e-12 rad on 2026-09-30; the 5 s tail leaves e^-24.5 = 2e-11 of the slowest closed-loop mode, time constant
      0.204 s, and the binary64 Horner sums over 41600 samples round at about N u = 5e-12 relative) and 900 times below
      the smallest amplitude sensitivity U_A measured in Gazebo (0.054 deg = 9.4e-4 rad), so an estimator error of
      TOL_ID changes no verdict.
Negative controls (each must break TOL_ID): d shifted by one execution (a one-sample misalignment; it moves the estimate
by about 1.2e-3 rad), and C with gains x 1.1 (the T3 gain control of decision 0005; about 0.07 rad).
"""

import cmath
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import mixer  # noqa: E402
import rate  # noqa: E402
import run_l4  # noqa: E402
import schema  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
AMPLITUDE = 0.5
DURATION_S = 8.0
TAIL_S = 5.0
T0_US = 0
TOL_ID = 2.0 ** -20
GAIN_CONTROL = 1.1
LOOPS = ("nominal", "J-,tau+")


@pytest.fixture(scope="module")
def design():
    return run_l4._rate_design(str(CARD), str(ROOT))


def band_of(res):
    ws = [m[2] for m in res["margins"]]
    return min(ws) / res["a"], res["a"] * max(ws)


def stamps(period_us, n):
    """floor(i T) microseconds for i < n, the SIL's stamps of the rate-loop executions (T = period_us, 312.5 us)."""
    return [math.floor(i * period_us) for i in range(n)]


def simulate(d, T, j, tau, kp, ki):
    """y_n = w(n T) of the synthetic loop (module docstring), from rest."""
    e = math.exp(-T / tau)
    one_minus_e = -math.expm1(-T / tau)
    w = m = integral = e_prev = 0.0
    y = []
    for n, dn in enumerate(d):
        y.append(w)
        err = -w
        if n:
            integral += ki * e_prev * T
        v = kp * err + integral + dn
        w, m = w + (tau * one_minus_e * m + (T - tau * one_minus_e) * v) / j, e * m + one_minus_e * v
        e_prev = err
    return y


def case(res, loop, axis="roll"):
    """(y, d, T, kp, ki, band, true PM, true crossover) of the synthetic loop `loop` for the axis's float32 gains."""
    T = res["T"]
    name, jt, tau = next(c for c in rate.corner_list(res["inputs"]["tau"], res["inputs"]["b_tau"], res["inputs"]["b_J"])
                         if c[0] == loop)
    ax = res["axes"][axis]
    j = ax["J"] * jt
    band = band_of(res)
    n = math.ceil((DURATION_S + TAIL_S) / T)
    dur_us = round(DURATION_S * run_l4.scn.MICROSECONDS_PER_SECOND)
    d = [run_l4.chirp_value(t, AMPLITUDE, band[0], band[1], T0_US, dur_us) for t in stamps(res["period_us"], n)]
    y = simulate(d, T, j, tau, ax["kp"], ax["ki"])
    theta, pm, why = rate.loop_crossover(ax["kp"] / j, ax["ki"] / j, T, rate.plant_constants(tau, T))
    assert theta is not None, why
    return y, d, T, ax["kp"], ax["ki"], band, pm, theta / T


# ---- the estimator recovers a known margin ---------------------------------------------------------------------------

@pytest.mark.parametrize("loop", LOOPS)
def test_identify_recovers_the_design_margin(design, loop):
    y, d, T, kp, ki, band, pm, wc = case(design, loop)
    r = run_l4.identify(y, d, T, kp, ki, band)
    assert r["pm"] is not None, r["reason"]
    assert abs(r["pm"] - pm) <= TOL_ID, (math.degrees(r["pm"]), math.degrees(pm))
    assert r["crossover"] == pytest.approx(wc, rel=TOL_ID)
    assert band[0] < r["crossover"] < band[1]


def test_the_two_loops_differ_by_far_more_than_the_tolerance(design):
    pms = [case(design, loop)[6] for loop in LOOPS]
    assert abs(pms[0] - pms[1]) > 1000 * TOL_ID


# ---- negative controls ----------------------------------------------------------------------------------------------

def test_control_one_execution_misalignment_breaks_the_tolerance(design):
    y, d, T, kp, ki, band, pm, _ = case(design, "nominal")
    shifted = [0.0] + d[:-1]
    r = run_l4.identify(y, shifted, T, kp, ki, band)
    assert r["pm"] is not None and abs(r["pm"] - pm) > TOL_ID


def test_control_wrong_controller_breaks_the_tolerance(design):
    y, d, T, kp, ki, band, pm, _ = case(design, "nominal")
    r = run_l4.identify(y, d, T, kp * GAIN_CONTROL, ki * GAIN_CONTROL, band)
    assert r["pm"] is None or abs(r["pm"] - pm) > TOL_ID


def test_a_band_without_a_crossover_is_refused(design):
    y, d, T, kp, ki, band, _, wc = case(design, "nominal")
    r = run_l4.identify(y, d, T, kp, ki, (2 * wc, band[1]))
    assert r["pm"] is None and "sign changes" in r["reason"]


# ---- the pieces ------------------------------------------------------------------------------------------------------

def test_dtft_pair_is_the_direct_sum():
    x = [math.sin(0.37 * n) + 0.25 * n for n in range(64)]  # scenario test values: an arbitrary short sequence
    w, T = 3.0, 1e-3
    rx = list(reversed(x))
    a, b = run_l4.dtft_pair(x, rx, w, T)
    # gamma_8N sum |x|: each Horner step is one complex product and one complex sum (at most 8 real roundings), u = 2^-53
    nu = 8 * len(x) * 2.0 ** -53
    bound = nu / (1 - nu) * sum(abs(v) for v in x)
    for got, seq in ((a, x), (b, rx)):
        direct = sum(v * cmath.exp(-1j * w * n * T) for n, v in enumerate(seq))
        assert abs(got - direct) <= bound
    assert abs(a - b) > bound  # control: the two sums are of different sequences


def test_chirp_value_is_the_exponential_sweep_and_zero_outside_its_window():
    amp, w_lo, w_hi, t0, dur = 0.5, 2.0, 32.0, 1000, 4_000_000  # scenario test values: a 16:1 band over 4 s
    assert run_l4.chirp_value(t0 - 1, amp, w_lo, w_hi, t0, dur) == 0.0
    assert run_l4.chirp_value(t0 + dur, amp, w_lo, w_hi, t0, dur) == 0.0
    assert run_l4.chirp_value(t0, amp, w_lo, w_hi, t0, dur) == 0.0  # sin(phi(0)) = 0
    sweep, dur_s = math.log(w_hi / w_lo), dur / 1e6
    for tau_us in (1, 123_457, dur // 2, dur - 1):
        phi = w_lo * dur_s / sweep * (math.exp(sweep * tau_us / dur) - 1)
        assert run_l4.chirp_value(t0 + tau_us, amp, w_lo, w_hi, t0, dur) == pytest.approx(amp * math.sin(phi),
                                                                                          abs=1e-12)
    # the instantaneous frequency d phi / d tau runs from w_lo to w_hi
    def phi(tau_s):
        return w_lo * dur_s / sweep * math.expm1(sweep * tau_s / dur_s)
    h = 1e-6
    assert (phi(h) - phi(0)) / h == pytest.approx(w_lo, rel=1e-4)
    assert (phi(dur_s) - phi(dur_s - h)) / h == pytest.approx(w_hi, rel=1e-4)


def test_tau_held_is_the_held_collective_envelope():
    card = schema.load_yaml(CARD)
    m, _, idle = mixer.mixer_matrix(card, CARD)
    k = card["rotors"]["thrust_coeff"]["value"]
    f_min, f_max = k * idle ** 2, k * card["rotors"]["speed_range"]["value"][1] ** 2
    c_h = (f_min + f_max) / 2 / max(r[0] for r in m)  # every motor at or below mid-range: both limits can bind
    for col in (1, 2, 3):
        t = run_l4.tau_held(m, col, c_h, f_min, f_max)
        want = min(min(f_max - c_h * r[0], c_h * r[0] - f_min) / abs(r[col]) for r in m if r[col])
        assert t == want

        def inside(torque):
            return all(f_min <= c_h * r[0] + s * torque * r[col] <= f_max for r in m for s in (1, -1))
        assert inside(t * (1 - 2.0 ** -40)) and not inside(t * (1 + 2.0 ** -20))
    assert run_l4.tau_held(m, 1, 2 * f_max / min(r[0] for r in m), f_min, f_max) is None  # control: above f_max


def test_max_sensitivity_is_the_largest_value_on_a_finer_grid(design):
    T = design["T"]
    ax = design["axes"]["roll"]
    loops = rate.corner_list(design["inputs"]["tau"], design["inputs"]["b_tau"], design["inputs"]["b_J"])
    s_max, at, loop = run_l4.max_sensitivity(T, ax["kp"] / ax["J"], ax["ki"] / ax["J"], loops)
    n = 4 * run_l4.SENSITIVITY_GRID_POINTS  # a different, finer grid
    brute = max(abs(1 / (1 + run_l4.design_loop(math.pi / T * 2.0 ** (-20 * (1 - i / (n - 1))), T, ax["kp"] / ax["J"],
                                                      ax["ki"] / ax["J"], jt, tau)))
                for _, jt, tau in loops for i in range(n))
    assert brute <= s_max <= brute * (1 + 2.0 ** -20)
    name, jt, tau = next(c for c in loops if c[0] == loop)
    assert abs(1 / (1 + run_l4.design_loop(at, T, ax["kp"] / ax["J"], ax["ki"] / ax["J"], jt, tau))) == s_max


def test_dshot_idle_bound_is_the_esc_map_of_the_idle_speed():
    card = schema.load_yaml(CARD)
    lo, top = card["rotors"]["speed_range"]["value"]
    params = {"idle_speed": float(lo), "rotor_speed_min": float(lo), "rotor_speed_max": float(top)}
    c = run_l4.prim_constants.load()
    assert run_l4.dshot_idle_bound(params) == (c["kDshotThrottleMin"], c["kDshotThrottleMax"])
    raised = dict(params, idle_speed=float(lo) + (top - lo) / 2)  # control: the idle half-way up the map
    assert run_l4.dshot_idle_bound(raised)[0] == math.ceil((c["kDshotThrottleMin"] + c["kDshotThrottleMax"]) / 2)
