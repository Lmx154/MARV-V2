"""Gyro-chain design block (tools/card/gyro_chain_design.py; L6 stage (b), decision 0013): the cutoff rule, the notch Q
rule on the prewarped (digital) notch, the vibration sweep top, the exact discrete loop against rate.py (cross-check 1) and
the Python coefficients against the firmware's float coefficients (cross-check 2: gyro_chain_coeff_tool.cpp compiled here
with the host compiler against fw/gyro_chain, flags as marv_apply_flags(FIRMWARE) on host).

The expected values restate the rules independently: the digital notch / Butterworth gain from the K form
(K = tan(w0/2), x = tan(w/2)/K), not from the module's closed form or its RBJ coefficients. Every acceptance test has a
control that breaks it (core 7.2). Tolerances are derived on their lines.
"""

import cmath
import math
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import gen_imu_config as gic  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import rate  # noqa: E402
import schema  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"
TOOL_SRC = Path(__file__).resolve().parent / "gyro_chain_coeff_tool.cpp"

FS = 6400.0            # the tick rate, 1e6 * den / num of the scenario register (625/4 us); asserted against the register below
A_MIN = 0.1            # gyro_chain_attenuation_min (owner decision 3)
OMEGA_MAX = 2800.0     # rad/s, the card's speed_range[1]
ODR = 65e-6            # the profile's odr_error as a fraction
# Root-find tolerance of the independent Q bisection: both sides are exact to double rounding (1e-15 relative on the gain),
# the gain's sensitivity to Q is order 1 / Q, so 1e-9 relative leaves six decades.
Q_REL_TOL = 1e-9
# The two Q computations agree to that; the edge gain is a_min to 1e-9 absolute for the same reason.
GAIN_TOL = 1e-9
# Cross-check 1: the L4 tests' TOL (test_rate_design.py: 2^-30 rad, bisection to adjacent doubles on both sides).
PM_TOL = 2.0 ** -30
CROSSOVER_REL_TOL = 2.0 ** -30
ESC = (0.0, 0.02, 0.038)
UPPER_EDGE_RECORD = (12.277, 11.116, 9.328)   # decision 0013, "Q definition"; three decimals, so +-5e-4
U = 2.0 ** -24         # float32 unit roundoff
LIBM_ULPS = 2.0        # support.hpp kLibmUlps (INFERRED, checked in coeff_test.cpp)


def k_response(f, f0, q, fs):
    """|H| of the digital notch at f0 evaluated at f, from the K-form coefficients (independent of the module)."""
    k = math.tan(math.pi * f0 / fs)
    norm = 1 / (1 + k / q + k * k)
    b = ((1 + k * k) * norm, 2 * (k * k - 1) * norm, (1 + k * k) * norm)
    a = (1, 2 * (k * k - 1) * norm, (1 - k / q + k * k) * norm)
    zi = cmath.exp(-2j * math.pi * f / fs)
    return abs((b[0] + b[1] * zi + b[2] * zi * zi) / (a[0] + a[1] * zi + a[2] * zi * zi))


def worst_edge_gain(f0, q, eps, fs):
    return max(k_response(f0 * (1 - eps), f0, q, fs), k_response(f0 * (1 + eps), f0, q, fs))


def bisect_q(f0, eps, fs, a):
    """The largest Q with worst_edge_gain <= a, by bisection (the gain rises with Q on the edges)."""
    lo, hi = 1e-3, 1e4
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if worst_edge_gain(f0, mid, eps, fs) <= a:
            lo = mid
        else:
            hi = mid
    return lo


def unwarped_q(f0, f_edge, a):
    x = f_edge / f0
    return a * x / (math.sqrt(1 - a * a) * abs(1 - x * x))


# ---- the cutoff rule -------------------------------------------------------------------------------------------------


def test_the_tick_rate_constant_is_the_register_tick():
    sc = schema.load_yaml(SCENARIO)
    assert 1e6 * sc["tick_period_den"]["value"] / sc["tick_period_num_us"]["value"] == FS


def test_cutoff_is_625_hz_and_gives_a_min_at_the_rate_loop_nyquist():
    fc = gcd.lowpass_cutoff_hz(FS, 2, A_MIN)
    assert fc == pytest.approx(625.4, abs=0.05)   # the lead's figure, to the 0.1 Hz it is quoted to (0013: "625 Hz")
    k = math.tan(math.pi * fc / FS)
    x = math.tan(math.pi * (FS / 2 / 2) / FS) / k   # f_r/2 = f_s/4 for D = 2
    assert 1 / math.sqrt(1 + x ** 4) == pytest.approx(A_MIN, abs=1e-12)   # Butterworth |H|, double rounding of tan only


def test_cutoff_refuses_a_divisor_below_two():
    for d in (1, 0):
        with pytest.raises(gcd.DesignError):
            gcd.lowpass_cutoff_hz(FS, d, A_MIN)
    assert gcd.lowpass_cutoff_hz(FS, 3, A_MIN) < gcd.lowpass_cutoff_hz(FS, 2, A_MIN)


def test_cutoff_control_a_min_times_1p1_breaks_the_nyquist_gain():
    fc = gcd.lowpass_cutoff_hz(FS, 2, A_MIN * 1.1)
    k = math.tan(math.pi * fc / FS)
    gain = 1 / math.sqrt(1 + (math.tan(math.pi / 4) / k) ** 4)
    assert abs(fc - gcd.lowpass_cutoff_hz(FS, 2, A_MIN)) > 1.0       # more than a hertz: a_min x 1.1 is not the same design
    assert abs(gain - A_MIN) > 1e-3                                    # and fails the a_min check at 1e-12 by nine decades


# ---- the notch Q rule ------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("esc", ESC)
@pytest.mark.parametrize("h", gcd.HARMONICS)
def test_q_attenuates_to_a_min_at_both_edges_and_is_the_largest(h, esc):
    eps = gcd.epsilon(ODR, esc)
    f0 = h * OMEGA_MAX / (2 * math.pi)
    assert f0 * (1 + eps) < FS / 2
    q = gcd.notch_q(f0, eps, A_MIN, FS)
    lower, upper = k_response(f0 * (1 - eps), f0, q, FS), k_response(f0 * (1 + eps), f0, q, FS)
    assert max(lower, upper) == pytest.approx(A_MIN, abs=GAIN_TOL)
    assert q == pytest.approx(bisect_q(f0, eps, FS, A_MIN), rel=Q_REL_TOL)    # independent root find
    # The control: Q x 1.1 violates the bound at the binding edge, by far more than the tolerance.
    assert worst_edge_gain(f0, 1.1 * q, eps, FS) > A_MIN * 1.05


def test_q_set_and_epsilon():
    assert gcd.epsilon(ODR, 0.02) == 2.0 ** -8 + ODR + 0.02
    eps = gcd.epsilon(ODR, 0.0)
    qs = gcd.notch_q_set(OMEGA_MAX, eps, A_MIN, FS)
    assert [gcd.notch_q(h * OMEGA_MAX / (2 * math.pi), eps, A_MIN, FS) for h in (1, 2, 3)] == list(qs)
    assert qs[0] > qs[1] > qs[2]


def test_the_lower_edge_binds_and_the_recorded_values_are_the_upper_edge():
    """Decision 0013 records Q = 12.277 / 11.116 / 9.328 at ESC error 0. They are the upper edge f0 (1 + eps); the digital
    notch is narrower below f0, so the lower edge needs a smaller Q and the rule's value is the smaller (a finding for the
    lead; the firmware test's scratch Q = 12.2 / 11.1 / 9.3 sits at or below the lower-edge value except h = 2)."""
    eps = gcd.epsilon(ODR, 0.0)
    for h, rec in zip(gcd.HARMONICS, UPPER_EDGE_RECORD):
        f0 = h * OMEGA_MAX / (2 * math.pi)
        assert gcd._edge_q(f0, f0 * (1 + eps), FS, A_MIN) == pytest.approx(rec, abs=5e-4)
        assert gcd.notch_q(f0, eps, A_MIN, FS) == gcd._edge_q(f0, f0 * (1 - eps), FS, A_MIN) < rec


def test_the_rule_uses_the_digital_notch_not_the_unwarped_form():
    eps = gcd.epsilon(ODR, 0.0)
    over = {}
    for h in gcd.HARMONICS:
        f0 = h * OMEGA_MAX / (2 * math.pi)
        q = gcd.notch_q(f0, eps, A_MIN, FS)
        qu = min(unwarped_q(f0, f0 * (1 - eps), A_MIN), unwarped_q(f0, f0 * (1 + eps), A_MIN))
        over[h] = qu / q - 1
        if h > 1:
            assert q != pytest.approx(qu, rel=1e-2)
    # the unwarped Q violates the digital bound by a wide margin at h = 2 and 3 (decision 0013: about 14 % and 36 % over)
    assert over[2] > 0.10 and over[3] > 0.30
    f0 = 3 * OMEGA_MAX / (2 * math.pi)
    qu = min(unwarped_q(f0, f0 * (1 - eps), A_MIN), unwarped_q(f0, f0 * (1 + eps), A_MIN))
    assert worst_edge_gain(f0, qu, eps, FS) > 1.2 * A_MIN


def test_q_refuses_an_edge_at_or_above_nyquist():
    with pytest.raises(gcd.DesignError):
        gcd.notch_q(FS / 2 / 1.02, 0.03, A_MIN, FS)


def test_threshold():
    assert gcd.omega_threshold(1100.0, A_MIN) == pytest.approx(1100.0 * math.sqrt(0.1), rel=1e-15)
    assert gcd.omega_threshold(1100.0, 0.4) != gcd.omega_threshold(1100.0, 0.1)


# ---- the vibration sweep top -----------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def committed():
    card, budget, scenario = (schema.load_yaml(p) for p in (CARD, BUDGET, SCENARIO))
    res = rate.design(card, budget, scenario, CARD)
    si, _, _ = gic.imu_config(PROFILE)
    return {"res": res, "card": card, "scenario": scenario, "si": si}


def test_sweep_top_does_not_saturate_and_a_larger_one_does(committed):
    si, card, scenario = committed["si"], committed["card"], committed["scenario"]
    fs_gyro = si["gyro_full_scale"]
    rate_max = scenario["rate_max_roll"]["value"]
    hover = math.sqrt(card["mass"]["value"] * gic.G / (4 * card["rotors"]["thrust_coeff"]["value"]))
    p = 2
    a_max = gcd.vibration_amplitude_max(fs_gyro, rate_max, OMEGA_MAX, hover, p)
    assert a_max == pytest.approx(0.75, abs=0.01)          # decision 0013: the architect's scratch value, about 0.75 rad/s
    assert gcd.peak_gyro(a_max, OMEGA_MAX, hover, rate_max, p) <= fs_gyro * (1 + 4 * 2.0 ** -52)
    step = 2.0 ** -10                                       # labelled: any positive step violates the bound; this one is far above rounding
    assert gcd.peak_gyro(a_max * (1 + step), OMEGA_MAX, hover, rate_max, p) > fs_gyro
    # the worst case is at omega_max, not hover: the hover-only amplitude would saturate with the p = 2 scaling
    assert gcd.peak_gyro(fs_gyro / 12, OMEGA_MAX, hover, 0.0, p) > fs_gyro
    with pytest.raises(gcd.DesignError):
        gcd.vibration_amplitude_max(fs_gyro, fs_gyro, OMEGA_MAX, hover, p)


def test_sweep_halves_to_the_first_value_below_sigma_d(committed):
    si = committed["si"]
    rep, _ = gic.derived_report(si, 625, 4)
    sigma_d = rep["gyro"][4]
    sw = gcd.vibration_sweep(0.75, sigma_d)
    assert sw[0] == 0.0 and sw[1] == 0.75
    assert all(sw[i + 1] == sw[i] / 2 for i in range(1, len(sw) - 1))
    assert sw[-1] < sigma_d <= sw[-2]
    assert gcd.vibration_sweep(sigma_d / 2, sigma_d) == [0.0, sigma_d / 2]


# ---- cross-check 1: the loop model against rate.py -------------------------------------------------------------------


def test_loop_model_with_no_chain_reproduces_rate_py_at_every_corner(committed):
    res = committed["res"]
    inp = res["inputs"]
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    t_s = res["T"] / committed["scenario"]["rate_loop_divisor"]["value"]
    mine = gcd.margins(res["kappa_p"], res["kappa_i"], t_s, 2, corners)
    assert len(mine) == len(res["margins"]) == 5
    for (name, pm, wx, n), (rname, rpm, rwx, _) in zip(mine, res["margins"]):
        assert name == rname and n == 1
        assert abs(pm - rpm) <= PM_TOL, (name, pm - rpm)
        assert abs(wx / rwx - 1) <= CROSSOVER_REL_TOL, (name, wx / rwx - 1)


def test_cross_check_1_control_one_sample_of_latency_breaks_it(committed):
    res = committed["res"]
    inp = res["inputs"]
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    t_s = res["T"] / 2
    mine = gcd.margins(res["kappa_p"], res["kappa_i"], t_s, 2, corners, latency=1)
    for (_, pm, _, _), (_, rpm, _, _) in zip(mine, res["margins"]):
        assert abs(pm - rpm) > 1000 * PM_TOL


def test_chain_phase_drop_is_the_group_delay_at_crossover(committed):
    """Sanity of the aliasing sum, hold and latency with a real chain: the margin falls by omega_c (group delay + latency)
    to first order. The residual is the curvature of the chain's phase and the shift of the crossover; 1e-4 rad is a
    labelled bound, one decade above the measured residual (1e-5 rad) and two below the 0.02 rad the chain costs."""
    res, card, scenario, si = committed["res"], committed["card"], committed["scenario"], committed["si"]
    d = gcd.chain_design(res, card, scenario, si, A_MIN, 0.0)
    st = gcd.chain_stages(d["t_s"], d["f_c"], d["q"], d["omega_th"], [d["omega_th"]] * 4)
    corners = rate.corner_list(res["inputs"]["tau"], res["inputs"]["b_tau"], res["inputs"]["b_J"])[:1]
    base = gcd.margins(res["kappa_p"], res["kappa_i"], d["t_s"], d["divisor"], corners)[0]
    with_chain = gcd.margins(res["kappa_p"], res["kappa_i"], d["t_s"], d["divisor"], corners,
                             lambda th: gcd.chain_response(st, th), si["latency_samples"])[0]
    tau_total = gcd.group_delay_s(st, base[2], d["t_s"]) + si["latency_samples"] * d["t_s"]
    drop = base[1] - with_chain[1]
    assert drop == pytest.approx(base[2] * tau_total, abs=1e-4)
    assert drop > 0.01                                      # the control: a model that ignores the chain gives drop 0


# ---- cross-check 2: the Python double coefficients against the firmware float coefficients -----------------------------


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


class Err:
    """The running error analysis of tests/regression/quad/L06/gyro_chain/support.hpp (Err, notch_err, lowpass_err), ported."""

    def __init__(self, v, e=0.0):
        self.v, self.e = v, e

    def __add__(s, o):
        v = s.v + o.v
        return Err(v, s.e + o.e + U * abs(v))

    def __sub__(s, o):
        v = s.v - o.v
        return Err(v, s.e + o.e + U * abs(v))

    def __mul__(s, o):
        v = s.v * o.v
        return Err(v, abs(s.v) * o.e + abs(o.v) * s.e + s.e * o.e + U * abs(v))

    def __truediv__(s, o):
        v = s.v / o.v
        return Err(v, (s.e + abs(v) * o.e) / (abs(o.v) - o.e) + U * abs(v))


def rounded(v):
    return Err(v, U * abs(v))


def scale2(a):
    return Err(2 * a.v, 2 * a.e)


def sin_e(a):
    v = math.sin(a.v)
    return Err(v, abs(math.cos(a.v)) * a.e + LIBM_ULPS * 2 * U * abs(v))


def cos_e(a):
    v = math.cos(a.v)
    return Err(v, abs(math.sin(a.v)) * a.e + LIBM_ULPS * 2 * U * abs(v))


def tan_e(a):
    v = math.tan(a.v)
    return Err(v, (1 + v * v) * a.e + LIBM_ULPS * 2 * U * abs(v))


def notch_bound(f0, q, period):
    two_pi = scale2(rounded(math.pi))
    w0 = two_pi * Err(f0) * Err(period)
    alpha = sin_e(w0) / scale2(Err(q))
    cw = cos_e(w0)
    norm = Err(1.0) / (Err(1.0) + alpha)
    t = scale2(cw) * norm
    b1 = Err(-t.v, t.e)   # the negation is exact
    a2 = (Err(1.0) - alpha) * norm
    return [norm, b1, norm, b1, a2]


def lowpass_bound(fc, period):
    k = tan_e(rounded(math.pi) * Err(fc) * Err(period))
    k2 = k * k
    rk = rounded(math.sqrt(2.0)) * k
    norm = Err(1.0) / ((Err(1.0) + rk) + k2)
    b0 = k2 * norm
    return [b0, scale2(b0), b0, scale2(k2 - Err(1.0)) * norm, ((Err(1.0) - rk) + k2) * norm]


@pytest.fixture(scope="module")
def firmware_coeffs(tmp_path_factory, committed):
    cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if cxx is None:
        pytest.fail("no C++ compiler on this host: the firmware cross-check cannot run (it is a required check)")
    exe = tmp_path_factory.mktemp("coeff_tool") / "gyro_chain_coeff_tool"
    inc = [f"-I{ROOT / 'fw' / d / 'include'}" for d in ("gyro_chain", "prim", "types")]
    build = subprocess.run([cxx, "-std=c++20", "-ffp-contract=off", "-fno-exceptions", "-fno-rtti", "-Wall", "-Wextra",
                            "-Werror", *inc, str(TOOL_SRC), str(ROOT / "fw" / "gyro_chain" / "src" / "gyro_chain.cpp"),
                            "-o", str(exe)], capture_output=True, text=True, check=False)
    assert build.returncode == 0, build.stderr
    res, card, scenario, si = committed["res"], committed["card"], committed["scenario"], committed["si"]
    d = gcd.chain_design(res, card, scenario, si, A_MIN, 0.0)
    period = f32(d["t_s"])
    cases = [("lowpass", f32(d["f_c"]), None, period)]
    for w in (d["omega_th"], d["omega_max"] / 2, d["omega_max"]):
        for h, qh in zip(gcd.HARMONICS, d["q"]):
            cases.append(("notch", f32(h * w / (2 * math.pi)), f32(qh), period))
    # test values at the ends of the validated range (as coeff_test.cpp): a low and a high Q, one near Nyquist
    cases += [("notch", 400.0, 0.5, period), ("notch", 400.0, 100.0, period), ("notch", f32(0.9 * FS / 2), 9.3, period)]
    stdin = "".join(f"{k} {float(a).hex()} " + (f"{float(q).hex()} " if q is not None else "") + f"{float(p).hex()}\n"
                    for k, a, q, p in cases)
    run = subprocess.run([str(exe)], input=stdin, capture_output=True, text=True, check=False)
    assert run.returncode == 0
    rows = [[float.fromhex(x) for x in line.split()] for line in run.stdout.splitlines()]
    assert len(rows) == len(cases)
    return cases, rows


def python_and_bound(case):
    kind, a, q, period = case
    if kind == "lowpass":
        return gcd.lowpass_coeffs(a, period), lowpass_bound(a, period)
    return gcd.notch_coeffs(a, q, period), notch_bound(a, q, period)


def test_firmware_float_coefficients_match_the_python_double_coefficients(firmware_coeffs):
    cases, rows = firmware_coeffs
    for case, fw in zip(cases, rows):
        py, bound = python_and_bound(case)
        for i, name in enumerate(("b0", "b1", "b2", "a1", "a2")):
            # the bound's own reference is the same formula in double, so the Python value must sit within the bound's
            # reference tolerance (1e-12, the K-form / RBJ-form double difference of coeff_test.cpp kFormTol) of it
            assert abs(py[i] - bound[i].v) <= 1e-12, (case, name)
            assert abs(fw[i] - py[i]) <= bound[i].e + 1e-12, (case, name, fw[i] - py[i], bound[i].e)
        # the bound is not vacuous (support.hpp: far below 1e-5)
        assert max(b.e for b in bound) < 1e-5


def test_cross_check_2_control_a_coefficient_beyond_the_bound_fails(firmware_coeffs):
    cases, rows = firmware_coeffs
    case, fw = cases[1], rows[1]
    py, bound = python_and_bound(case)
    bad = fw[3] + 4 * bound[3].e
    assert abs(bad - py[3]) > bound[3].e + 1e-12
    # and a Python formula with the wrong Q (x 1.1) is seen by the same comparison
    kind, f0, q, period = case
    wrong = gcd.notch_coeffs(f0, 1.1 * q, period)
    assert any(abs(fw[i] - wrong[i]) > bound[i].e + 1e-12 for i in range(5))


# ---- the report ------------------------------------------------------------------------------------------------------


def test_report_lists_every_esc_case_with_one_worst_corner_each(committed):
    lines = gcd.report_lines(committed["res"], committed["card"], committed["scenario"], committed["si"])
    text = "\n".join(lines)
    assert text.count("ESC clock error") == 3
    assert text.count("loop with the chain") == 6                      # worst-case operating point and all-bypassed, per ESC case
    assert sum(1 for line in lines if line.lstrip().startswith("*")) == 7  # one marked worst corner per table, base table included
    assert "worst case: all four notches at omega_th" in text and "all notches bypassed" in text
