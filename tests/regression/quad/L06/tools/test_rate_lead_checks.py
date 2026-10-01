"""Checks of tools/card/rate_lead.py's model pieces that do not need the full N* search (L6 stage (c), decision 0014): the
2-periodic firmware loop reduces to the LTI loop at equal dt; the tool's controller is the firmware law (rate_law_tool.cpp
drives fw/rate's RateLoop<float>); the coherent D + FF noise formula against a brute-force simulation.

The gains are the rule's own at its output: Model.gains(OMEGA_C, N_STAR) on the committed card, with N_STAR, OMEGA_C and
T_FF the values design() returns (test_rate_lead.py asserts they are, so a rule change that moves them fails there). Every
acceptance test has a control that breaks it (core 7.2); tolerances are derived on their lines.
"""

import cmath
import math
import random
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import gen_imu_config as gic  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import gyro_chain_params as gcp  # noqa: E402
import mixer  # noqa: E402
import rate  # noqa: E402
import rate_lead as rl  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"
TOOL_SRC = Path(__file__).resolve().parent / "rate_law_tool.cpp"

# The rule's output on the committed card (rate_lead.py --report; test_rate_lead.py checks design() returns exactly these).
N_STAR = 3.877369229832578
OMEGA_C = 12.129191367206076      # rad/s, after the f32 guard's step-down
T_FF = 0.012311602011322975       # s

EPS = 2.0 ** -52                  # double unit roundoff
U32 = 2.0 ** -24                  # float32 unit roundoff
CONTROL = 1.1                     # the controls' factor on kd or T_f


@pytest.fixture(scope="module")
def setup():
    card, budget, scenario, profile = rl.load(CARD, BUDGET, SCENARIO, PROFILE)
    l4 = rate.design(card, budget, scenario, CARD)
    inp = l4["inputs"]
    chain = gcp.design(card, budget, scenario, profile, PROFILE, CARD)
    si, _, _ = gic.imu_config(PROFILE)
    stages = gcd.chain_stages(chain["t_s"], chain["f_c"], chain["q"], chain["omega_th"], [chain["omega_th"]] * gcd.MOTORS)
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    model = rl.Model(chain["t_s"], chain["divisor"], si["latency_samples"], stages, corners, l4["a"], inp["inertia"])
    gains = model.gains(OMEGA_C, N_STAR)
    num, den = scenario["tick_period_num_us"]["value"], scenario["tick_period_den"]["value"]
    m_matrix, _, _ = mixer.mixer_matrix(card, CARD)
    sigma_d = gic.derived_report(si, num, den)[0]["gyro"][4]
    noise = rl.Noise(chain, m_matrix, inp["inertia"], inp["tau"], sigma_d)
    rate_max = [scenario[f"rate_max_{a}"]["value"] for a in rl.AXES]
    return {"model": model, "gains": gains, "axes32": rl.f32_axes(model, gains), "dts": rl.execution_dts(num, den, chain["divisor"]),
            "chain": chain, "noise": noise, "m": m_matrix, "inertia": inp["inertia"], "tau": inp["tau"], "sigma_d": sigma_d,
            "rate_max": rate_max, "stamps": (num, den, chain["divisor"])}


# ---- 1. equal dt: the 2-periodic loop is the LTI loop ---------------------------------------------------------------

# Tolerance: the periodic closed forms and the LTI forms are the same rational functions, evaluated in a different order.
# Their double rounding is amplified by the cancellations 1 - 1/z and z - 1 (relative error eps/theta) and by the
# near-cancelling denominators 1 - p/z, 1 - p0 p1/z^2 and 1 - 1/z^2 (|1 - p/z| >= alpha, so a factor 1/alpha). With a
# few tens of operations: tol(theta) = 64 eps (1 + 1/theta + 1/alpha). Measured residuals are below a tenth of it.
TOL_FACTOR = 64
GRID_STRIDE = 8                   # every 8th point of gyro_chain_design's 4096-point grid


def _tol(theta, alpha):
    return TOL_FACTOR * EPS * (1 + 1 / theta + 1 / alpha)


def test_equal_dt_reduces_the_periodic_loop_to_the_lti_loop(setup):
    model, g = setup["model"], setup["gains"]
    t = model.t
    alpha, beta = rl.lowpass_pole(t, g[3])
    same = [(t, alpha), (t, alpha)]
    alternating = rl.firmware_phases(setup["dts"], g[3], 0)
    grid = gcd._THETA[::GRID_STRIDE]
    worst_alt = 0.0
    for theta in grid:
        z = cmath.exp(1j * theta)
        tol = _tol(theta, alpha)
        for j in rl.periodic_integral(z, [t, t]):
            assert abs(j / (t / (z - 1)) - 1) <= tol, theta
        for x in rl.periodic_derivative(z, same):
            assert abs(x / rl.derivative(z, t, alpha, beta) - 1) <= tol, theta
        for _, jt, tau in model.corners:
            lti = model.loop(g, jt, tau)(theta)
            assert abs(model.loop(g, jt, tau, same)(theta) / lti - 1) <= tol, theta
            worst_alt = max(worst_alt, abs(model.loop(g, jt, tau, alternating)(theta) / lti - 1) / tol)
    # Control: the firmware's alternating dt (312/313 us, f32 alpha) is not the LTI loop.
    assert worst_alt > 1


# ---- 2. the tool's controller is the firmware law -------------------------------------------------------------------

EXECUTIONS = 400                  # rate executions after the seed: three time constants of the D low-pass (T_f/T = 134)
TAU_REF_UNUSED = 1.0              # s, execute_bypass never reads tau_ref; validate needs it > 0
CONTOUR_POINTS = 2048             # inverse z-transform on |z| = r: r^-N = e^-40, so the aliased tail is below 1e-17
CONTOUR_LOG_R = 40.0
# The reference's own error: the contour sum amplifies double rounding by r^n <= e^(40 * 400 / 2048) = 2.5e3 over N terms;
# it is checked against a double recursion of the same law below at 2^-30 of the largest output (measured about 1e-12).
REFERENCE_TOL = 2.0 ** -30
SECOND_ORDER = 2                  # factor on the first-order running error bound (second-order terms, computed magnitudes)


@pytest.fixture(scope="module")
def law_tool(tmp_path_factory):
    cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if cxx is None:
        pytest.fail("no C++ compiler on this host: the firmware-law check cannot run (it is a required check)")
    exe = tmp_path_factory.mktemp("rate_law_tool") / "rate_law_tool"
    inc = [f"-I{ROOT / 'fw' / d / 'include'}" for d in ("rate", "prim", "types", "mixer")]
    build = subprocess.run([cxx, "-std=c++20", "-ffp-contract=off", "-fno-exceptions", "-fno-rtti", "-Wall", "-Wextra",
                            "-Werror", *inc, str(TOOL_SRC), "-o", str(exe)], capture_output=True, text=True, check=False)
    assert build.returncode == 0, build.stderr
    return exe


def _run_law(exe, axis32, period, stamps, ys):
    kp, ki, kd, tf = axis32
    lines = [" ".join(float(v).hex() for v in (kp, ki, kd, tf, TAU_REF_UNUSED, period))]
    lines += [f"{t} {float(y).hex()}" for t, y in zip(stamps, ys)]
    run = subprocess.run([str(exe)], input="\n".join(lines) + "\n", capture_output=True, text=True, check=False)
    assert run.returncode == 0, run.stderr
    return [[float.fromhex(v) for v in line.split()] for line in run.stdout.splitlines()]


def _reference(kp, ki, kd, phases, step):
    """-u of the tool's 2-periodic controller (rl.periodic_integral, rl.periodic_derivative; Y_p = kp + ki J_p + kd X_p) for
    an impulse (step False) or a unit step in y at n = 0, by the inverse z-transform on |z| = r of Y_(n mod 2)(z) X(z):
    for the input sum_z X(z) z^n the output is sum_z Y_(n mod 2)(z) X(z) z^n (the steady-state response per z)."""
    n_pts, r = CONTOUR_POINTS, math.exp(CONTOUR_LOG_R / CONTOUR_POINTS)
    w = [cmath.exp(2j * math.pi * k / n_pts) for k in range(n_pts)]
    f = [[], []]
    for k in range(n_pts):
        z = r * w[k]
        js, xs = rl.periodic_integral(z, [p[0] for p in phases]), rl.periodic_derivative(z, phases)
        x_in = z / (z - 1) if step else 1
        for p in range(2):
            f[p].append((kp + ki * js[p] + kd * xs[p]) * x_in)
    out = []
    for n in range(EXECUTIONS):
        fp = f[n % 2]
        out.append((r ** n * sum(fp[k] * w[(k * n) % n_pts] for k in range(n_pts)) / n_pts).real)
    return out


def _law_with_bound(kp, ki, kd, phases, ys):
    """The firmware's expressions (rate_loop.hpp step()) in double, with the first-order running error bound of their
    float evaluation (each operation adds U32 |result|; products by the exact inputs 0 and +-1 add nothing)."""
    i_ = df = e_prev = y_prev = 0.0
    ei = edf = 0.0
    out = []
    for n, y in enumerate(ys):
        dt, al = phases[n % 2]
        e = -y
        t1 = ki * e_prev
        t2 = t1 * dt
        i_ += t2
        ei += U32 * (abs(t1) * dt + abs(t2) + abs(i_))
        d = -kd * (y - y_prev) / dt
        s1 = d - df
        es1 = U32 * abs(d) + edf + U32 * abs(s1)
        s2 = al * s1
        df += s2
        edf += al * es1 + U32 * abs(s2) + U32 * abs(df)
        p2 = kp * e + i_
        u = p2 + df
        out.append((-u, U32 * (abs(kp * e) + abs(p2) + abs(u)) + ei + edf))
        e_prev, y_prev = e, y
    return out


@pytest.mark.parametrize("step", (False, True), ids=("impulse", "step"))
def test_tool_controller_is_the_firmware_law(setup, law_tool, step):
    model, axis32 = setup["model"], setup["axes32"][0]
    num, den, divisor = setup["stamps"]
    stamps = [rl.stamp_us(num, den, k * divisor) for k in range(EXECUTIONS + 1)]
    ys = [0.0] + [1.0 if (step or k == 0) else 0.0 for k in range(EXECUTIONS)]   # execution 0 seeds the law
    fw = _run_law(law_tool, axis32, model.t, stamps, ys)
    assert fw[0][0] == 0.0                                                       # the seed outputs nothing
    phases = rl.firmware_phases(setup["dts"], axis32[3], 0)
    # The law's alpha is the tool's f32 alpha at each execution's dt (host libm; the target is the guard's +-2 ulp).
    assert [a for _, a in fw[1:3]] == [p[1] for p in phases]
    assert [stamps[k + 1] - stamps[k] for k in range(2)] == setup["dts"]
    kp, ki, kd, _ = axis32
    ref = _reference(kp, ki, kd, phases, step)
    law = _law_with_bound(kp, ki, kd, phases, ys[1:])
    scale = max(abs(v) for v in ref)
    assert max(abs(a - b[0]) for a, b in zip(ref, law)) <= REFERENCE_TOL * scale     # the reference is the law in double
    u_fw = [-row[0] for row in fw[1:]]
    for n, (a, (_, bound)) in enumerate(zip(ref, law)):
        assert abs(u_fw[n] - a) <= SECOND_ORDER * bound + REFERENCE_TOL * scale, (n, u_fw[n] - a, bound)
    # Controls: kd x 1.1, and T_f x 1.1 (alpha from the f32 T_f x 1.1), in the tool's controller break the agreement.
    for bad in (_reference(kp, ki, CONTROL * kd, phases, step),
                _reference(kp, ki, kd, rl.firmware_phases(setup["dts"], CONTROL * axis32[3], 0), step)):
        assert any(abs(u_fw[n] - a) > SECOND_ORDER * law[n][1] + REFERENCE_TOL * scale for n, a in enumerate(bad))


# ---- 3. the coherent D + FF noise formula against brute force -------------------------------------------------------

SEED = 1                          # labelled: any seed; the test is deterministic for a given one
SIM_EXECUTIONS = 120000           # rate executions simulated
BURN_IN_TAU = 8                   # executions discarded: 8 time constants of the slowest filter (start-up below e^-8)
K_SIGMA = 4                       # the statistical bound in standard deviations (two-sided normal tail 6e-5)


def test_coherent_noise_formula_matches_a_brute_force_simulation(setup):
    noise, m_matrix, inertia, tau_m = setup["noise"], setup["m"], setup["inertia"], setup["tau"]
    axes32 = setup["axes32"]
    kd32, tf32 = [a[2] for a in axes32], axes32[0][3]
    w0 = list(setup["rate_max"])                          # rate_max on every axis, sign pattern (+, +, +)
    dts = setup["dts"]
    fw = (dts, 0)
    formula = noise.rms(None, w0, kd32, tf32, T_FF, fw=fw)
    ph_d, ph_f = rl.firmware_phases(dts, tf32, 0), rl.firmware_phases(dts, T_FF, 0)

    # Brute force: white gyro noise per tick and axis, the notch-bypassed chain (only its low-pass acts), decimation by D,
    # then the firmware-equivalent D path and the nonlinear feed-forward g = y x (J y) + tau_m gdot, through the mixer.
    c = setup["chain"]
    stages = [st for st in gcd.chain_stages(c["t_s"], c["f_c"], c["q"], c["omega_th"], [c["omega_th"]] * gcd.MOTORS, True)
              if st != gcd.IDENTITY]
    rnd = random.Random(SEED)
    sig = setup["sigma_d"]
    state = [[[0.0, 0.0] for _ in stages] for _ in range(3)]
    y_prev, df, gd = list(w0), [0.0] * 3, [0.0] * 3
    g_prev = rl.cross(w0, [j * w for j, w in zip(inertia, w0)])
    burn = BURN_IN_TAU * math.ceil(tf32 / noise.t)
    acc = [[0.0, 0.0] for _ in m_matrix]
    count = 0
    for n in range(SIM_EXECUTIONS):
        for _ in range(c["divisor"]):
            ys = []
            for a in range(3):
                v = rnd.gauss(0.0, sig)
                for (b0, b1, b2, a1, a2), s in zip(stages, state[a]):
                    x = v
                    v = b0 * x + s[0]
                    s[0] = b1 * x - a1 * v + s[1]
                    s[1] = b2 * x - a2 * v
                ys.append(w0[a] + v)
        (dt, al), (_, alf) = ph_d[n % 2], ph_f[n % 2]
        g = rl.cross(ys, [j * w for j, w in zip(inertia, ys)])
        torque = []
        for a in range(3):
            df[a] += al * (-kd32[a] * (ys[a] - y_prev[a]) / dt - df[a])
            gd[a] += alf * ((g[a] - g_prev[a]) / dt - gd[a])
            torque.append(df[a] + g[a] + tau_m * gd[a])
        y_prev, g_prev = ys, g
        if n >= burn:
            count += 1
            for i, row in enumerate(m_matrix):
                f = sum(row[1 + b] * torque[b] for b in range(3))
                acc[i][0] += f
                acc[i][1] += f * f
    sim = [math.sqrt(s2 / count - (s1 / count) ** 2) for s1, s2 in acc]

    # The statistical bound. For a Gaussian stationary output with PSD S and autocovariance r_k, the sample variance of n
    # samples has variance (2/n) sum_k r_k^2 = (2/n) (1/pi) int_0^pi S^2 (Parseval), so its relative standard deviation is
    # sqrt(2 mean(S^2)/n)/mean(S) and the RMS's is half that. S per motor from the formula's own pieces (phase 0; the two
    # phases differ by far less than the bound).
    g0 = rl.gyroscopic_jacobian(w0, inertia)
    a_th = noise.aliased_power(None)
    for i, row in enumerate(m_matrix):
        mi = [row[1 + a] for a in range(3)]
        gi = [sum(row[1 + b] * g0[b][a] for b in range(3)) for a in range(3)]
        s1 = s2 = 0.0
        for z, a in zip(noise.z, a_th):
            xd, xf = rl.periodic_derivative(z, ph_d)[0], rl.periodic_derivative(z, ph_f)[0]
            s = sig * sig * a * sum(abs(-mi[k] * kd32[k] * xd + gi[k] * (1 + tau_m * xf)) ** 2 for k in range(3))
            s1 += s
            s2 += s * s
        rel = math.sqrt(2 * (s2 / noise.points) / count) / (s1 / noise.points) / 2
        assert abs(sim[i] / formula["combined"][i] - 1) <= K_SIGMA * rel, (i, sim[i], formula["combined"][i], rel)
        if i == max(range(len(sim)), key=lambda k: formula["combined"][k]):
            # Control: the incoherent sum (no cross term) misses the simulation by more than the bound at the worst motor.
            incoherent = math.hypot(formula["d"][i], formula["ff"][i])
            assert abs(sim[i] / incoherent - 1) > K_SIGMA * rel, (i, sim[i], incoherent, rel)


# ---- 4. the N search's result does not depend on the process count ----------------------------------------------------

PROCS = (1, 2, 3, 4, 7, 8)        # serial, one level of speculation (2), two (3, 4), three (7, 8)


def _fake_evaluate(n):
    """A cheap stand-in for evaluate_n with the same shape of answer: Ms rises in N and crosses 2 between grid points; the
    noise floor rises too. Module level, so the workers receive it by name."""
    return {"n": n, "ms": 1 + 0.3 * math.log2(n), "floor": 1e-3 * n}


def _fake_feasible(r):
    return r["ms"] <= 2.0 and r["floor"] <= 1.0


def test_n_search_gives_the_serial_result_for_any_process_count():
    def summary(res):
        grid, k_first, lo, hi, history, bisections = res
        return [r["n"] for r in grid], k_first, lo["n"], hi["n"], [r["n"] for r in history], bisections

    serial = summary(rl.n_search(_fake_evaluate, _fake_feasible, 1, CARD))
    assert serial[5] > 3          # the bisection runs several levels, so the speculative tree is exercised
    for procs in PROCS[1:]:
        assert summary(rl.n_search(_fake_evaluate, _fake_feasible, procs, CARD)) == serial, procs
