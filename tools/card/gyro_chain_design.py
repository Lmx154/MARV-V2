#!/usr/bin/env python3
"""Gyro-chain design rules, the chain's frequency response and the exact discrete rate loop with the chain in it (quad
spec L6 stage (b), decisions 0013 "the design" and "the build", 0012 D; owner decisions 3, 4, 5 and 6 of 0013).

Plain Python double (math, cmath), like rate.py. Nothing here changes the L4 / L5 gain designs: the module only reads them
(rate.design) and evaluates the loop they give with the chain added. Stage (c) owns the redesign.

Rules (pure functions; every input is an argument, no default is a design number)

  lowpass_cutoff_hz   tan(pi f_c / f_s) = tan(pi f_r / (2 f_s)) / (1/a_min^2 - 1)^(1/4), f_r = f_s / D: the digital gain of
                      the second-order Butterworth at the rate loop's Nyquist f_r/2 is a_min. Refuses D < 2 (the rate-loop
                      Nyquist would be the tick's own).
  notch_q             the largest Q whose digital notch attenuates to at most a_min at f0 (1 +- eps), at the worst-case f0 =
                      h omega_max / (2 pi) (the bilinear transform narrows the notch most there). The notch is exactly
                      the firmware's: the prewarped bilinear transform at f0 of (s^2 + 1)/(s^2 + s/Q + 1) (the Audio EQ
                      Cookbook notch, alpha = sin(w0)/(2 Q); gyro_chain.hpp:25-33). With K = tan(w0/2) and x = tan(w/2)/K
                      the response is |H| = |1 - x^2| / sqrt((1 - x^2)^2 + x^2/Q^2), so |H| <= a at x gives, exactly,
                          Q <= a x / (sqrt(1 - a^2) |1 - x^2|),
                      and Q is the smaller of the two edges (closed form, no root find; the test evaluates the RBJ
                      coefficients independently). The unwarped analogue form (x = w/w0) overstates Q (decision 0013:
                      14 % at h = 2, 36 % at h = 3), so it is not offered.
  epsilon             2^-8 (telemetry mantissa resolution, decision 0013) + odr_error + esc_clock_error, all fractions.
  omega_threshold     omega_hover sqrt(a_min): the notch's active floor.
  vibration_amplitude_max / vibration_sweep
                      A_max = (FS - rate_max) / (n_motors n_harmonics (omega_max/omega_hover)^p): the largest reference
                      amplitude whose worst-case sum (all motors' harmonics in phase, at omega_max, on top of the largest
                      commanded body rate) does not reach the gyro full scale FS; p is the owner's exponent (decision 0013
                      owner decision 2). The sweep is {0} and A_max 2^-k for k = 0, 1, ... down to the first value below
                      sigma_d (owner decision 5).

Chain response: the 12 notches (operating point = the four rotor speeds) and the Butterworth low-pass, with the firmware's
coefficient formulas in double (gyro_chain.hpp notch_coeffs lines 90-107, lowpass_coeffs lines 73-87); the notch activation
rule is update_notches (lines 169-195): a notch is active iff omega >= omega_th and f0 < f_s/2, else the identity.

Exact discrete loop. T_s the tick, T = D T_s the rate-loop period, z = e^{j theta}, theta = omega T, z_s = e^{j theta_s}.
  L(e^{j theta}) = C(z) (1/D) sum_{k=0}^{D-1} F(e^{j (theta + 2 pi k)/D}),
  F(z_s) = (sum_{i<D} z_s^-i) P_s(z_s) H(z_s) z_s^-latency.
C is rate.py's controller (rate.py:16, kp (z - zc)/(z - 1)); P_s is rate.py's plant constants (plant_constants) at T_s, so
the hold of width T is the sum of D holds of width T_s (1 - e^{-sT})/s = (sum z_s^-i)(1 - e^{-s T_s})/s, and the sum over k
is the decimation by D of the tick-rate response (the images the rate loop's sampling folds back). With H = 1 and latency 0
this is exactly rate.py's L(z) at T (cross-check in the tests). Crossover: every |L| = 1 on a log grid, bisected; the phase is
the continuous branch (unwrapped by stepping the principal value along the grid from the analytic low-frequency phase
-pi), not rate.py's sum of atan2 terms (valid only for real-root factors). PM = pi + arg L, the minimum over the crossovers.

    uv run python tools/card/gyro_chain_design.py --report
"""

from __future__ import annotations

import argparse
import cmath
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_imu_config as gic  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import rate  # noqa: E402
import schema  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
TELEMETRY_STEP = 2.0 ** -8        # bidirectional-DShot telemetry mantissa resolution (decision 0013, owner decision 4)
MOTORS = 4                        # quad-X (core section 3)
HARMONICS = (1, 2, 3)             # QF-7: the harmonics the B1 vibration model includes
SQRT2 = math.sqrt(2.0)            # the Butterworth damping, gyro_chain.hpp:11

# Method constants of the numerical search (like rate.py's): resolution and termination only.
GRID_POINTS = 4096
GRID_LOG2_LOW = -20
MAX_BISECTIONS = 200
DELAY_LOG2_STEP = -12             # relative central-difference step of the group delay
MAX_PHASE_STEP = math.pi / 2      # an unwrapping step beyond this is refused (the grid is too coarse for the phase)

# Labelled scenario values of the report (the lead's cases for decision 0013 owner decision 6; registers do not carry them yet).
A_MIN = 0.1                       # gyro_chain_attenuation_min, owner decision 3
ESC_CLOCK_ERRORS = (0.0, 0.02, 0.038)


class DesignError(Exception):
    pass


# ---- Rules ----------------------------------------------------------------------------------------------------------


def lowpass_cutoff_hz(fs, divisor, a_min):
    if divisor < 2:
        raise DesignError(f"rate-loop divisor {divisor} < 2: the low-pass rule needs a rate-loop Nyquist below the tick's")
    if not 0 < a_min < 1:
        raise DesignError(f"a_min {a_min!r} outside (0, 1)")
    fr = fs / divisor
    k = math.tan(math.pi * fr / (2 * fs)) / (1 / (a_min * a_min) - 1) ** 0.25
    return fs / math.pi * math.atan(k)


def epsilon(odr_error, esc_clock_error, telemetry_step=TELEMETRY_STEP):
    return telemetry_step + odr_error + esc_clock_error


def omega_threshold(omega_hover, a_min):
    return omega_hover * math.sqrt(a_min)


def _edge_q(f0, f_edge, fs, a_min):
    """The largest Q with |H(f_edge)| <= a_min for the digital notch at f0 (formula in the module docstring)."""
    k = math.tan(math.pi * f0 / fs)
    x = math.tan(math.pi * f_edge / fs) / k
    return a_min * x / (math.sqrt(1 - a_min * a_min) * abs(1 - x * x))


def notch_q(f0, eps, a_min, fs):
    """Largest Q attenuating to <= a_min at f0 (1 - eps) and f0 (1 + eps), digital notch at f0 (Hz), tick rate fs (Hz)."""
    if not (0 < a_min < 1 and 0 <= eps < 1 and f0 > 0):
        raise DesignError("notch_q: a_min in (0, 1), eps in [0, 1), f0 > 0 required")
    if 2 * f0 * (1 + eps) >= fs:
        raise DesignError(f"notch_q: f0 (1 + eps) = {f0 * (1 + eps)!r} Hz is not below f_s/2")
    return min(_edge_q(f0, f0 * (1 - eps), fs, a_min), _edge_q(f0, f0 * (1 + eps), fs, a_min))


def notch_q_set(omega_max, eps, a_min, fs):
    """Q_h for h = 1, 2, 3, each at f0 = h omega_max / (2 pi)."""
    return tuple(notch_q(h * omega_max / (2 * math.pi), eps, a_min, fs) for h in HARMONICS)


def peak_gyro(amplitude, omega_max, omega_hover, rate_max, exponent):
    """The worst-case gyro magnitude: every motor's every harmonic in phase at omega_max, plus the largest body rate."""
    return MOTORS * len(HARMONICS) * amplitude * (omega_max / omega_hover) ** exponent + rate_max


def vibration_amplitude_max(full_scale, rate_max, omega_max, omega_hover, exponent):
    if not full_scale > rate_max:
        raise DesignError("vibration_amplitude_max: the largest body rate already reaches the gyro full scale")
    return (full_scale - rate_max) / (MOTORS * len(HARMONICS) * (omega_max / omega_hover) ** exponent)


def vibration_sweep(a_max, sigma_d):
    out, a = [0.0], a_max
    while True:
        out.append(a)
        if a < sigma_d:
            return out
        a /= 2


# ---- The chain's coefficients and response (double) -------------------------------------------------------------------


def lowpass_coeffs(f_c, period):
    """(b0, b1, b2, a1, a2), gyro_chain.hpp lowpass_coeffs (lines 73-87)."""
    k = math.tan(math.pi * f_c * period)
    k2 = k * k
    rk = SQRT2 * k
    norm = 1 / (1 + rk + k2)
    b0 = k2 * norm
    return (b0, 2 * b0, b0, 2 * (k2 - 1) * norm, (1 - rk + k2) * norm)


def notch_coeffs(f0, q, period):
    """(b0, b1, b2, a1, a2), gyro_chain.hpp notch_coeffs (lines 90-107, the Audio EQ Cookbook form)."""
    w0 = 2 * math.pi * f0 * period
    alpha = math.sin(w0) / (2 * q)
    cw = math.cos(w0)
    norm = 1 / (1 + alpha)
    b1 = -(2 * cw) * norm
    return (norm, b1, norm, b1, (1 - alpha) * norm)


IDENTITY = (1.0, 0.0, 0.0, 0.0, 0.0)


def chain_stages(period, f_c, q, omega_th, omegas, bypass_all=False):
    """The 12 notch stages (motor-major, harmonic-minor, as the firmware) then the low-pass, at the rotor speeds `omegas`
    (rad/s, all valid). A notch is active iff omega >= omega_th and f0 < f_s/2 (gyro_chain.hpp:169-195), else identity;
    bypass_all forces every notch to the identity."""
    stages = []
    for w in omegas:
        for h, qh in zip(HARMONICS, q):
            f0 = h * w / (2 * math.pi)
            if not bypass_all and w >= omega_th and f0 < 1 / (2 * period):
                stages.append(notch_coeffs(f0, qh, period))
            else:
                stages.append(IDENTITY)
    stages.append(lowpass_coeffs(f_c, period))
    return stages


def stage_response(c, zinv):
    b0, b1, b2, a1, a2 = c
    return (b0 + b1 * zinv + b2 * zinv * zinv) / (1 + a1 * zinv + a2 * zinv * zinv)


def chain_response(stages, theta_s):
    """H(e^{j theta_s}) of the cascade, theta_s = omega T_s."""
    zinv = cmath.exp(-1j * theta_s)
    h = 1 + 0j
    for c in stages:
        h *= stage_response(c, zinv)
    return h


def group_delay_s(stages, omega, period):
    """-d arg H / d omega (s) of the chain at omega (rad/s), central difference (relative step 2^-12)."""
    step = omega * 2.0 ** DELAY_LOG2_STEP
    hp = chain_response(stages, (omega + step) * period)
    hm = chain_response(stages, (omega - step) * period)
    return -cmath.phase(hp / hm) / (2 * step)


# ---- The exact discrete loop ----------------------------------------------------------------------------------------


class Loop:
    """L(e^{j theta}) of the module docstring for one band-box corner. kp, ki: the corner's gains (normalised design gains
    over the true inertia J_true, as rate.loop_margins takes them: kappa / jt); tau: the corner's motor lag; `h` maps theta_s
    to the chain response (None = 1)."""

    def __init__(self, kp, ki, t_s, divisor, tau, h=None, latency=0):
        self.kp, self.ki, self.t_s, self.divisor, self.latency = kp, ki, t_s, divisor, latency
        self.t = divisor * t_s
        self.zc = 1 - ki * self.t / kp
        self.e, _, self.g, self.z0 = rate.plant_constants(tau, t_s)
        self.h = h

    def f(self, theta_s):
        zs = cmath.exp(1j * theta_s)
        hold = sum(zs ** -i for i in range(self.divisor))
        p = self.g * (zs - self.z0) / ((zs - 1) * (zs - self.e))
        h = 1 if self.h is None else self.h(theta_s)
        return hold * p * h * zs ** -self.latency

    def __call__(self, theta):
        z = cmath.exp(1j * theta)
        c = self.kp * (z - self.zc) / (z - 1)
        g = sum(self.f((theta + 2 * math.pi * k) / self.divisor) for k in range(self.divisor)) / self.divisor
        return c * g


_THETA = [math.pi * 2.0 ** (GRID_LOG2_LOW * (1 - i / (GRID_POINTS - 1))) for i in range(GRID_POINTS)]


def _principal(x):
    return cmath.phase(x)


def crossovers(loop):
    """[(theta, PM)] for every |L| = 1 on the grid, bisected; PM = pi + arg L on the continuous branch from low frequency
    (L -> -pi there: two integrators). Raises DesignError when |L| does not run from above 1 to below 1, or a phase step
    on the grid reaches MAX_PHASE_STEP."""
    vals = [loop(t) for t in _THETA]
    if not (abs(vals[0]) > 1 and abs(vals[-1]) < 1):
        raise DesignError("|L| does not fall from above 1 at low frequency to below 1 at Nyquist")
    flips = [i for i in range(GRID_POINTS - 1) if (abs(vals[i]) > 1) != (abs(vals[i + 1]) > 1)]
    last = flips[-1] + 1 if flips else GRID_POINTS - 1
    # arg(-L) is near 0 at the lowest point: the unambiguous start of the branch.
    phase = [_principal(-vals[0]) - math.pi]
    for i in range(1, GRID_POINTS):
        step = _principal(vals[i] / vals[i - 1])
        if i <= last and abs(step) >= MAX_PHASE_STEP:
            raise DesignError(f"phase step {step!r} rad at theta {_THETA[i]!r}: grid too coarse to unwrap")
        phase.append(phase[-1] + step)
    out = []
    for i in range(GRID_POINTS - 1):
        if (abs(vals[i]) > 1) != (abs(vals[i + 1]) > 1):
            lo, hi = _THETA[i], _THETA[i + 1]
            above_lo = abs(vals[i]) > 1
            for _ in range(MAX_BISECTIONS):
                mid = (lo + hi) / 2
                if mid <= lo or mid >= hi:
                    break
                if (abs(loop(mid)) > 1) == above_lo:
                    lo = mid
                else:
                    hi = mid
            theta = (lo + hi) / 2
            ph = phase[i] + _principal(loop(theta) / vals[i])
            out.append((theta, math.pi + ph))
    return out


def margins(kp, ki, t_s, divisor, corners, h=None, latency=0):
    """[(corner name, PM (min over its crossovers), crossover rad/s of that PM, number of crossovers)] for the corners of
    rate.corner_list, gains normalised (J = 1) as rate.loop_margins takes them."""
    t = divisor * t_s
    out = []
    for name, jt, tau in corners:
        xs = crossovers(Loop(kp / jt, ki / jt, t_s, divisor, tau, h, latency))
        theta, pm = min(xs, key=lambda x: x[1])
        out.append((name, pm, theta / t, len(xs)))
    return out


# ---- The report -----------------------------------------------------------------------------------------------------


def load_design(card_path, budget_path, scenario_path, profile_path):
    """(rate.design result, card, scenario, imu si values) of the committed files, the same loaders flatten.py uses."""
    card, budget, scenario = (schema.load_yaml(p) for p in (card_path, budget_path, scenario_path))
    res = rate.design(card, budget, scenario, card_path)
    si, _, _ = gic.imu_config(profile_path)
    return res, card, scenario, si


def chain_design(res, card, scenario, si, a_min, esc_error, telemetry_step=TELEMETRY_STEP):
    """The chain's design numbers for this ESC clock error: (dict). rate.design's T and the scenario's divisor give f_c; the card's
    speed range gives omega_max; omega_hover is sqrt(m G / (4 k)) at standard gravity (labelled: the plant uses the site's
    WGS 84 gravity, which differs by well under 1 %)."""
    divisor = scenario["rate_loop_divisor"]["value"]
    num, den = scenario["tick_period_num_us"]["value"], scenario["tick_period_den"]["value"]
    micro = 1e-6
    t_s = num / den * micro
    fs = 1 / t_s
    mass = card["mass"]["value"]
    k = card["rotors"]["thrust_coeff"]["value"]
    omega_max = float(card["rotors"]["speed_range"]["value"][1])
    omega_hover = math.sqrt(mass * gic.G / (4 * k))
    eps = epsilon(si["odr_error"], esc_error, telemetry_step)
    return {"t_s": t_s, "fs": fs, "divisor": divisor, "omega_max": omega_max, "omega_hover": omega_hover, "eps": eps,
            "f_c": lowpass_cutoff_hz(fs, divisor, a_min), "q": notch_q_set(omega_max, eps, a_min, fs),
            "omega_th": omega_threshold(omega_hover, a_min)}


def report_lines(res, card, scenario, si, a_min=A_MIN, esc_errors=ESC_CLOCK_ERRORS):
    deg = math.degrees
    kp, ki = res["kappa_p"], res["kappa_i"]
    inp = res["inputs"]
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    latency = si["latency_samples"]
    t_s = scenario["tick_period_num_us"]["value"] / scenario["tick_period_den"]["value"] * 1e-6
    divisor = scenario["rate_loop_divisor"]["value"]
    base = margins(kp, ki, t_s, divisor, corners)
    lines = [
        "MARV L6 stage (b) gyro-chain design report (tools/card/gyro_chain_design.py, decision 0013)",
        f"L4 gains (J = 1): kappa_p {kp!r}  kappa_i {ki!r}; rate loop {1 / res['T']:.6g} Hz (D = {divisor}), tick {1 / t_s:.6g} Hz, "
        f"latency {latency} sample(s); a_min {a_min!r}; odr_error {si['odr_error']!r}",
        "phase margin = pi + arg L, unwrapped; corners of the tau x J band box (nominal first); worst corner marked *",
    ]

    def table(title, pm_rows):
        worst = min(r[1] for r in pm_rows)
        lines.append(title)
        for name, pm, wx, n in pm_rows:
            lines.append(f"  {'*' if pm == worst else ' '} {name:<8} crossover {wx:10.5f} rad/s   PM {deg(pm):9.4f} deg"
                         + ("" if n == 1 else f"   ({n} crossovers, minimum shown)"))
        lines.append(f"    worst PM {deg(worst):.4f} deg (PM_min {deg(inp['PM_min']):.4f} deg, margin {deg(worst - inp['PM_min']):+.4f} deg)")

    table("L4 design as built (H = 1, latency 0; rate.py's values)", base)
    for esc in esc_errors:
        d = chain_design(res, card, scenario, si, a_min, esc)
        lines.append(f"ESC clock error {esc!r}: eps {d['eps']!r}; f_c {d['f_c']:.4f} Hz; omega_hover {d['omega_hover']:.3f} rad/s, "
                     f"omega_th {d['omega_th']:.3f} rad/s ({d['omega_th'] / (2 * math.pi):.3f} Hz), omega_max {d['omega_max']!r}")
        lines.append(f"  Q_h (h = 1, 2, 3, at omega_max): {d['q'][0]:.4f} {d['q'][1]:.4f} {d['q'][2]:.4f}")
        for label, bypass in (("worst case: all four notches at omega_th", False), ("all notches bypassed (low-pass only)", True)):
            st = chain_stages(d["t_s"], d["f_c"], d["q"], d["omega_th"], [d["omega_th"]] * MOTORS, bypass)

            def h(theta_s, st=st):
                return chain_response(st, theta_s)

            ms = margins(kp, ki, d["t_s"], d["divisor"], corners, h, latency)
            wc = ms[0][2]
            gd = group_delay_s(st, wc, d["t_s"])
            lines.append(f"  {label}: chain group delay at the nominal crossover {wc:.4f} rad/s: {gd * 1e6:.2f} us "
                         f"(+ {latency} sample latency {latency * d['t_s'] * 1e6:.2f} us = {(gd + latency * d['t_s']) * 1e6:.2f} us, "
                         f"{deg(wc * (gd + latency * d['t_s'])):.4f} deg at the crossover)")
            table("    loop with the chain", ms)
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--report", action="store_true", required=True, help="print the phase-margin report at today's L4 gains")
    ap.add_argument("--card", default=str(ROOT / "vehicles" / "uzh_neurobem_5in.yaml"))
    ap.add_argument("--budget", default=str(ROOT / "design" / "budget.yaml"))
    ap.add_argument("--scenario", default=str(ROOT / "design" / "scenario_values.yaml"))
    ap.add_argument("--profile", default=str(ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"))
    ap.add_argument("--a-min", type=float, default=A_MIN)
    ap.add_argument("--esc", type=float, nargs="*", default=list(ESC_CLOCK_ERRORS))
    args = ap.parse_args(argv)
    try:
        res, card, scenario, si = load_design(args.card, args.budget, args.scenario, args.profile)
        for line in report_lines(res, card, scenario, si, args.a_min, args.esc):
            print(line)
    except (gpc.GenError, DesignError) as e:
        print(e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
