#!/usr/bin/env python3
"""L4 rate-loop parameters from a vehicle card, the design budget and the scenario register (quad spec L4, QF-2,
QF-3, decision 0005 "Gain rule", "QF-2" and "kd").

Used by flatten.py (--out-rate, which needs --scenario). Plain Python double (math), no numpy, like mixer.py. For a
linted card it yields the params_gen entries rate_kp_<axis>, rate_ki_<axis>, rate_kd_<axis> (= 0) and rate_tau_ref_<axis>
for axis in roll, pitch, yaw, and a derivation report. The rule generalises the toolbox's loopshape.ts (Kessler's
symmetric optimum, spacing a) to the exact sampled-data loop and to the tau x J robustness box.

Rule (all inputs come from the card, the budget register, the scenario register or the mixer; nothing is compiled in)

  1. T = rate_loop_divisor * tick_period_num_us / tick_period_den microseconds, in seconds.
  2. Design model, J normalised to 1: P(s) = 1/(s (1 + tau s)), ZOH-discretised exactly at T:
         P(z) = T/(z-1) - tau (1-e)/(z-e) = g (z-z0)/((z-1)(z-e)),  e = exp(-T/tau), g = T - tau (1-e),
         z0 = (T e - tau (1-e))/g.
     Controller as the firmware implements it, C(z) = kp + ki T/(z-1) = kp (z-zc)/(z-1), zc = 1 - (ki/kp) T (forward-Euler
     integrator, the increment ki e_{n-1} T applied at n), zero computation delay, instantaneous gyro sample, kd = 0.
     L = C P. An axis with inertia J_true and gains (J_a kp, J_a ki) has the loop L J_a/J_true.
  3. Crossover: the w in (0, pi/T) with |L(e^{jwT})| = 1, required unique (found on a 1024-point log grid from
     pi 2^-20 to pi rad/sample, then bisected; a grid with zero or several sign changes of |L| - 1 is refused).
     PM = pi + arg L on the continuous branch from low frequency: every factor of L is (z - real) and its argument, on
     the upper unit half circle, lies in (0, pi), so
         PM = atan2(s, c-zc) + atan2(s, c-z0) - atan2(s, c-e) - wT,  c + js = e^{jwT}.
  4. a = sqrt((1 + sin PM_min)/(1 - sin PM_min)). For a nominal crossover w: ki/kp = w/a and kp such that
     |L_nom(e^{jwT})| = 1 (J_true = 1, tau nominal). PM_worst(w) = minimum PM over nominal and the four corners
     J_true in {1 - b_J, 1 + b_J} x tau in {(1 - b_tau) tau, (1 + b_tau) tau}; a point where a corner has no unique
     crossover is infeasible.
  5. w* = sup{w : PM_worst(w') >= PM_min for all w' in (0, w]}: geometric scan upward from pi/T 2^-17 rad/s in steps of
     2^(1/16), to the first infeasible point, then bisection in that bracket. The bisection stops when the f32-rounded
     gains (kp_a, ki_a) = (f32(J_a kp), f32(J_a ki)) of the midpoint equal those of a bracket end on every axis, or the
     bracket collapses in double. The feasible end is taken.
  6. Guard: PM_worst is evaluated on the f32-rounded gains of every axis (kappa = f32 gain / J_a) and must be at least
     PM_min + delta_num, delta_num = (max over the five loops of |dPM/dw| at the crossover, by a central difference of
     step 2^-20 w) x (the final bracket width). A candidate that fails is replaced by the previous feasible point of the
     scan-and-bisection history. No feasible candidate: the card is refused.
  7. Condition of the continuous-model monotonicity proof: at the corner (J_lo, tau_lo), w_c^2 T tau_lo < 1/2 with w_c
     that corner's crossover (of the design model, on the chosen gains); the card is refused otherwise.
  8. tau_cl = max over the five loops of t63, the first rate-loop sample time n T at which the closed-loop step response
     (no prefilter) of the discrete design model, on the chosen (double) gains, reaches 1 - exp(-1).
  9. tau_max,a: the largest tau >= 0 such that some collective c has f_min <= c M[i,thrust] + tau M[i,a] <= f_max for
     every motor i (f_min = k idle^2, f_max = k speed_max^2), solved exactly from the pairwise form
         tau D_ij >= R_ij,  D_ij = M[i,a] M[j,thrust] - M[j,a] M[i,thrust],  R_ij = f_min M[j,thrust] - f_max M[i,thrust].
     Both torque signs are solved and the smaller is taken (equal for a symmetric card). alpha_max,a = tau_max,a /
     (J_a (1 + b_J)); tau_ref,a = max(tau_cl, rate_max_a / alpha_max,a), stored as the smallest f32 >= that maximum.
 10. Emitted f32, method derived(<rule>), sigma UNKNOWN (kd: 0, a definition, exact): rate_kp (N m s/rad), rate_ki
     (N m/rad), rate_kd (0, N m s^2/rad), rate_tau_ref (s).

The generator refuses (GenError) a card, budget or scenario register for which a step is undefined or infeasible.
"""

from __future__ import annotations

import math
import struct

import gen_plant_config as gpc
import mixer
import schema

AXES = ("roll", "pitch", "yaw")
INERTIA_INDEX = {"roll": 0, "pitch": 1, "yaw": 2}
MIXER_COLUMN = {"roll": 1, "pitch": 2, "yaw": 3}
CORNER_NAMES = ("J-,tau-", "J-,tau+", "J+,tau-", "J+,tau+")

# Method constants of the numerical search (decision 0005, "Generator numerics"), not vehicle or design numbers: grid
# sizes, scan start and step, iteration caps and the finite-difference step. They set resolution and termination only;
# the rule's result is the f32 gains, whose stopping test (bisection until the f32 gains stop changing) and guard
# (PM_min + delta_num) do not depend on these values beyond making the search reach them.
GRID_POINTS = 1024
GRID_LOG2_LOW = -20
SCAN_LOG2_START = -17
SCAN_STEPS_PER_OCTAVE = 16
MAX_BISECTIONS = 200
DERIVATIVE_LOG2_STEP = -20
MAX_STEP_SAMPLES = 2 ** 24

GAIN_METHOD = (
    "derived(rate PI by the robust loop-shaping rule of decision 0005: integral zero w_c,nom/a with "
    "a = sqrt((1 + sin PM_min)/(1 - sin PM_min)); w_c,nom the largest crossover whose worst-case phase margin over the "
    "tau_robustness_band x inertia_robustness_band box stays >= PM_min, on the exact ZOH-discretised design model "
    "1/(s (1 + tau s)) at the rate-loop period with the firmware's forward-Euler PI; kappa_p from |L(w_c,nom)| = 1, "
    "kp = J kappa_p, ki = J kappa_i; tools/card/rate.py)")
GAIN_SOURCE = ("vehicle card inertia_diag and rotors.motor_lag.tau; design-budget PM_min, tau_robustness_band, "
               "inertia_robustness_band; scenario register tick_period_num_us, tick_period_den, rate_loop_divisor "
               "(decision 0005, gain rule)")
KD_METHOD = "derived(PI rule, no derivative term; D design at L6, decision 0005)"
KD_SOURCE = "decision 0005 owner decision 5: PID structure, kd = 0; the D design moves to L6"
TAU_REF_METHOD = (
    "derived(tau_ref = max(tau_cl, rate_max/alpha_max) of decision 0005 QF-2: tau_cl the largest 63.2 % rise time over "
    "the robustness box of the design-model closed-loop step response, alpha_max = tau_max/(J (1 + "
    "inertia_robustness_band)), tau_max the air-mode envelope torque of the mixer M with the collective free; "
    "tools/card/rate.py)")
TAU_REF_SOURCE = ("the gain rule's inputs (decision 0005) and the vehicle card rotors thrust_coeff and speed_range, the "
                  "mixer M and the scenario register rate_max_<axis>")


def r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def r32_up(x):
    """The smallest float32 >= x (as a double): tau_ref is rounded up so that the stored f32 is >= both of its terms."""
    y = r32(x)
    if y >= x:
        return y
    bits = struct.unpack("<I", struct.pack("<f", y))[0]
    return struct.unpack("<f", struct.pack("<I", bits + 1))[0]


def refuse(card_path, reason):
    gpc.refuse(card_path, "rate", f"{reason} (decision 0005)")


def plant_constants(tau, T):
    """(e, 1 - e, g, z0) of the ZOH-sampled 1/(s (1 + tau s)) at period T."""
    x = T / tau
    e = math.exp(-x)
    one_minus_e = -math.expm1(-x)
    g = T - tau * one_minus_e
    z0 = (T * e - tau * one_minus_e) / g
    return e, one_minus_e, g, z0


class _Grid:
    def __init__(self):
        n = GRID_POINTS
        self.theta = [math.pi * 2.0 ** (GRID_LOG2_LOW * (1 - i / (n - 1))) for i in range(n)]
        self.cos = [math.cos(t) for t in self.theta]
        self.sin = [math.sin(t) for t in self.theta]
        self.half = [math.sin(t / 2) for t in self.theta]


_GRID = _Grid()


def _mag(c, s, half, kp, zc, pc):
    # |z - 1|^2 = 4 sin^2(wT/2) on the unit circle (half = sin(wT/2)): the 4 is that identity, not a parameter.
    e, _, g, z0 = pc
    return kp * g * math.hypot(c - zc, s) * math.hypot(c - z0, s) / (4 * half * half * math.hypot(c - e, s))


def _pm(c, s, theta, zc, pc):
    e, _, _, z0 = pc
    return math.atan2(s, c - zc) + math.atan2(s, c - z0) - math.atan2(s, c - e) - theta


def loop_crossover(kp, ki, T, pc):
    """(crossover angle w T, PM, reason): angle and PM are None when |L| = 1 has no unique solution on the grid."""
    zc = 1 - ki * T / kp
    g = _GRID
    above = [_mag(g.cos[i], g.sin[i], g.half[i], kp, zc, pc) > 1 for i in range(GRID_POINTS)]
    changes = [i for i in range(GRID_POINTS - 1) if above[i] != above[i + 1]]
    if not above[0] or above[-1]:
        return None, None, "|L| does not fall from above 1 at low frequency to below 1 at Nyquist"
    if len(changes) != 1:
        return None, None, f"|L| = 1 has {len(changes)} solutions on the grid"
    lo, hi = g.theta[changes[0]], g.theta[changes[0] + 1]
    for _ in range(MAX_BISECTIONS):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        if _mag(math.cos(mid), math.sin(mid), math.sin(mid / 2), kp, zc, pc) > 1:
            lo = mid
        else:
            hi = mid
    theta = (lo + hi) / 2
    return theta, _pm(math.cos(theta), math.sin(theta), theta, zc, pc), ""


def spacing(pm_min):
    s = math.sin(pm_min)
    return math.sqrt((1 + s) / (1 - s))


def design_gains(omega, a, T, pc):
    """(kp, ki) of the normalised design (J = 1) with crossover omega on the plant pc."""
    theta = omega * T
    zc = 1 - omega / a * T
    m = _mag(math.cos(theta), math.sin(theta), math.sin(theta / 2), 1.0, zc, pc)
    return 1 / m, omega / a / m


def corner_list(tau, b_tau, b_j):
    return [("nominal", 1.0, tau), ("J-,tau-", 1 - b_j, (1 - b_tau) * tau), ("J-,tau+", 1 - b_j, (1 + b_tau) * tau),
            ("J+,tau-", 1 + b_j, (1 - b_tau) * tau), ("J+,tau+", 1 + b_j, (1 + b_tau) * tau)]


def loop_margins(kp, ki, T, corners):
    """[(name, PM, crossover rad/s, reason)] of the loop kp, ki (normalised gains, J = 1) on each corner."""
    out = []
    for name, jt, tau in corners:
        theta, pm, why = loop_crossover(kp / jt, ki / jt, T, plant_constants(tau, T))
        out.append((name, pm, None if theta is None else theta / T, why))
    return out


def worst_margin(kp, ki, T, corners):
    ms = loop_margins(kp, ki, T, corners)
    if any(m[1] is None for m in ms):
        return -math.inf, ms
    return min(m[1] for m in ms), ms


def rise_time(kp, ki, T, tau, jt):
    """t63: first sample time n T of the discrete design model's closed-loop unit step response to reach 1 - e^-1."""
    kp, ki = kp / jt, ki / jt
    e, one_minus_e, _, _ = plant_constants(tau, T)
    target = 1 - math.exp(-1)
    w = m = xi = 0.0
    for n in range(MAX_STEP_SAMPLES):
        if w >= target:
            return n * T
        err = 1 - w
        u = kp * err + xi
        xi += ki * T * err
        w, m = w + T * u + (m - u) * tau * one_minus_e, e * m + one_minus_e * u
    return None


def _tau_max_one_sign(mt, ma, f_min, f_max):
    best = None
    for i in range(len(mt)):
        for j in range(len(mt)):
            if i == j:
                continue
            d = ma[i] * mt[j] - ma[j] * mt[i]
            r = f_min * mt[j] - f_max * mt[i]
            if d < 0:
                c = r / d
                best = c if best is None else min(best, c)
    return best


def tau_max(m, axis, f_min, f_max):
    """(tau_max, tau_max for +torque, for -torque): the air-mode envelope of one axis, the collective free."""
    mt = [row[0] for row in m]
    ma = [row[MIXER_COLUMN[axis]] for row in m]
    pos = _tau_max_one_sign(mt, ma, f_min, f_max)
    neg = _tau_max_one_sign(mt, [-x for x in ma], f_min, f_max)
    if pos is None or neg is None or pos < 0 or neg < 0:
        return None, pos, neg
    return min(pos, neg), pos, neg


def envelope_feasible(m, axis, torque, f_min, f_max, tol=0.0):
    """Whether some collective c gives f_min <= c M[i,thrust] + torque M[i,axis] <= f_max on every motor (tol widens)."""
    lo = max((f_min - torque * row[MIXER_COLUMN[axis]]) / row[0] for row in m)
    hi = min((f_max - torque * row[MIXER_COLUMN[axis]]) / row[0] for row in m)
    return lo <= hi + tol


def _value(doc, name, where, card_path):
    if name not in doc or not schema.is_number(doc[name]["value"]):
        refuse(card_path, f"{where} {name} is missing or UNKNOWN")
    return doc[name]["value"]


def design(card, budget, scenario, card_path):
    """The whole rule: a dict of results (raises gpc.GenError when the card is refused)."""
    inertia_entry = card["inertia_diag"]
    tau_entry = card["rotors"]["motor_lag"]["tau"]
    if inertia_entry["value"] == schema.UNKNOWN or tau_entry["value"] == schema.UNKNOWN:
        refuse(card_path, "inertia_diag or rotors.motor_lag.tau is UNKNOWN")
    inertia = [float(v) for v in inertia_entry["value"]]
    tau = float(tau_entry["value"])
    pm_min = float(_value(budget, "PM_min", "budget entry", card_path))
    b_tau = float(_value(budget, "tau_robustness_band", "budget entry", card_path))
    b_j = float(_value(budget, "inertia_robustness_band", "budget entry", card_path))
    num = _value(scenario, "tick_period_num_us", "scenario entry", card_path)
    den = _value(scenario, "tick_period_den", "scenario entry", card_path)
    divisor = _value(scenario, "rate_loop_divisor", "scenario entry", card_path)
    rate_max = {a: float(_value(scenario, f"rate_max_{a}", "scenario entry", card_path)) for a in AXES}
    if not (tau > 0 and all(j > 0 for j in inertia) and 0 < pm_min < math.pi / 2 and 0 <= b_tau < 1 and 0 <= b_j < 1
            and num > 0 and den > 0 and divisor >= 1):
        refuse(card_path, "an input is out of its domain (tau, J > 0; 0 < PM_min < pi/2; bands in [0, 1); "
                          "tick period > 0; divisor >= 1)")
    period_us = divisor * num / den
    T = period_us * 1e-6  # µs to s (SI prefix micro)
    a = spacing(pm_min)
    corners = corner_list(tau, b_tau, b_j)
    pc_nom = plant_constants(tau, T)
    theta_nyquist = math.pi

    def normalised(omega):
        return design_gains(omega, a, T, pc_nom)

    def f32_gains(omega):
        kp, ki = normalised(omega)
        return tuple((r32(j * kp), r32(j * ki)) for j in inertia)

    def feasible(omega):
        kp, ki = normalised(omega)
        return worst_margin(kp, ki, T, corners)[0] >= pm_min

    ratio = 2.0 ** (1 / SCAN_STEPS_PER_OCTAVE)
    omega = math.pi / T * 2.0 ** SCAN_LOG2_START
    if not feasible(omega):
        refuse(card_path, f"no feasible crossover: the scan start {omega:.6g} rad/s already violates PM_min")
    history = []
    lo = hi = None
    while lo is None:
        history.append(omega)
        nxt = omega * ratio
        if nxt * T >= theta_nyquist:
            refuse(card_path, "the worst-case margin stays above PM_min up to Nyquist: the scan finds no bracket")
        if feasible(nxt):
            omega = nxt
        else:
            lo, hi = omega, nxt
    scan_bracket = (lo, hi)
    bisections = 0
    for _ in range(MAX_BISECTIONS):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        if f32_gains(mid) in (f32_gains(lo), f32_gains(hi)):
            break
        bisections += 1
        if feasible(mid):
            lo = mid
            history.append(mid)
        else:
            hi = mid
    width = hi - lo

    step = lo * 2.0 ** DERIVATIVE_LOG2_STEP
    slope = 0.0
    up, down = normalised(lo + step), normalised(lo - step)
    for k in range(len(corners)):
        pu = worst_margin(*up, T, [corners[k]])[0]
        pd = worst_margin(*down, T, [corners[k]])[0]
        slope = max(slope, abs(pu - pd) / (2 * step))
    delta_num = slope * width

    chosen = None
    for cand in reversed(history):
        gains = f32_gains(cand)
        per_axis = {}
        for axis, (kp32, ki32) in zip(AXES, gains):
            j = inertia[INERTIA_INDEX[axis]]
            per_axis[axis] = worst_margin(kp32 / j, ki32 / j, T, corners)
        pm_f32 = min(v[0] for v in per_axis.values())
        if pm_f32 >= pm_min + delta_num:
            chosen = (cand, gains, per_axis, pm_f32)
            break
    if chosen is None:
        refuse(card_path, "no feasible crossover passes the guard PM_worst(f32 gains) >= PM_min + delta_num")
    omega_c, gains32, per_axis, pm_f32 = chosen
    kp_n, ki_n = normalised(omega_c)
    pm_worst, margins = worst_margin(kp_n, ki_n, T, corners)
    stepped_down = history.index(omega_c) != len(history) - 1

    lo_corner = next(m for m in margins if m[0] == "J-,tau-")
    tau_lo = (1 - b_tau) * tau
    condition = lo_corner[2] ** 2 * T * tau_lo
    if not condition < 0.5:
        refuse(card_path, f"the monotonicity condition w_c^2 T tau_lo = {condition:.6g} is not below 1/2 at the corner "
                          "(J-, tau-)")

    t63 = []
    for name, jt, tc in corners:
        t = rise_time(kp_n, ki_n, T, tc, jt)
        if t is None:
            refuse(card_path, f"the step response at {name} does not reach 1 - e^-1 within {MAX_STEP_SAMPLES} samples")
        t63.append((name, t))
    tau_cl = max(t for _, t in t63)

    m_matrix, _, idle = mixer.mixer_matrix(card, card_path)
    rotors = card["rotors"]
    k_thrust = float(gpc._known(rotors["thrust_coeff"], "rotors.thrust_coeff", card_path))
    speed = gpc._known(rotors["speed_range"], "rotors.speed_range", card_path)
    f_min, f_max = k_thrust * idle * idle, k_thrust * float(speed[1]) ** 2
    axes = {}
    for axis in AXES:
        j = inertia[INERTIA_INDEX[axis]]
        t_max, t_pos, t_neg = tau_max(m_matrix, axis, f_min, f_max)
        if t_max is None:
            refuse(card_path, f"the air-mode envelope of axis {axis} is unbounded or empty")
        alpha = t_max / (j * (1 + b_j))
        authority = rate_max[axis] / alpha
        idx = AXES.index(axis)
        axes[axis] = {"J": j, "kp": gains32[idx][0], "ki": gains32[idx][1], "kp_double": j * kp_n, "ki_double": j * ki_n,
                      "tau_max": t_max, "tau_max_pos": t_pos, "tau_max_neg": t_neg, "alpha_max": alpha,
                      "authority_term": authority, "tau_ref_exact": max(tau_cl, authority),
                      "tau_ref": r32_up(max(tau_cl, authority)), "rate_max": rate_max[axis],
                      "pm_worst_f32": per_axis[axis][0]}
    return {"T": T, "period_us": period_us, "a": a, "omega_c": omega_c, "kappa_p": kp_n, "kappa_i": ki_n,
            "inputs": {"inertia": inertia, "tau": tau, "PM_min": pm_min, "b_tau": b_tau, "b_J": b_j,
                       "f_min": f_min, "f_max": f_max},
            "margins": margins, "pm_nom": margins[0][1], "pm_worst": pm_worst, "pm_worst_f32": pm_f32,
            "delta_num": delta_num, "slope": slope, "bracket_width": width, "scan_bracket": scan_bracket,
            "bisections": bisections, "stepped_down": stepped_down, "condition": condition, "t63": t63,
            "tau_cl": tau_cl, "axes": axes}


def entries_from(result):
    out = []
    ax = result["axes"]
    for axis in AXES:
        out.append((f"rate_kp_{axis}", {"type": "f32", "value": ax[axis]["kp"], "unit": "N m s/rad",
                                        "method": GAIN_METHOD, "source": GAIN_SOURCE, "sigma": schema.UNKNOWN}))
    for axis in AXES:
        out.append((f"rate_ki_{axis}", {"type": "f32", "value": ax[axis]["ki"], "unit": "N m/rad",
                                        "method": GAIN_METHOD, "source": GAIN_SOURCE, "sigma": schema.UNKNOWN}))
    for axis in AXES:
        out.append((f"rate_kd_{axis}", {"type": "f32", "value": 0, "unit": "N m s^2/rad", "method": KD_METHOD,
                                        "source": KD_SOURCE, "sigma": 0}))
    for axis in AXES:
        out.append((f"rate_tau_ref_{axis}", {"type": "f32", "value": ax[axis]["tau_ref"], "unit": "s",
                                             "method": TAU_REF_METHOD, "source": TAU_REF_SOURCE,
                                             "sigma": schema.UNKNOWN}))
    return out


def report_text(result, where):
    r, i = result, result["inputs"]
    deg = math.degrees
    lines = [
        "MARV L4 rate-loop derivation (tools/card/rate.py, decision 0005)",
        f"card                     {where}",
        f"J (kg m^2)               roll {i['inertia'][0]!r}  pitch {i['inertia'][1]!r}  yaw {i['inertia'][2]!r}",
        f"motor tau (s)            {i['tau']!r}",
        f"PM_min (rad, deg)        {i['PM_min']!r}  {deg(i['PM_min']):.6f}",
        f"bands                    tau +-{i['b_tau']!r}  J +-{i['b_J']!r}",
        f"T (s)                    {r['T']!r}  ({r['period_us']!r} us, {1 / r['T']:.6g} Hz)",
        f"spacing a                {r['a']!r}",
        f"w_c,nom (rad/s)          {r['omega_c']!r}",
        f"kappa_p, kappa_i (J = 1) {r['kappa_p']!r}  {r['kappa_i']!r}",
        f"scan bracket (rad/s)     {r['scan_bracket'][0]!r} .. {r['scan_bracket'][1]!r}",
        f"bisections               {r['bisections']}  final bracket width {r['bracket_width']!r} rad/s",
        f"stepped down by the guard {r['stepped_down']}",
        f"max |dPM/dw|             {r['slope']!r} rad per rad/s",
        f"delta_num (rad)          {r['delta_num']!r}",
        "phase margin per loop (double design gains): crossover rad/s, PM rad, PM deg",
    ]
    for name, pm, wx, _ in r["margins"]:
        lines.append(f"  {name:<8}               {wx!r}  {pm!r}  {deg(pm):.6f}")
    lines += [
        f"PM_nom (deg)             {deg(r['pm_nom']):.6f}",
        f"PM_worst (rad, deg)      {r['pm_worst']!r}  {deg(r['pm_worst']):.6f}",
        f"PM_worst on f32 gains    {r['pm_worst_f32']!r}  {deg(r['pm_worst_f32']):.6f}  (guard: >= PM_min + delta_num)",
        f"condition w_c^2 T tau_lo {r['condition']!r}  (must be < 1/2; w_c the corner J-,tau- crossover)",
        "t63 per loop (s):        " + "  ".join(f"{n} {t!r}" for n, t in r["t63"]),
        f"tau_cl (s)               {r['tau_cl']!r}",
        f"f_min, f_max (N)         {i['f_min']!r}  {i['f_max']!r}",
    ]
    for axis in AXES:
        v = r["axes"][axis]
        lines += [
            f"axis {axis}",
            f"  kp, ki (f32)           {v['kp']!r}  {v['ki']!r}   (double: {v['kp_double']!r}  {v['ki_double']!r})",
            f"  tau_max (N m)          {v['tau_max']!r}  (+ {v['tau_max_pos']!r}, - {v['tau_max_neg']!r})",
            f"  alpha_max (rad/s^2)    {v['alpha_max']!r}",
            f"  rate_max/alpha_max (s) {v['authority_term']!r}  (rate_max {v['rate_max']!r} rad/s)",
            f"  tau_ref (s)            {v['tau_ref']!r}  (max of tau_cl and the line above, {v['tau_ref_exact']!r}, up to f32)",
        ]
    return lines


def rate_entries(card, budget, scenario, card_path, where=None):
    """(ordered (name, params_gen entry) list, report lines, result dict); raises gpc.GenError on refusal. `where` is
    the card path written into the report (default card_path)."""
    result = design(card, budget, scenario, card_path)
    return entries_from(result), report_text(result, where or card_path), result
