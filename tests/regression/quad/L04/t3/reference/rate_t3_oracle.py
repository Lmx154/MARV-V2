#!/usr/bin/env python3
"""Independent double-precision oracle of the L4 T3 suite (quad spec 4 L4, decision 0005 "T3"; quad spec 4 L6 stage (c),
decision 0014: the D term with its low-pass, and the gyro chain in front of the rate loop).

Frozen path: tests/regression/quad/L04/t3/reference/rate_t3_oracle.py. Plain Python math, no numpy, no code shared with
the C++ rate loop or gyro chain. Reads rate_t3_inputs.txt (the f32 parameter values, beside this script) and writes
rate_t3_golden.txt and rate_t3_envelope.txt beside itself (or into --dir):

    uv run python tests/regression/quad/L04/t3/reference/rate_t3_oracle.py
    uv run python tests/regression/quad/L04/t3/reference/rate_t3_oracle.py --refresh-inputs \
        build/<preset>/generated/marv_params/marv/params/param_defaults.cpp

--refresh-inputs rewrites rate_t3_inputs.txt from a generated parameter table (the product set) and stops. The file is
the T3 fixture; the golden and the envelope are functions of it.

What it computes, per axis a in (roll, pitch, yaw), in double:

  Gyro chain (fw/gyro_chain as the rate-group step fw/rate_group runs it): every tick the gyro sample goes through the
  chain. T3 has no rotor-speed sample, so every notch is bypassed (an exact identity) and the chain is its low-pass: the
  second-order Butterworth through the bilinear transform prewarped at f_c = gyro_lpf_cutoff_hz, direct form I, its
  states seeded with the first sample (seed_first_sample). With h = num / den us the tick period:
      K = tan(pi f_c h),  norm = 1 / (1 + sqrt(2) K + K^2),  b0 = K^2 norm,  b1 = 2 b0,  b2 = b0,
      a1 = 2 (K^2 - 1) norm,  a2 = (1 - sqrt(2) K + K^2) norm,
      f_t = b0 x_t + b1 x_(t-1) + b2 x_(t-2) - a1 f_(t-1) - a2 f_(t-2),  x_t the plant omega at tick t (no sensor latency).
  Law (the firmware's, decision 0005 "PID law" with the D low-pass of decision 0014; no anti-windup, mixer out of the
  loop, feed-forward off as in the product parameters). Executions n = 1..N at tick index D (n - 1), D =
  rate_loop_divisor, with stamp t_n = floor(tick * num / den) microseconds, dt_n = t_n - t_(n-1) in seconds (the stamps
  alternate 312 / 313 us at 312.5 us), y_n the chain output at that tick:
      n = 1:  r = y, I = 0, e = 0, Df = 0, u = 0
      n > 1:  alpha = 1 - exp(-dt_n / tau_ref);  r += alpha (sp - r);  e_n = r - y_n;  I += ki e_(n-1) dt_n;
              d_n = -kd (y_n - y_(n-1)) / dt_n;  Df += (1 - exp(-dt_n / T_f)) (d_n - Df)  (Df = d_n when T_f = 0);
              u_n = kp e_n + I + Df
  with the f32 parameter values as doubles, T_f = rate_d_filter_tau_a. The setpoint is 0 -> rate_max_a at execution 1
  (seed: r = y = 0).
  Plant (design model): J w' = u_m, tau u_m' = u - u_m, u held over the D ticks of an execution (zero computation
  delay), advanced tick by tick by the exact per-tick ZOH map (h = num / den us), since the chain samples every tick.

  Golden: y_n = omega at each execution, nominal J = inertia_<a>, tau = motor_tau.
  Horizon: N = 1 + ceil(HORIZON_TAUS * tau_ref_a / T), T = D num / den us (a stated rule: ten reference time constants).

  Rounding bound (README, first order): with U = 2^-24 (binary32 unit roundoff, round to nearest; epsilon = 2^-23) the
  float chain's and law's local rounding errors are injected at seven nodes (x, f, r, e, I, D, u) with the magnitudes
  RHO_* below, evaluated on the double trajectory; each propagates to y through the closed-loop impulse response g_node
  from that node, whose l1 norm (sum over a horizon-long response, maximum over the two dt phases; for the chain nodes x
  and f, injected every tick, the sum of the D tick positions of an execution) this script computes. The tolerance is
      TOL_a = sum over the nodes of l1_node max(rho_node).
  The f node carries the float coefficients' error as well: first-order bounds on |b0 - fl(b0)| .. |a2 - fl(a2)| from
  the coefficient formula's float operations (lowpass_f32_error) times the operands of each tick.

  Envelope: pointwise min / max over the tau x J band box (J (1 +- inertia_robustness_band), tau (1 +- tau_robustness_band))
  of the design-model step responses with the same prefilter, chain low-pass, D low-pass and f32 gains: the corners plus
  a 9 x 9 grid, then one halving (17 x 17); the largest change of any envelope point between the two is recorded.
"""
import argparse
import math
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

AXES = ("roll", "pitch", "yaw")
INERTIA = {"roll": "inertia_xx", "pitch": "inertia_yy", "yaw": "inertia_zz"}
NODES = ("x", "f", "r", "e", "I", "D", "u")
TICK_NODES = ("x", "f")  # the chain's nodes, injected at every tick (the others once per execution)

# Scenario test values (rationale in README.md).
HORIZON_TAUS = 10       # horizon: ten reference time constants
GRID = 9                # envelope grid points per band axis; the halving doubles the intervals (2 (GRID - 1) + 1)
UNIT_ROUNDOFF = 2.0 ** -24  # binary32, round to nearest
MICROSECOND = 1.0e-6

INPUT_KEYS = (
    "rate_loop_divisor", "tick_period_num_us", "tick_period_den",
    "inertia_xx", "inertia_yy", "inertia_zz", "motor_tau",
    "rate_max_roll", "rate_max_pitch", "rate_max_yaw",
    "rate_kp_roll", "rate_kp_pitch", "rate_kp_yaw",
    "rate_ki_roll", "rate_ki_pitch", "rate_ki_yaw",
    "rate_kd_roll", "rate_kd_pitch", "rate_kd_yaw",
    "rate_d_filter_tau_roll", "rate_d_filter_tau_pitch", "rate_d_filter_tau_yaw",
    "rate_tau_ref_roll", "rate_tau_ref_pitch", "rate_tau_ref_yaw",
    "gyro_lpf_cutoff_hz", "gyro_notch_q_h1", "gyro_notch_q_h2", "gyro_notch_q_h3", "gyro_notch_omega_min",
    "tau_robustness_band", "inertia_robustness_band",
)
INT_KEYS = ("rate_loop_divisor", "tick_period_num_us", "tick_period_den")


def r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def ulp32(x):
    """One unit in the last place of the binary32 number |x| (normal range)."""
    return 2.0 ** (math.frexp(abs(x))[1] - 24)


def read_inputs(path):
    values = {}
    with open(path) as f:
        for line in f:
            line = line.split("#")[0].split()
            if not line:
                continue
            key, text = line
            values[key] = int(text) if key in INT_KEYS else float.fromhex(text)
    missing = [k for k in INPUT_KEYS if k not in values]
    if missing:
        sys.exit(f"{path}: missing {missing}")
    return values


def refresh_inputs(defaults_cpp, path):
    text = open(defaults_cpp).read()
    table = {}
    for name, kind, f32, i32 in re.findall(r"// (\w+)\n\s*\{\{ParamType::(F32|I32), ([^,]+), (-?\d+)\}", text):
        table[name] = int(i32) if kind == "I32" else r32(float(f32.rstrip("f")))
    with open(path, "w") as out:
        out.write("# The T3 fixture: the f32 values the oracle and the T3 test use (hex floats are exact). Equal to the product\n")
        out.write(f"# parameters of 2026-10-01, stage (c) of decision 0014 (rate_t3_oracle.py --refresh-inputs from "
                  f"{os.path.basename(defaults_cpp)});\n")
        out.write("# the T3 test reads this file, not the live parameters, so it does not follow later parameter changes.\n")
        out.write("# gyro_notch_*: the chain's configuration only; every notch is bypassed in T3 (no rotor-speed sample).\n")
        for key in INPUT_KEYS:
            v = table[key]
            out.write(f"{key} {v}\n" if key in INT_KEYS else f"{key} {v.hex()}  # {v:.9g}\n")


def lowpass_coeffs(f_c, h):
    """(b0, b1, b2, a1, a2) of the chain's Butterworth low-pass at f_c Hz for the tick period h s, in double."""
    k = math.tan(math.pi * f_c * h)
    k2 = k * k
    rk = math.sqrt(2.0) * k
    norm = 1.0 / (1.0 + rk + k2)
    b0 = k2 * norm
    return b0, 2.0 * b0, b0, 2.0 * (k2 - 1.0) * norm, (1.0 - rk + k2) * norm


def lowpass_f32_error(f_c, h):
    """First-order bounds on |c - fl(c)|, c = b0, b1, b2, a1, a2, when the firmware computes them in binary32 (README):
    theta = fl(fl(pi_f f_c) h_f) with pi_f and the tick period h_f rounded once each (4 U theta), tanf within one ulp
    (INFERRED, as expf), sqrtf(2) correctly rounded, then every operation of the formula as written (b1 = 2 b0 exact)."""
    u = UNIT_ROUNDOFF
    theta = math.pi * f_c * h
    k = math.tan(theta)
    dk = (1.0 + k * k) * 4.0 * u * theta + ulp32(k)
    k2 = k * k
    dk2 = 2.0 * k * dk + u * k2
    rk = math.sqrt(2.0) * k
    drk = math.sqrt(2.0) * dk + 2.0 * u * rk
    den = 1.0 + rk + k2
    dden = drk + u * (1.0 + rk) + dk2 + u * den
    norm = 1.0 / den
    dnorm = norm * (dden / den + u)
    b0 = k2 * norm
    db0 = norm * dk2 + k2 * dnorm + u * b0
    m = k2 - 1.0
    dm = dk2 + u * abs(m)
    a1 = 2.0 * m * norm
    da1 = 2.0 * dm * norm + 2.0 * abs(m) * dnorm + u * abs(a1)
    q = 1.0 - rk + k2
    dq = drk + u * abs(1.0 - rk) + dk2 + u * abs(q)
    a2 = q * norm
    da2 = dq * norm + abs(q) * dnorm + u * abs(a2)
    return db0, 2.0 * db0, db0, da1, da2


class Setup:
    def __init__(self, p):
        self.p = p
        self.divisor = p["rate_loop_divisor"]
        self.num = p["tick_period_num_us"]
        self.den = p["tick_period_den"]
        self.tick_s = self.num / self.den * MICROSECOND
        self.period_s = self.divisor * self.tick_s
        self.tau_band = p["tau_robustness_band"]
        self.j_band = p["inertia_robustness_band"]
        self.cutoff_hz = p["gyro_lpf_cutoff_hz"]
        self.lowpass = lowpass_coeffs(self.cutoff_hz, self.tick_s)
        self.lowpass_error = lowpass_f32_error(self.cutoff_hz, self.tick_s)

    def stamp_us(self, n):
        """Sample stamp of execution n (1-based): floor(tick * num / den) microseconds, tick = D (n - 1)."""
        return (self.divisor * (n - 1) * self.num) // self.den

    def dt(self, n):
        return (self.stamp_us(n) - self.stamp_us(n - 1)) * MICROSECOND

    def executions(self, axis):
        return 1 + math.ceil(HORIZON_TAUS * self.p[f"rate_tau_ref_{axis}"] / self.period_s)

    def tick_map(self, inertia, tau):
        """(a12, b1, a22, b2) of one tick of input u:  w' = w + a12 u_m + b1 u,  u_m' = a22 u_m + b2 u."""
        h = self.tick_s
        x = h / tau
        e = math.exp(-x)
        em = -math.expm1(-x)
        # One tick: u_m' = e u_m + em u ;  w' = w + (tau em / J) u_m + ((h - tau em) / J) u.
        return tau * em / inertia, (h - tau * em) / inertia, e, em

    def plant_map(self, inertia, tau):
        """(a11, a12, b1, a21, a22, b2): state (w, u_m) after D ticks of input u:  w' = w + a12 u_m + b1 u,
        u_m' = a22 u_m + b2 u (a21 = 0). The composition of D tick_map's (closed_loop steps tick by tick)."""
        w_um, w_u, um_um, um_u = self.tick_map(inertia, tau)  # coefficients of the one-tick map
        a12, b1, a22, b2 = 0.0, 0.0, 1.0, 0.0    # accumulated map, identity
        for _ in range(self.divisor):
            a12, b1 = a12 + w_um * a22, b1 + w_um * b2 + w_u
            a22, b2 = um_um * a22, um_um * b2 + um_u
        return a12, b1, a22, b2


def closed_loop(su, axis, inertia, tau, n_exec, sp, inject=None, want_rho=False):
    """omega at each execution (index n - 1). inject = (node, k) for a node of the law, (node, k, tick) for a chain node
    (TICK_NODES; tick 0 .. D - 1 of execution k): a unit injection there, with sp = 0 (the impulse response). want_rho
    also returns, per node, the maximum first-order rounding injection."""
    p = su.p
    kp, ki, kd = p[f"rate_kp_{axis}"], p[f"rate_ki_{axis}"], p[f"rate_kd_{axis}"]
    tau_ref = p[f"rate_tau_ref_{axis}"]
    t_f = p[f"rate_d_filter_tau_{axis}"]
    a12, b1, a22, b2 = su.tick_map(inertia, tau)
    c0, c1, c2, c3, c4 = su.lowpass         # b0, b1, b2, a1, a2
    dc0, dc1, dc2, dc3, dc4 = su.lowpass_error
    w = um = 0.0
    x1 = x2 = f1 = f2 = 0.0                 # the low-pass's direct-form-I history
    r = integral = e_prev = y_prev = d_f = u = 0.0
    y_out = []
    rho = dict.fromkeys(NODES, 0.0)
    for n in range(1, n_exec + 1):
        y_out.append(w)
        for tick in range(su.divisor):
            x = w
            if inject and inject == ("x", n, tick):
                x += 1.0
            if n == 1 and tick == 0:
                x1 = x2 = f1 = f2 = x       # seed_first_sample: the steady state of the first sample
            p0, p1, p2, p3, p4 = c0 * x, c1 * x1, c2 * x2, c3 * f1, c4 * f2
            s1 = p0 + p1
            s2 = s1 + p2
            s3 = s2 - p3
            f = s3 - p4
            if want_rho:
                rho["x"] = max(rho["x"], UNIT_ROUNDOFF * abs(w))
                rho["f"] = max(rho["f"], UNIT_ROUNDOFF * (abs(p0) + abs(p1) + abs(p2) + abs(p3) + abs(p4)
                                                          + abs(s1) + abs(s2) + abs(s3) + abs(f))
                               + dc0 * abs(x) + dc1 * abs(x1) + dc2 * abs(x2) + dc3 * abs(f1) + dc4 * abs(f2))
            if inject and inject == ("f", n, tick):
                f += 1.0
            x2, x1, f2, f1 = x1, x, f1, f
            if tick == 0:
                y = f
                if n == 1:
                    r, e_prev, integral, d_f, u = y, 0.0, 0.0, 0.0, 0.0
                else:
                    dt = su.dt(n)
                    xr = dt / tau_ref
                    alpha = -math.expm1(-xr)
                    r_prev = r
                    r = r + alpha * (sp - r)
                    if inject and inject == ("r", n):
                        r += 1.0
                    e = r - y
                    if inject and inject == ("e", n):
                        e += 1.0
                    i_prev_term = ki * e_prev * dt
                    integral = integral + i_prev_term
                    if inject and inject == ("I", n):
                        integral += 1.0
                    d = -kd * (y - y_prev) / dt
                    d_prev = d_f
                    if t_f > 0.0:
                        xd = dt / t_f
                        alpha_d = -math.expm1(-xd)
                        d_f = d_f + alpha_d * (d - d_f)
                    else:
                        d_f = d
                    if inject and inject == ("D", n):
                        d_f += 1.0
                    pi_sum = kp * e + integral
                    u = pi_sum + d_f
                    if inject and inject == ("u", n):
                        u += 1.0
                    if want_rho:
                        d_alpha = math.exp(-xr) * xr * 2 * UNIT_ROUNDOFF + ulp32(math.exp(-xr)) + UNIT_ROUNDOFF * alpha
                        s = abs(sp - r_prev)
                        rho["r"] = max(rho["r"], d_alpha * s + 2 * UNIT_ROUNDOFF * alpha * s + UNIT_ROUNDOFF * abs(r))
                        rho["e"] = max(rho["e"], UNIT_ROUNDOFF * abs(e))
                        rho["I"] = max(rho["I"], 3 * UNIT_ROUNDOFF * abs(i_prev_term) + UNIT_ROUNDOFF * abs(integral))
                        rho_d = 4 * UNIT_ROUNDOFF * abs(d)
                        if t_f > 0.0:
                            d_alpha_d = (math.exp(-xd) * xd * 2 * UNIT_ROUNDOFF + ulp32(math.exp(-xd))
                                         + UNIT_ROUNDOFF * alpha_d)
                            s_d = abs(d - d_prev)
                            rho_d = (alpha_d * rho_d + d_alpha_d * s_d + 2 * UNIT_ROUNDOFF * alpha_d * s_d
                                     + UNIT_ROUNDOFF * abs(d_f))
                        rho["D"] = max(rho["D"], rho_d)
                        rho["u"] = max(rho["u"], UNIT_ROUNDOFF * (abs(kp * e) + abs(pi_sum) + abs(u)))
                    e_prev = e
                y_prev = y
            w, um = w + a12 * um + b1 * u, a22 * um + b2 * u
    return (y_out, rho) if want_rho else y_out


def l1_norm(su, axis, node, k, n_exec):
    """Sum over the responses at executions k + 1 .. N of |omega| to the unit injections at `node` in execution k: one
    for a node of the law; one per tick for a chain node (D separate injections, their norms added)."""
    inertia, tau = su.p[INERTIA[axis]], su.p["motor_tau"]
    where = [(node, k, tick) for tick in range(su.divisor)] if node in TICK_NODES else [(node, k)]
    total = 0.0
    for at in where:
        g = closed_loop(su, axis, inertia, tau, n_exec, 0.0, inject=at)
        total += sum(abs(v) for v in g[k:])
    return total


def band_envelope(su, axis, n_exec, sp, points):
    inertia, tau = su.p[INERTIA[axis]], su.p["motor_tau"]
    lo = [math.inf] * n_exec
    hi = [-math.inf] * n_exec
    for i in range(points):
        for j in range(points):
            s = -1.0 + 2.0 * i / (points - 1)
            t = -1.0 + 2.0 * j / (points - 1)
            y = closed_loop(su, axis, inertia * (1 + s * su.j_band), tau * (1 + t * su.tau_band), n_exec, sp)
            lo = [min(a, b) for a, b in zip(lo, y)]
            hi = [max(a, b) for a, b in zip(hi, y)]
    return lo, hi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=HERE, help="directory of rate_t3_inputs.txt and of the outputs")
    ap.add_argument("--refresh-inputs", metavar="PARAM_DEFAULTS_CPP")
    args = ap.parse_args()
    inputs_path = os.path.join(args.dir, "rate_t3_inputs.txt")
    if args.refresh_inputs:
        refresh_inputs(args.refresh_inputs, inputs_path)
        return
    p = read_inputs(inputs_path)
    su = Setup(p)
    if not (su.cutoff_hz > 0.0 and 2.0 * su.cutoff_hz * su.tick_s < 1.0):
        sys.exit("gyro_lpf_cutoff_hz is not in (0, f_s / 2) (the chain's validate)")
    for axis in AXES:
        if not p[f"rate_d_filter_tau_{axis}"] >= 0.0:
            sys.exit(f"rate_d_filter_tau_{axis} is negative (the rate loop's validate)")
    golden = [
        "# T3 golden of the L4 rate loop with the gyro chain's low-pass and the D term (decisions 0005 'T3', 0014).",
        "# Written by rate_t3_oracle.py; see README.md.",
        f"# period_us {su.period_s / MICROSECOND!r}  tick_us {su.tick_s / MICROSECOND!r}  unit_roundoff 2^-24",
        "# chain low-pass b0 b1 b2 a1 a2 (double): " + " ".join(repr(c) for c in su.lowpass),
        "# their f32 error bounds (first order): " + " ".join(repr(c) for c in su.lowpass_error),
        "# per axis: the trajectory y_n (rad/s) at each execution, the l1 norms of the impulse responses from the seven",
        "# rounding nodes to y, the largest rounding injection per node, and the tolerance sum(l1 * rho).",
    ]
    envelope = [
        "# T3 band envelope of the L4 rate loop (decision 0005 'QF-2'): pointwise min / max of the design-model step",
        "# responses over the tau x J band box, f32 gains, same prefilter, chain low-pass and D low-pass. Written by",
        "# rate_t3_oracle.py; see README.md.",
    ]
    for axis in AXES:
        n_exec = su.executions(axis)
        sp = p[f"rate_max_{axis}"]
        inertia, tau = p[INERTIA[axis]], p["motor_tau"]
        y, rho = closed_loop(su, axis, inertia, tau, n_exec, sp, want_rho=True)
        l1 = {node: max(l1_norm(su, axis, node, k, n_exec) for k in (2, 3)) for node in NODES}
        tol = sum(l1[node] * rho[node] for node in NODES)
        golden.append(f"axis {axis}")
        golden.append(f"executions {n_exec}")
        golden.append(f"setpoint {sp.hex()}")
        for node in NODES:
            golden.append(f"l1_{node} {l1[node]!r}")
        for node in NODES:
            golden.append(f"rho_{node} {rho[node]!r}")
        golden.append(f"tolerance {tol!r}")
        golden.append("trajectory")
        golden.extend(f"{v!r}" for v in y)

        lo9, hi9 = band_envelope(su, axis, n_exec, sp, GRID)
        lo17, hi17 = band_envelope(su, axis, n_exec, sp, 2 * (GRID - 1) + 1)
        change = max(max(abs(a - b) for a, b in zip(lo9, lo17)), max(abs(a - b) for a, b in zip(hi9, hi17)))
        envelope.append(f"axis {axis}")
        envelope.append(f"executions {n_exec}")
        envelope.append(f"halving_max_change {change!r}")
        envelope.append("envelope_min_max")
        envelope.extend(f"{a:.12g} {b:.12g}" for a, b in zip(lo17, hi17))
    for name, lines in (("rate_t3_golden.txt", golden), ("rate_t3_envelope.txt", envelope)):
        with open(os.path.join(args.dir, name), "w") as f:
            f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
