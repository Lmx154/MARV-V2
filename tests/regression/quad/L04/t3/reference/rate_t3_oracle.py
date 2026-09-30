#!/usr/bin/env python3
"""Independent double-precision oracle of the L4 T3 suite (quad spec 4 L4, decision 0005 "T3").

Frozen path: tests/regression/quad/L04/t3/reference/rate_t3_oracle.py. Plain Python math, no numpy, no code shared with
the C++ rate loop. Reads rate_t3_inputs.txt (the f32 parameter values, beside this script) and writes
rate_t3_golden.txt and rate_t3_envelope.txt beside itself (or into --dir):

    uv run python tests/regression/quad/L04/t3/reference/rate_t3_oracle.py
    uv run python tests/regression/quad/L04/t3/reference/rate_t3_oracle.py --refresh-inputs \
        build/<preset>/generated/marv_params/marv/params/param_defaults.cpp

--refresh-inputs rewrites rate_t3_inputs.txt from a generated parameter table (the product set) and stops. The file is
the T3 fixture; the golden and the envelope are functions of it.

What it computes, per axis a in (roll, pitch, yaw), in double:

  Law (the firmware's, decision 0005 "PID law", kd = 0, no anti-windup, mixer out of the loop). Executions n = 1..N at
  tick index D (n - 1), D = rate_loop_divisor, with stamp t_n = floor(tick * num / den) microseconds, dt_n = t_n -
  t_(n-1) in seconds (the stamps alternate 312 / 313 us at 312.5 us), y_n the plant omega at that tick:
      n = 1:  r = y, I = 0, e = 0, u = 0
      n > 1:  alpha = 1 - exp(-dt_n / tau_ref);  r += alpha (sp - r);  e_n = r - y_n;  I += ki e_(n-1) dt_n;
              u_n = kp e_n + I
  with the f32 parameter values as doubles. The setpoint is 0 -> rate_max_a at execution 1 (seed: r = y = 0).
  Plant (design model): J w' = u_m, tau u_m' = u - u_m, u held over the D ticks of an execution (zero computation
  delay), advanced exactly: the composition of D per-tick exact ZOH maps (h = num / den us) is the exact ZOH map of
  D h, applied per execution.

  Golden: y_n = omega at each execution, nominal J = inertia_<a>, tau = motor_tau.
  Horizon: N = 1 + ceil(HORIZON_TAUS * tau_ref_a / T), T = D num / den us (a stated rule: ten reference time constants).

  Rounding bound (README, first order): with U = 2^-24 (binary32 unit roundoff, round to nearest; epsilon = 2^-23) the
  float law's local rounding errors are injected at four nodes (r, e, I, u) with the magnitudes RHO_* below, evaluated on
  the double trajectory; each propagates to y through the closed-loop impulse response g_node from that node, whose
  l1 norm (sum over a horizon-long response, maximum over the two dt phases) this script computes. The tolerance is
      TOL_a = l1_r max(rho_r) + l1_e max(rho_e) + l1_I max(rho_I) + l1_u max(rho_u).

  Envelope: pointwise min / max over the tau x J band box (J (1 +- inertia_robustness_band), tau (1 +- tau_robustness_band))
  of the design-model step responses with the same prefilter and f32 gains: the corners plus a 9 x 9 grid, then one
  halving (17 x 17); the largest change of any envelope point between the two is recorded.
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
NODES = ("r", "e", "I", "u")

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
    "rate_tau_ref_roll", "rate_tau_ref_pitch", "rate_tau_ref_yaw",
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
        out.write(f"# parameters of 2026-09-30 (rate_t3_oracle.py --refresh-inputs from {os.path.basename(defaults_cpp)});\n")
        out.write("# the T3 test reads this file, not the live parameters, so it does not follow later parameter changes.\n")
        for key in INPUT_KEYS:
            v = table[key]
            out.write(f"{key} {v}\n" if key in INT_KEYS else f"{key} {v.hex()}  # {v:.9g}\n")


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

    def stamp_us(self, n):
        """Sample stamp of execution n (1-based): floor(tick * num / den) microseconds, tick = D (n - 1)."""
        return (self.divisor * (n - 1) * self.num) // self.den

    def dt(self, n):
        return (self.stamp_us(n) - self.stamp_us(n - 1)) * MICROSECOND

    def executions(self, axis):
        return 1 + math.ceil(HORIZON_TAUS * self.p[f"rate_tau_ref_{axis}"] / self.period_s)

    def plant_map(self, inertia, tau):
        """(a11, a12, b1, a21, a22, b2): state (w, u_m) after D ticks of input u:  w' = w + a12 u_m + b1 u,
        u_m' = a22 u_m + b2 u (a21 = 0)."""
        h = self.tick_s
        x = h / tau
        e = math.exp(-x)
        em = -math.expm1(-x)
        # One tick: u_m' = e u_m + em u ;  w' = w + (tau em / J) u_m + ((h - tau em) / J) u.
        t12, t1 = tau * em / inertia, (h - tau * em) / inertia
        w_um, w_u, um_um, um_u = t12, t1, e, em  # coefficients of the one-tick map
        a12, b1, a22, b2 = 0.0, 0.0, 1.0, 0.0    # accumulated map, identity
        for _ in range(self.divisor):
            a12, b1 = a12 + w_um * a22, b1 + w_um * b2 + w_u
            a22, b2 = um_um * a22, um_um * b2 + um_u
        return a12, b1, a22, b2


def closed_loop(su, axis, inertia, tau, n_exec, sp, inject=None, want_rho=False):
    """omega at each execution (index n - 1). inject = (node, k): a unit injection at that node of execution k, with
    sp = 0 (the impulse response). want_rho also returns, per node, the maximum first-order rounding injection."""
    p = su.p
    kp, ki, kd = p[f"rate_kp_{axis}"], p[f"rate_ki_{axis}"], p[f"rate_kd_{axis}"]
    tau_ref = p[f"rate_tau_ref_{axis}"]
    a12, b1, a22, b2 = su.plant_map(inertia, tau)
    w = um = 0.0
    r = integral = e_prev = 0.0
    y_out = []
    rho = dict.fromkeys(NODES, 0.0)
    for n in range(1, n_exec + 1):
        y = w
        y_out.append(y)
        if n == 1:
            r, e_prev, integral, u = y, 0.0, 0.0, 0.0
        else:
            dt = su.dt(n)
            x = dt / tau_ref
            alpha = -math.expm1(-x)
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
            u = kp * e + integral
            if inject and inject == ("u", n):
                u += 1.0
            if want_rho:
                d_alpha = math.exp(-x) * x * 2 * UNIT_ROUNDOFF + ulp32(math.exp(-x)) + UNIT_ROUNDOFF * alpha
                s = abs(sp - r_prev)
                rho["r"] = max(rho["r"], d_alpha * s + 2 * UNIT_ROUNDOFF * alpha * s + UNIT_ROUNDOFF * abs(r))
                rho["e"] = max(rho["e"], UNIT_ROUNDOFF * (abs(y) + abs(e)))
                rho["I"] = max(rho["I"], 3 * UNIT_ROUNDOFF * abs(i_prev_term) + UNIT_ROUNDOFF * abs(integral))
                rho["u"] = max(rho["u"], UNIT_ROUNDOFF * abs(kp * e) + UNIT_ROUNDOFF * abs(u))
            e_prev = e
        w, um = w + a12 * um + b1 * u, a22 * um + b2 * u
    return (y_out, rho) if want_rho else y_out


def l1_norm(su, axis, node, k, n_exec):
    inertia, tau = su.p[INERTIA[axis]], su.p["motor_tau"]
    g = closed_loop(su, axis, inertia, tau, n_exec, 0.0, inject=(node, k))
    return sum(abs(v) for v in g[k:])  # responses at executions k + 1 .. N


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
    for axis in AXES:
        if p[f"rate_kd_{axis}"] != 0.0:
            sys.exit("the rounding derivation assumes kd = 0 (decision 0005, owner decision 5)")
    su = Setup(p)
    golden = [
        "# T3 golden of the L4 rate loop (decision 0005 'T3'). Written by rate_t3_oracle.py; see README.md.",
        f"# period_us {su.period_s / MICROSECOND!r}  tick_us {su.tick_s / MICROSECOND!r}  unit_roundoff 2^-24",
        "# per axis: the trajectory y_n (rad/s) at each execution, the l1 norms of the impulse responses from the four",
        "# rounding nodes to y, the largest rounding injection per node, and the tolerance sum(l1 * rho).",
    ]
    envelope = [
        "# T3 band envelope of the L4 rate loop (decision 0005 'QF-2'): pointwise min / max of the design-model step",
        "# responses over the tau x J band box, f32 gains, same prefilter. Written by rate_t3_oracle.py; see README.md.",
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
