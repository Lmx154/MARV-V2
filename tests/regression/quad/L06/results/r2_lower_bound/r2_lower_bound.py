#!/usr/bin/env python3
"""Lower bound on the R2 rate excess forced by the motor lag (decision 0014, owner decision 5, condition 1).

    uv run python tests/regression/quad/L06/results/r2_lower_bound/r2_lower_bound.py --out <file>

Claim. R2 as specified before its steady-tumble setup (decision 0015), the hover-rotor tumble
scenarios/quad/L05/recover_tumble_prop_strike.yaml, starts at the body rates w0 with every rotor at the card's hover speed,
so the rotor torque is zero at t = 0 while the coupling w x Jw is not. No command history, so no controller, can keep the
body rate inside the R2 predicate's design tolerance F + Q (tests/regression/quad/L05/gz/test_t4_recovery.py) on the
binding axis.

Plant (the model of sim/plant/src/plant_model.hpp and marv_plant.cpp; rigid body: Euler's equation).
  J w_a' = m_a - c_a(w),  c = w x Jw = ((J_z - J_y) w_y w_z, (J_x - J_z) w_z w_x, (J_y - J_x) w_x w_y), J the card's
  diagonal; m_a = sum_i B[a, i] k W_i^2, B the card's effectiveness matrix (tools/card/mixer.py: the plant's forward map);
  W_i' = (u_i - W_i) / tau, W_i(0) the scenario's rotor speeds, u_i the ESC command: 0 (DShot 0) or a speed in
  [W_min, W_max] (plant_model.hpp omega_cmd).

Derivation, per axis a, with s_a = sign(-c_a(w0)) the direction the coupling pushes w_a.
  1. Reachable rotor speeds. W_i(t) = W_i(0) e^(-t/tau) + int_0^t e^(-(t-r)/tau) u_i(r) dr / tau is increasing in u_i, so
     for every command history in the hull [0, W_max] of the command set
         lo_i(t) = W_i(0) e^(-t/tau)  <=  W_i(t)  <=  W_max - (W_max - W_i(0)) e^(-t/tau) = hi_i(t).
     The plant's sub-steps are the exact ZOH solution, so they sample this bound.
  2. Reachable torque. Each term B[a, i] k W_i^2 is monotone in W_i >= 0, so m_a(t) lies in [mn_a(t), mx_a(t)], the sums of
     the per-rotor extremes at lo_i(t) and hi_i(t). mn_a is nonincreasing and mx_a nondecreasing in t.
  3. Host step. marv_plant_step advances the motors over the host step and then computes the wrench, which gz applies
     over that step: the torque at time r is the reachable torque at most H later, H the longest host step of the
     scenario's m_sequence. So s_a m_a(r) >= P_a(r + H), P_a = mn_a (s_a = +1) or -mx_a (s_a = -1). This covers the
     continuous model as well.
  4. Coupling. An a-priori box |w_b(r) - w0_b| <= R_b(r) over the window [0, t_end]: R_glob solves R = Phi(R),
     Phi_b(R) = int_0^t_end (Mag_b + Cmax_b(R)) / J_b, Mag_b(r) the largest of |mn_b|, |mx_b| on [r, r + H] and Cmax_b(R)
     the largest |c_b| on the box; Phi(R) <= R is checked, and the integrand is positive, so w cannot leave the box before
     t_end. R_b(r) = min(R_glob_b, int_0^r (Mag_b + Cmax_b(R_glob)) / J_b). D_a(r) = the smallest s_a (-c_a) on the box of
     radius R(r) (c_a is bilinear: a corner).
  5. Integration. s_a (w_a(t) - w0_a) >= L_a(t) = int_0^t (D_a(r) + P_a(r + H)) dr / J_a. The integrand is nonincreasing,
     so the right-endpoint sum on the quadrature grid is a lower sum: L_a is a lower bound at every grid time.
  6. Against the predicate. The run's m = 1 rate channel y(n) at attitude execution n (t_n = n T_a) violates the R2
     predicate unless s_a (y(n) - env(n)) <= E(n) + F + Q, env = hi (s_a = +1) or lo (s_a = -1) of the design envelope
     and F, Q the test's own terms (test_t4_recovery.make_design). By 5, s_a (y(n) - env(n)) >= X_a(n) = L_a(t_n) -
     s_a (env(n) - w0_a). The claim on axis a: max_n X_a(n) > F_a + Q_a. E(n) = |y_m=1(n) - y_m=2(n)| is a measurement of
     the run; the predicate can hold at the binding execution only if E there is at least X - (F + Q).

Nothing in steps 1 to 5 uses a gain: every command history in the hull is admitted. Steps 1 to 5 only ever enlarge the
actuator (the command hull, the host-step shift) or shrink the coupling (the box), so L_a under-estimates the departure.

Inputs, all read from committed files: the card (gen_plant_config.load_linted, config_from; inertia_diag; mixer.mixer_matrix),
the scenario (l5_scenario; run_l5.hover_rotor_speeds), the recorded T3 fixture and the design envelope
(test_t4_recovery.make_design), and the committed R2 replay (tests/regression/quad/L05/results/recovery_cause/cause.txt) for
the cross-check. Negative controls: (NC1) the rotors at the steady-tumble speeds (collective plus the mixer inverse of
w0 x Jw0), where the net torque at t = 0 is zero; at the hover collective that rule gives a motor a negative thrust, so the
control uses the smallest collective that keeps every rotor at or above W_min (a value for this control only); (NC2) equal
principal moments, where w x Jw = 0. Both must collapse the bound to at most F + Q.
"""
import argparse
import hashlib
import itertools
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
for _p in (ROOT / "tools" / "card", ROOT / "tools" / "sim", ROOT / "tests" / "regression" / "quad" / "L05" / "gz"):
    sys.path.insert(0, str(_p))
import gen_plant_config as gpc  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import mixer  # noqa: E402
import recovery_model as rm  # noqa: E402
import run_l4  # noqa: E402
import run_l5  # noqa: E402
import test_t4_recovery as t4r  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN_NAME = "recover_tumble_prop_strike"
SCEN = ROOT / "scenarios" / "quad" / "L05" / f"{SCEN_NAME}.yaml"
CAUSE = ROOT / "tests" / "regression" / "quad" / "L05" / "results" / "recovery_cause" / "cause.txt"
T3_INPUTS = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference" / "attitude_t3_inputs.txt"
AXES = ("roll", "pitch", "yaw")
RATE_CH = ("w_x", "w_y", "w_z")
# Numerical choices (labelled; neither changes the validity of the bound, only its tightness):
QUAD_PER_TICK = 16  # quadrature steps per tick: the right-endpoint sum is a lower sum at any step
WINDOW_EXEC = 32  # attitude executions in the window [0, t_end]; the script refuses if a maximum sits at its end
MAX_ITER = 200  # cap on the fixed-point iteration of the a-priori box; the script refuses if Phi(R) <= R is not reached
REPORT_EXEC = (1, 4, 8, 16, 24, 32)  # executions printed per axis
CAUSE_SIG_DIGITS = 5  # significant digits of cause.txt floats (recovery_cause.py:141, fmt: f"{x:.5g}")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def g(x):
    return f"{x:.5g}"


def gl(xs):
    return "[" + ", ".join(g(x) for x in xs) + "]"


def coupling(j, w):
    """w x Jw for diagonal J, in the difference form (exactly 0 for equal moments)."""
    return ((j[2] - j[1]) * w[1] * w[2], (j[0] - j[2]) * w[2] * w[0], (j[1] - j[0]) * w[0] * w[1])


def setup():
    card_doc, profile = gpc.load_linted(CARD)
    cfg, _ = gpc.config_from(card_doc, profile, CARD)
    m, b, _ = mixer.mixer_matrix(card_doc, CARD)
    vals = l5s.values(l5s.load(SCEN))
    st = vals["initial_state"]
    if st["rotor_speed_rad_s"] != "hover":
        raise SystemExit(f"{SCEN}: initial rotor speed {st['rotor_speed_rad_s']!r} is not 'hover': this proof is about R2 as specified")
    su = rm.oracle.Setup(t4r.recorded_inputs())
    if (su.num, su.den) != (vals["tick_period_num_us"], vals["tick_period_den"]):
        raise SystemExit("the scenario's tick differs from the recorded T3 fixture")
    return {
        "J": [float(x) for x in card_doc["inertia_diag"]["value"]],
        "k": cfg["thrust_coeff"], "w_min": cfg["omega_min_rad_s"], "w_max": cfg["omega_max_rad_s"], "tau": cfg["motor_tau_s"],
        "M": m, "B": b, "w0": [float(x) for x in st["body_rates_frd_rad_s"]], "vals": vals,
        "hover_thrust": run_l4.hover_thrust(CARD, vals), "rotors_hover": list(run_l5.hover_rotor_speeds(CARD, vals)),
        "tick": su.tick_s, "t_a": su.t_a, "H": max(vals["m_sequence"]) * su.tick_s,
        "ticks_per_exec": su.divisor * su.ratio,
    }


def torque_range(c, rotors, t):
    """([mn_a], [mx_a]) of the reachable torque at time t (derivation steps 1 and 2)."""
    e = math.exp(-t / c["tau"])
    mn, mx = [0.0] * 3, [0.0] * 3
    for i, r0 in enumerate(rotors):
        lo = r0 * e
        hi = c["w_max"] - (c["w_max"] - r0) * e
        for a in range(3):
            coef = c["B"][a + 1][i] * c["k"]
            p, q = coef * lo * lo, coef * hi * hi
            mn[a] += min(p, q)
            mx[a] += max(p, q)
    return mn, mx


def box_corners(w0, r):
    return [tuple(w0[b] + s[b] * r[b] for b in range(3)) for s in itertools.product((-1.0, 1.0), repeat=3)]


def bound(c, j, rotors, sigma):
    """Per axis: L(t_j) on the quadrature grid, the a-priori box and its check (derivation steps 1 to 5)."""
    h = c["tick"] / QUAD_PER_TICK
    per_exec = c["ticks_per_exec"] * QUAD_PER_TICK
    n = WINDOW_EXEC * per_exec
    w0, H = c["w0"], c["H"]
    ranges = [torque_range(c, rotors, k * h + H) for k in range(n + 1)]
    mag = [[max(abs(mn[b]), abs(mx[b])) for b in range(3)] for mn, mx in ranges]
    mag0 = [[max(abs(mn[b]), abs(mx[b])) for b in range(3)] for mn, mx in (torque_range(c, rotors, k * h) for k in range(n))]
    # |m_b| on step k is at most Mag on [t_(k-1), t_k + H]; mn, mx are monotone, so that is the larger endpoint value
    mag_step = [[max(mag0[k - 1][b], mag[k][b]) for b in range(3)] for k in range(1, n + 1)]

    def cmax(r):
        return [max(abs(coupling(j, w)[b]) for w in box_corners(w0, r)) for b in range(3)]

    def phi(r):
        cm = cmax(r)
        return [sum(h * (ms[b] + cm[b]) / j[b] for ms in mag_step) for b in range(3)]

    r = [0.0, 0.0, 0.0]
    for _ in range(MAX_ITER):
        nxt = phi(r)
        if all(x <= y for x, y in zip(nxt, r)):
            break
        r = nxt
    box_ok = all(x <= y for x, y in zip(phi(r), r))
    cm = cmax(r)
    acc = [0.0, 0.0, 0.0]
    big_l = [[0.0] for _ in range(3)]
    for k in range(1, n + 1):
        acc = [acc[b] + h * (mag_step[k - 1][b] + cm[b]) / j[b] for b in range(3)]
        rk = [min(r[b], acc[b]) for b in range(3)]
        corners = box_corners(w0, rk)
        mn, mx = ranges[k]
        for a in range(3):
            d = min(sigma[a] * -coupling(j, w)[a] for w in corners)
            p = mn[a] if sigma[a] > 0 else -mx[a]
            big_l[a].append(big_l[a][-1] + h * (d + p) / j[a])
    return {"L": big_l, "box": r, "box_ok": box_ok, "per_exec": per_exec, "h": h}


def against_design(c, res, design, sigma):
    """Per axis: X(n) for n = 1 .. WINDOW_EXEC (derivation step 6) and the predicate's design tolerance F + Q."""
    out = []
    for a in range(3):
        ch = 3 + a
        env = design.hi[ch] if sigma[a] > 0 else design.lo[ch]
        xs = {n: res["L"][a][n * res["per_exec"]] - sigma[a] * (env[n] - c["w0"][a]) for n in range(1, WINDOW_EXEC + 1)}
        n_star = max(xs, key=xs.get)
        lbest = max(range(1, len(res["L"][a])), key=lambda k: res["L"][a][k])
        out.append({"X": xs, "n_star": n_star, "X_star": xs[n_star], "F": design.F[ch], "halving": design.halving[ch], "Q": design.Q[ch],
                    "tol": design.F[ch] + design.Q[ch], "L_star": res["L"][a][lbest], "t_star": lbest * res["h"],
                    "env_moves": any(sigma[a] * (env[m] - c["w0"][a]) > 0.0 for m in range(WINDOW_EXEC + 1))})
    return out


def cause_trace():
    """{n: (w strings)} of the R2 m = 1 gz trace lines of cause.txt (section 2), and the world's initial rotor speeds."""
    text = CAUSE.read_text(encoding="utf-8")
    sec = text.split("== 2. R2 cause", 1)[1].split("== 3.", 1)[0]
    rows = {int(m.group(1)): m.group(2).split(", ")
            for m in re.finditer(r"^  exec (\d+) \(t [0-9.]+ s\): alpha \S+, err \[[^]]*\], w \[([^]]*)\]", sec, re.M)}
    rot = re.search(r"world initial_rotor_speed_rad_s ([0-9. ]+)\)", text).group(1).split()
    return rows, [float(x) for x in rot]


def half_unit(s):
    """Half a unit in the last significant digit of s as cause.txt prints it: f"{x:.5g}" (recovery_cause.py:141), which
    drops trailing zeros, so the digit count comes from the format, not from the string."""
    y = abs(float(s))
    return 0.5 * 10.0 ** (math.floor(math.log10(y)) - (CAUSE_SIG_DIGITS - 1))


def compute():
    c = setup()
    j = c["J"]
    w0 = c["w0"]
    cw0 = coupling(j, w0)
    sigma = [1.0 if -x > 0 else -1.0 for x in cw0]
    net0 = torque_range(c, c["rotors_hover"], 0.0)[0]
    design = t4r.make_design(SCEN_NAME, {})
    res = bound(c, j, c["rotors_hover"], sigma)
    ax = against_design(c, res, design, sigma)
    for a in range(3):
        if ax[a]["n_star"] == WINDOW_EXEC or ax[a]["t_star"] >= WINDOW_EXEC * c["t_a"]:
            raise SystemExit(f"{AXES[a]}: the maximum sits at the window's end: widen WINDOW_EXEC")
    binding = max(range(3), key=lambda a: ax[a]["X_star"] - ax[a]["tol"])

    rows, rot_cause = cause_trace()
    cross = []
    for n, ws in sorted(rows.items()):
        if 1 <= n <= WINDOW_EXEC:
            for a in range(3):
                y, hu = float(ws[a]), half_unit(ws[a])
                env = design.hi[3 + a][n] if sigma[a] > 0 else design.lo[3 + a][n]
                cross.append({"n": n, "a": a, "y": ws[a], "dep": sigma[a] * (y - w0[a]) - hu, "L": res["L"][a][n * res["per_exec"]],
                              "exc": sigma[a] * (y - env) - hu, "X": ax[a]["X"][n]})

    # the architect's form g^2 tau / (2 J Delta), Delta = 2 tau_max, and the largest reachable slew over the window
    t_end = WINDOW_EXEC * c["t_a"] + c["H"]
    arch = []
    for a in range(3):
        coefs = [abs(c["B"][a + 1][i]) * c["k"] for i in range(len(c["rotors_hover"]))]
        tau_max = 0.5 * sum(cf * (c["w_max"] ** 2 - c["w_min"] ** 2) for cf in coefs)
        e = math.exp(-t_end / c["tau"])
        slew = 0.0
        for cf, r0 in zip(coefs, c["rotors_hover"]):
            hi = c["w_max"] - (c["w_max"] - r0) * e
            slew += 2.0 * cf * hi * max(hi, c["w_max"] - hi) / c["tau"]
        arch.append({"tau_max": tau_max, "slew_arch": 2.0 * tau_max / c["tau"], "slew_win": slew,
                     "L": cw0[a] ** 2 * c["tau"] / (2.0 * j[a] * 2.0 * tau_max)})

    # NC1: steady-tumble rotor speeds, T = M (F, w0 x Jw0)
    m = c["M"]
    t_min, t_max = c["k"] * c["w_min"] ** 2, c["k"] * c["w_max"] ** 2

    def thrusts(f):
        return [m[i][0] * f + sum(m[i][a + 1] * cw0[a] for a in range(3)) for i in range(len(m))]

    t_rule = thrusts(c["hover_thrust"])
    f_real = max((t_min - sum(m[i][a + 1] * cw0[a] for a in range(3))) / m[i][0] for i in range(len(m)))
    t_real = thrusts(f_real)
    nc1 = {"t_rule": t_rule, "rule_ok": all(t_min <= x <= t_max for x in t_rule), "f_real": f_real, "t_real": t_real,
           "real_ok": max(t_real) <= t_max}
    rotors_st = [math.sqrt(max(x, t_min) / c["k"]) for x in t_real]  # the binding motor sits at W_min (max: its rounding)
    nc1["rotors"] = rotors_st
    nc1["net0"] = [mn - cc for mn, cc in zip(torque_range(c, rotors_st, 0.0)[0], cw0)]
    nc1["res"] = bound(c, j, rotors_st, sigma)
    nc1["ax"] = against_design(c, nc1["res"], design, sigma)

    # NC2: equal principal moments (the mean of the card's diagonal)
    j_eq = [sum(j) / len(j)] * 3
    nc2 = {"J": j_eq, "cw0": coupling(j_eq, w0)}
    nc2["res"] = bound(c, j_eq, c["rotors_hover"], sigma)
    nc2["ax"] = against_design(c, nc2["res"], design, sigma)
    return {"c": c, "cw0": cw0, "sigma": sigma, "net0": net0, "design": design, "res": res, "ax": ax, "binding": binding,
            "cross": cross, "rot_cause": rot_cause, "arch": arch, "nc1": nc1, "nc2": nc2}


def render(r):
    c, ax, res = r["c"], r["ax"], r["res"]
    b = r["binding"]
    L = ["MARV quad L6 stage (c): lower bound on the R2 rate excess forced by the motor lag (decision 0014, owner decision 5, "
         "condition 1)",
         "command: uv run python tests/regression/quad/L06/results/r2_lower_bound/r2_lower_bound.py --out <file>",
         f"inputs sha256: card {sha(CARD)}; scenario {sha(SCEN)}; test_t4_recovery.py {sha(t4r.__file__)}; recovery_model.py "
         f"{sha(rm.__file__)}; attitude_t3_inputs.txt {sha(T3_INPUTS)}; cause.txt {sha(CAUSE)}",
         "",
         "== 0. inputs",
         f"card: J diag {gl(c['J'])} kg m^2; k {g(c['k'])} N/(rad/s)^2; motor tau {g(c['tau'])} s; speed range "
         f"[{g(c['w_min'])}, {g(c['w_max'])}] rad/s; B (roll, pitch, yaw rows, per motor thrust) "
         + "; ".join(gl(c["B"][a + 1]) for a in range(3)),
         f"scenario: w0 {gl(c['w0'])} rad/s; rotors 'hover' -> run_l5.hover_rotor_speeds {[repr(x) for x in c['rotors_hover']]} "
         f"rad/s; cause.txt world initial_rotor_speed_rad_s equal: {c['rotors_hover'] == r['rot_cause']}",
         f"timing: tick {g(c['tick'])} s; attitude period T_a {g(c['t_a'])} s; host step H = max(m_sequence) ticks = {g(c['H'])} s; "
         f"quadrature step {g(res['h'])} s; window {WINDOW_EXEC} executions = {g(WINDOW_EXEC * c['t_a'])} s",
         "",
         "== 1. coupling at t = 0",
         f"w0 x Jw0 {gl(r['cw0'])} N m; push direction s = sign(-(w0 x Jw0)) {gl(r['sigma'])}; rotor torque at t = 0 "
         f"(hover rotors, B k W^2) {gl(r['net0'])} N m",
         "",
         "== 2. lower bound L(t) on s (w(t) - w0), every command history in [0, W_max] (no gain enters)",
         f"a-priori box radius {gl(res['box'])} rad/s over the window; Phi(R) <= R: {res['box_ok']}"]
    for a in range(3):
        L.append(f"{AXES[a]:<5}: L* {g(ax[a]['L_star'])} rad/s at t* {g(ax[a]['t_star'])} s; L(t_n) at n = "
                 + ", ".join(f"{n}: {g(res['L'][a][n * res['per_exec']])}" for n in REPORT_EXEC))
    L += ["",
          "== 3. against the R2 predicate (test_t4_recovery.make_design: envelope, F, Q; E is the run's m = 1 vs m = 2 term)"]
    for a in range(3):
        x = ax[a]
        L.append(f"{RATE_CH[a]}: F {g(x['F'])} (its halving term {g(x['halving'])}) Q {g(x['Q'])} F + Q {g(x['tol'])} rad/s; "
                 f"the envelope moves past w0 in the push direction within the window: {x['env_moves']}; X* = max_n X(n) {g(x['X_star'])} rad/s at n {x['n_star']} "
                 f"(t {g(x['n_star'] * c['t_a'])} s); X*/(F + Q) {x['X_star'] / x['tol']:.3g}; E needed at n*: >= "
                 f"{g(x['X_star'] - x['tol'])} rad/s")
    L.append(f"binding axis (largest X* - (F + Q)): {RATE_CH[b]}; claim X* > F + Q: {ax[b]['X_star'] > ax[b]['tol']} (the "
             f"predicate then holds at n* only if E(n*) >= X* - (F + Q))")
    L += ["",
          "== 4. cross-check against the committed R2 replay (cause.txt section 2, gz m = 1 TRUTH; minus half a printed unit)"]
    for row in r["cross"]:
        L.append(f"exec {row['n']} {RATE_CH[row['a']]} {row['y']}: departure {g(row['dep'])} >= L {g(row['L'])}: "
                 f"{row['dep'] >= row['L']}; excess over the envelope {g(row['exc'])} >= X {g(row['X'])}: {row['exc'] >= row['X']}")
    L += ["",
          "== 5. comparison: the architect's form g^2 tau / (2 J Delta), Delta = 2 tau_max (tau_max = half the full-range "
          "torque), slew 2 tau_max / tau"]
    for a in range(3):
        x = r["arch"][a]
        L.append(f"{AXES[a]:<5}: tau_max {g(x['tau_max'])} N m; slew {g(x['slew_arch'])} N m/s against the largest "
                 f"reachable slew over the window {g(x['slew_win'])} N m/s; bound {g(x['L'])} rad/s")
    nc1, nc2 = r["nc1"], r["nc2"]
    L += ["",
          "== 6. negative controls (each must collapse the bound to at most F + Q)",
          f"NC1 steady-tumble rotors, T = M (F, w0 x Jw0). At the hover collective {g(c['hover_thrust'])} N: T {gl(nc1['t_rule'])} N "
          f"against [k W_min^2, k W_max^2] = [{g(c['k'] * c['w_min'] ** 2)}, {g(c['k'] * c['w_max'] ** 2)}] N; realisable: "
          f"{nc1['rule_ok']}",
          f"    control variant: the smallest collective with every T >= k W_min^2, {g(nc1['f_real'])} N: T {gl(nc1['t_real'])} N, "
          f"largest T <= k W_max^2: {nc1['real_ok']}; rotors {gl(nc1['rotors'])} rad/s; net torque at t = 0 {gl(nc1['net0'])} N m"]
    for name, nc in (("NC1", nc1), ("NC2", nc2)):
        if name == "NC2":
            L.append(f"NC2 equal moments J {gl(nc2['J'])} kg m^2 (the card's mean), hover rotors: w0 x Jw0 {gl(nc2['cw0'])} N m")
        for a in range(3):
            x = nc["ax"][a]
            L.append(f"    {RATE_CH[a]}: L* {g(max(0.0, x['L_star']))} rad/s; X* {g(x['X_star'])} at n {x['n_star']}; X* <= F + Q "
                     f"{g(x['tol'])}: {x['X_star'] <= x['tol']}")
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    r = compute()
    text = render(r)
    Path(a.out).write_text(text, encoding="utf-8")
    print(text, end="")
    return r


if __name__ == "__main__":
    main()
