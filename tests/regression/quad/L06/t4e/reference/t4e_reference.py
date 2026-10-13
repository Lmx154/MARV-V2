#!/usr/bin/env python3
"""Generator of the refdata set quad/L06/t4e (quad spec 4 L6 pass bar (e); decision 0019 rulings 3-6 and second-round
rulings 1-3; the noise suite design memo of 2026-10-04, F-a, sections 1 and 3, commit C6a). Generated, not committed
(decision 0011): tools/refdata/refdata.py runs it and checks every file against SHA256SUMS beside it.

    uv run python tests/regression/quad/L06/t4e/reference/t4e_reference.py --dir DIR [--procs N] [--only ID ...]
    uv run python tests/regression/quad/L06/t4e/reference/t4e_reference.py --list
    uv run python tests/regression/quad/L06/t4e/reference/t4e_reference.py --refresh-inputs [--dir DIR]

--dir holds the inputs (refdata copies them in) and receives the outputs. --list prints the output file names. --only
writes only the named files (ids of --list without .txt; the index needs every scenario). --refresh-inputs rewrites
t4e_inputs.txt from the repository (below) and stops. The output does not depend on --procs.

Inputs. t4e_inputs.txt (this directory; exact hex floats): latency_samples (the sensor profile), omega_hover
(sqrt(m G / (4 k)) of the card at standard gravity, gyro_chain_design.chain_design's rule), gyro_sigma_d, gyro_lsb and
gyro_k_rw at the nominal tick (l6_noise_term.noise_inputs: gen_imu_config's loaders), rate_ff_filter_tau (the product
parameter table tests/regression/quad/L06/product_params), noise_term_confidence and t4_pass_probability_* /
t4_confidence_* (design/budget.yaml). The L4 T3 fixture rate_t3_inputs.txt, the L5 T3 fixtures attitude_t3_inputs.txt
and attitude_t3_q_inputs.txt (their own directories; refdata copies them in). Read from the repository: the scenario files
(scenarios/quad/L04, L05), the card, the l4_rate_scripted register (acro's segment capacity), the frozen oracles and the
tools/sim models (l6_t4e_model, l6_noise_term, l6_ff_eval, run_l4, run_l5, recovery_model).

Contents, per scenario at the nominal corner (no bias, no clock error), one file each (format in write_scenario), on the
T4e linear design model (tools/sim/l6_t4e_model.py): the band per channel per checked execution (17 x 17 tau x J grid),
H (its last-halving change from the 9 x 9 grid), TOL (the oracles' first-order float32 rules on the T4e chain: L4 the L4
oracle's, L5 the L5 oracle's on the step and yaw goldens; the recoveries the frozen recovery test's T3 term, INFERRED
there and here), Q (L5: the oracle's quantiser rule; L4: none, no L4 predicate has Q), the kinematics term (L5), F = TOL +
H (+ kin at L5), and the noise term: sigma_c(n) per checked point (the largest over the grid and the frozen operating points) and its
maximum, z = Phi^-1(1 - (1 - c)/(2 N M_s)), W(n) = z sigma_c(n) per point and W = z max sigma_c, with N and M_s. Which W
the predicate uses is not decided here. The chirps (their PM channels) are UNKNOWN; R2 is provisional (its yaw saturates
at executions 1-93, where the linear model does not apply; decision 0019, the R2 package). t4e_index.txt lists every
scenario with its status and summary.

Checked points (M_s, from each frozen predicate's definition; tests/regression/quad/L04/gz, L05/gz):
  L4 step_<a>      the stepped axis at the N window executions (test_t4_steps.py): M_s = N;
  L4 acro          the three axes at the linear executions ((iii)) and the recovery window (its own test): M_s = 3 x
                   their count (l6_noise_term.l4_case's points);
  L5 step_<a>      the three components (roll, pitch, heading) at n = 0 .. 2H: M_s = 3 (2H + 1);
  L5 yaw_release   omega at n = 0 .. 2H, heading_lock from the lock, heading_release from n_r, the two at-zero items at the
                   end; the lock taken at the band's earliest (the largest count);
  L5 yaw_fallback  sigma_r w from n_r to n_fb (the band's latest fallback), omega and heading_lock from n_fb (the earliest),
                   the two at-zero items; heading_release is reported, not asserted, so not counted;
  L5 R1, R2        six channels at n = 0 .. end, the rates not at n = 0: M_s = 6 count - 3;
  L5 R1X           alpha at n = 0 .. end: M_s = count.
Noise channels: L4 the truth rate per axis; L5 theta (angle) and omega per axis of L5Lin. Derived combinations (the
standard deviation of a sum is at most the sum of the standard deviations, Minkowski): heading_release sigma(n) +
sigma(n_r), heading_lock sigma(n) + the largest sigma over the band's lock range; R1X alpha sqrt(sigma_x^2 + sigma_y^2)
(|d alpha| <= |(d err_x, d err_y)|). The L5 rows' sigma is the larger of the two frozen Jacobian ends (kp and 0, memo
section 1); each channel line also gives the largest sigma at each end alone (sigma_max_fb_hi, sigma_max_fb_0). The L5
noise is indexed by the run's execution (steps and yaw: the origin a0 + n,
run_l5.plan; the recoveries: n), the L4 steps' by the run's window execution (run_l4.plan), as the noise starts at the
run's tick 0.
"""

import argparse
import dataclasses
import math
import os
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(ROOT / "tools" / "card"))
import l6_t4e_model as m  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import run_l5  # noqa: E402

fe, nt, lo4, ao, rm = m.fe, m.nt, m.lo4, m.ao, m.rm
run_l4 = fe.run_l4
schema = fe.schema
gic = nt.gic

T4E_INPUTS = "t4e_inputs.txt"
L4_FIXTURE = "rate_t3_inputs.txt"
L5_FIXTURE = "attitude_t3_inputs.txt"
L5_Q_FIXTURE = "attitude_t3_q_inputs.txt"
INDEX = "t4e_index"
PRODUCT_TABLE = ROOT / "tests" / "regression" / "quad" / "L06" / "product_params" / "marv_params_product_table.txt"
L4_SCEN = ROOT / "scenarios" / "quad" / "L04"
L5_SCEN = ROOT / "scenarios" / "quad" / "L05"
INT_KEYS = ("latency_samples",)
BUDGET_KEYS = ("noise_term_confidence", "t4_pass_probability_tracking", "t4_confidence_tracking",
               "t4_pass_probability_safety", "t4_confidence_safety")
T4E_KEYS = ("latency_samples", "omega_hover", "gyro_sigma_d", "gyro_lsb", "gyro_k_rw", "rate_ff_filter_tau",
            *BUDGET_KEYS)
AXES = ("roll", "pitch", "yaw")
COMPONENTS = ("roll", "pitch", "heading")

L4_STEPS = ("step_roll", "step_pitch", "step_yaw")
L4_CHIRPS = ("chirp_roll", "chirp_pitch", "chirp_yaw")
L5_STEPS = ("step_roll", "step_pitch")
L5_YAW = ("yaw_release", "yaw_fallback")
L5_CHIRPS = ("chirp_roll", "chirp_pitch", "chirp_yaw")
R1, R2, R1X = "recover_inverted", "recover_tumble", "recover_inverted_exact"
L5_RECOVERIES = (R1, R2, R1X)
SCENARIOS = ([("L4", s) for s in (*L4_STEPS, *L4_CHIRPS, "acro")]
             + [("L5", s) for s in (*L5_STEPS, *L5_YAW, *L5_CHIRPS, *L5_RECOVERIES)])
CLASS = {"acro": nt.CLASS_SAFETY, R1: nt.CLASS_SAFETY, R2: nt.CLASS_SAFETY, R1X: nt.CLASS_SAFETY}  # else tracking
CHIRP_UNKNOWN = ("UNKNOWN: the PM channels (sigma_PM through run_l4.float_input_term's propagation with l2 and the "
                 "truth-noise variance, PM_design and U on the T4e chain, the L4 identification's z^-1) are not derived "
                 "in C6a")
R2_PROVISIONAL = ("provisional: R2's yaw saturates at executions 1-93, where the linear model does not apply (decision "
                  "0019, the R2 package); the set holds what the linear model gives")


def scenario_id(level, name):
    return f"{level.lower()}_{name}"


def output_names():
    return [f"{INDEX}.txt"] + [f"{scenario_id(lv, s)}.txt" for lv, s in SCENARIOS if s not in L4_CHIRPS + L5_CHIRPS]


def r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


# ---- the inputs ---------------------------------------------------------------------------------------------------

def read_t4e_inputs(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        line = line.split("#")[0].split()
        if not line:
            continue
        key, text = line
        values[key] = int(text) if key in INT_KEYS else float.fromhex(text)
    missing = [k for k in T4E_KEYS if k not in values]
    if missing:
        sys.exit(f"{path}: missing {missing}")
    return values


def product_value(name):
    for line in PRODUCT_TABLE.read_text().splitlines():
        f = line.split("\t")
        if len(f) > 4 and f[1] == name:
            return float.fromhex(f[4]) if f[2] == "f32" else int(f[3])
    sys.exit(f"{PRODUCT_TABLE}: no record {name}")


def t4e_values(p):
    """The t4e_inputs.txt values from the repository (module docstring), for the L4 fixture p's tick."""
    si, _, _ = gic.imu_config(fe.PROFILE)
    card = schema.load_yaml(fe.CARD)
    mass, k = card["mass"]["value"], card["rotors"]["thrust_coeff"]["value"]
    noise = nt.noise_inputs({"fixture": p, "su1": lo4.Setup({**p, "rate_loop_divisor": 1})}, 0.0)
    budget = schema.load_yaml(fe.BUDGET)
    out = {"latency_samples": si["latency_samples"], "omega_hover": math.sqrt(mass * gic.G / (4 * k)),
           "gyro_sigma_d": noise.sigma_d, "gyro_lsb": noise.lsb, "gyro_k_rw": noise.k_rw,
           "rate_ff_filter_tau": product_value("rate_ff_filter_tau")}
    out.update({key: float(budget[key]["value"]) for key in BUDGET_KEYS})
    return out


def write_t4e_inputs(path, values):
    lines = [
        "# The T4e inputs of the refdata set quad/L06/t4e (t4e_reference.py --refresh-inputs; hex floats are exact):",
        "# latency_samples of the sensor profile; omega_hover = sqrt(m G / (4 k)) of the card at standard gravity",
        "# (gyro_chain_design.chain_design); gyro_sigma_d, gyro_lsb and gyro_k_rw at the nominal tick",
        "# (l6_noise_term.noise_inputs); rate_ff_filter_tau from the product parameter table; the rest design/budget.yaml.",
    ]
    for key in T4E_KEYS:
        v = values[key]
        lines.append(f"{key} {v}" if key in INT_KEYS else f"{key} {float(v).hex()}  # {v!r}")
    Path(path).write_text("\n".join(lines) + "\n")


class Inputs:
    def __init__(self, directory):
        d = Path(directory)
        self.t4 = read_t4e_inputs(d / T4E_INPUTS)
        self.p4 = lo4.read_inputs(str(d / L4_FIXTURE))
        self.p5 = ao.read_inputs(str(d / L5_FIXTURE))
        self.qfix = ao.read_q_inputs(str(d / L5_Q_FIXTURE))
        shared = [k for k in self.p4 if k in self.p5 and self.p4[k] != self.p5[k]]
        if shared:
            sys.exit(f"the L4 and L5 fixtures differ in {shared}")
        self.su4 = lo4.Setup(self.p4)
        self.su5 = ao.Setup(self.p5)
        lat, om = self.t4["latency_samples"], self.t4["omega_hover"]
        self.chain4 = m.Chain("T4e", lat, m.t4e_stages(self.p4, om))
        self.chain5 = m.Chain("T4e", lat, m.t4e_stages(self.p5, om))
        if self.chain4 != self.chain5:
            sys.exit("the L4 and L5 fixtures give different T4e chains")
        su1 = lo4.Setup({**self.p4, "rate_loop_divisor": 1})
        self.noise = nt.Noise(sigma_d=self.t4["gyro_sigma_d"], lsb=self.t4["gyro_lsb"], k_rw=self.t4["gyro_k_rw"],
                              dt=su1.tick_s)
        self.reg = {k: {"value": self.t4[k]} for k in BUDGET_KEYS}
        p = self.p4
        pid = fe.Law("PID", kp=tuple(p[f"rate_kp_{a}"] for a in AXES), ki=tuple(p[f"rate_ki_{a}"] for a in AXES),
                     kd=tuple(p[f"rate_kd_{a}"] for a in AXES), tau_ref=tuple(p[f"rate_tau_ref_{a}"] for a in AXES),
                     d_tau=tuple(p[f"rate_d_filter_tau_{a}"] for a in AXES))
        ffl = dataclasses.replace(pid, name=nt.FF_LAW, inertia=tuple(p[k] for k in fe.INERTIA_KEYS),
                                     motor_tau=p["motor_tau"], ff_tau=self.t4["rate_ff_filter_tau"])
        plan = run_l4.plan_acro(run_l4.l4s.load_acro(fe.ACRO), fe.plan_params(p), fe.CARD)
        sps, stamps = [], None
        for a in range(3):
            s_, stamps = run_l4.acro_setpoints(plan, a)
            sps.append(s_)
        self.sensor = self.chain4.sensor()
        self.c = {"fixture": p, "laws": {"PID": pid, nt.FF_LAW: ffl}, "su1": su1, "plan": plan, "sps": sps,
                  "stamps": stamps, "sensors": {"T4e": fe.Sensor("T4e", 0, (), linear=self.sensor)}}

    def seeds(self, klass):
        return nt.seed_count(klass, self.reg)

    def quantile(self, klass, m_points):
        conf = self.reg["noise_term_confidence"]["value"]
        n = self.seeds(klass)
        return conf, n, nt.z_value(conf, n, m_points)


# ---- the scenarios ------------------------------------------------------------------------------------------------

def klass_of(name):
    return CLASS.get(name, nt.CLASS_TRACKING)


def l4_step(inp, name, procs):
    p, su = inp.p4, inp.su4
    plan = run_l4.plan(run_l4.l4s.load(L4_SCEN / f"{name}.yaml"), p, fe.CARD)
    axis = AXES[plan.axis_index]
    n_exec = su.executions(axis)
    if plan.n_exec != n_exec or len(plan.window) != n_exec:
        sys.exit(f"{name}: the plan's window ({plan.n_exec}) is not the oracle's horizon {n_exec}")
    sp = p[f"rate_max_{axis}"]
    stamps = m.oracle_stamps(su, n_exec)
    band = m.l4_band(p, axis, inp.chain4, [sp] * n_exec, stamps, procs)
    tol, l1, rho = m.l4_tol(p, axis, inp.chain4, [sp] * n_exec, stamps, n_exec, procs)
    case = nt.Case(name, klass_of(name), tuple(plan.window), (plan.axis_index,), (nt.Op(),))
    series = nt.sigma_series(inp.c, case, inp.noise, m.GRID, 0.0, procs, inp.sensor)[plan.axis_index]
    m_s = n_exec
    conf, n_seeds, z = inp.quantile(case.klass, m_s)
    ch = {"name": f"w_{axis}", "first": 1, "lo": band["lo"], "hi": band["hi"], "run": list(plan.window),
          "sigma": series["sigma"], "H": band["H"], "TOL": tol, "Q": None, "kin": None,
          "extra": {"F": tol + band["H"]}}
    return {"name": name, "level": "L4", "klass": case.klass, "status": "final", "m_s": m_s, "conf": conf, "N": n_seeds,
            "z": z, "channels": [ch],
            "points": f"the stepped axis {axis} at the N = {n_exec} window executions {plan.window.start}.."
                      f"{plan.window.stop - 1} (oracle n = 1..N is run execution k0 + n - 2, k0 = {plan.step_execution})",
            "notes": [f"l1 {k} {v!r}" for k, v in l1.items()] + [f"rho {k} {v!r}" for k, v in rho.items()]}


def l4_acro(inp, procs):
    p, su, c = inp.p4, inp.su4, inp.c
    plan = c["plan"]
    phases, _ = run_l4.acro_phases(plan)
    case, peak = nt.l4_case(c, "acro", procs)
    series = nt.sigma_series(c, case, inp.noise, m.GRID, 0.0, procs, inp.sensor)
    m_s = len(case.axes) * len(case.points)
    conf, n_seeds, z = inp.quantile(case.klass, m_s)
    checked = set(case.points)
    channels = []
    for a, axis in enumerate(AXES):
        band = m.l4_band(p, axis, inp.chain4, c["sps"][a], c["stamps"], procs)
        tol, _, _ = m.l4_tol(p, axis, inp.chain4, c["sps"][a], c["stamps"], su.executions(axis), procs)
        big = max(max(abs(x) for x in band["lo"]), max(abs(x) for x in band["hi"]))
        if big != peak[a]:
            sys.exit(f"acro {axis}: the band's peak {big!r} differs from l6_noise_term.acro_peak's {peak[a]!r}")
        sig = dict(zip(case.points, series[a]["sigma"]))
        z_rest = run_l4.rest_bound(band["lo"], band["hi"])
        window = [k for k, ph in enumerate(phases) if ph == "recovery"]
        channels.append({"name": f"w_{axis}", "first": 0, "lo": band["lo"], "hi": band["hi"],
                         "run": list(range(len(band["lo"]))),
                         "sigma": [sig[k] if k in checked else None for k in range(len(band["lo"]))],
                         "H": band["H"], "TOL": tol, "Q": None, "kin": None,
                         "extra": {"P": big, "F": tol + band["H"],
                                   "Z_recovery_max": max(z_rest[k] for k in window)}})
    linear = sum(1 for ph in phases if ph == "linear")
    recovery = sum(1 for ph in phases if ph == "recovery")
    return {"name": "acro", "level": "L4", "klass": case.klass, "status": "final", "m_s": m_s, "conf": conf,
            "N": n_seeds, "z": z, "channels": channels,
            "points": f"three axes at the {linear} linear executions ((iii): 3 x {linear}) and the {recovery} recovery "
                      f"executions (3 x {recovery}); rows hold the band at every execution, sigma and W at the checked ones",
            "notes": [f"feed-forward operating points: w0 = 0 and (+-P_roll, +-P_pitch, +-P_yaw), P = "
                      + ", ".join(repr(x) for x in peak)]}


def l5_plan(inp, name):
    return run_l5.plan(l5s.load(L5_SCEN / f"{name}.yaml"), inp.p5, fe.CARD)


def l5_h_seg(plan):
    starts = [s[0] for s in plan.segments]
    if len(starts) != 2 or starts[0] != 1 or plan.end != 2 * (starts[1] - 1):
        sys.exit(f"the scenario's segments {starts} and end {plan.end} are not the T3 script (1, 1 + H, 2 H)")
    return starts[1] - 1


def _sig(series, axis, ch, k):
    return series[axis][ch]["sigma"][k]


def l5_sources(l5noise):
    """The noise series: the larger of the two frozen Jacobian ends (the rows) and each end alone (reported)."""
    return l5noise, m.l5_end_view(l5noise, 0), m.l5_end_view(l5noise, 1)


def with_end_maxima(channels, sig_lists, sources):
    """Adds sigma_max_fb_hi and sigma_max_fb_0 (the channel's largest sigma at the Jacobian end kp, resp. 0, alone) to
    each channel's extra."""
    for label, src in (("sigma_max_fb_hi", sources[1]), ("sigma_max_fb_0", sources[2])):
        for ch, lst in zip(channels, sig_lists(src)):
            ch["extra"][label] = max(s for s in lst[ch["first"]:] if s is not None)


_TOL = {}


def l5_golden(inp, name, h, procs):
    """(run, (TOL_theta, TOL_omega), (kin_theta, kin_omega), checks) of the T4e golden of script `name` (cached)."""
    if name not in _TOL:
        su = inp.su5
        run = m.simulate4e(su, name, h, inp.chain5)
        if run["cross_axes"] != 0.0:
            sys.exit(f"{name}: the error bound of an off-axis rate setpoint is {run['cross_axes']!r}, not 0: the single-axis "
                     "derivation does not hold")
        kin = m.kinematics_change(su, name, h, inp.chain5, run)
        bound, tol = m.l5_tolerance(su, name, run, inp.chain5, procs)
        checks = [f"{name} kinematics change below TOL (the oracle's check): "
                  f"{'yes' if kin[0] < tol[0] and kin[1] < tol[1] else 'NO'}"]
        if name == "yaw_release":
            before, at, rbound, by = run["lock_info"]
            margin = min(before, -at)
            checks.append(f"yaw_release golden lock_execution {run['lock_a']} by {by}; decision margin {margin!r} exceeds "
                          f"TOL_omega + the rate bound {tol[1] + rbound!r} (the oracle's check): "
                          f"{'yes' if margin > tol[1] + rbound else 'NO'}")
        checks += [f"{name} err_theta_{n} {b[0]!r} err_omega_{n} {b[1]!r}" for n, b in bound.items()]
        _TOL[name] = (run, tol, kin, checks)
    return _TOL[name]


def l5_step(inp, name, l5noise, procs):
    su = inp.su5
    plan = l5_plan(inp, name)
    h = l5_h_seg(plan)
    axis = ao.scenario_axis(name)
    env, q, saturated = m.tilt_envelope(su, axis, h, inp.chain5, inp.qfix, procs)
    _, tol, kin, checks = l5_golden(inp, name, h, procs)
    n_att = su.attitude_executions(h)
    m_s = len(COMPONENTS) * n_att
    conf, n_seeds, z = inp.quantile(klass_of(name), m_s)
    a0 = plan.origin
    run_ex = [a0 + n for n in range(n_att)]
    sources = l5_sources(l5noise)

    def sig_lists(src):
        return [[_sig(src, c, "theta", k) for k in run_ex] for c in range(len(COMPONENTS))]

    e = env["theta"]
    channels = []
    for c, (comp, sig) in enumerate(zip(COMPONENTS, sig_lists(sources[0]))):
        stepped = c == axis
        channels.append({"name": comp, "first": 0, "lo": e.lo if stepped else [0.0] * n_att,
                         "hi": e.hi if stepped else [0.0] * n_att, "run": run_ex, "sigma": sig, "H": e.halving(),
                         "TOL": tol[0], "Q": q["theta"]["q"] if stepped else 0.0, "kin": kin[0],
                         "extra": {"band": "theta of the script" if stepped else "0 (a pure single-axis script)",
                                   "F": tol[0] + e.halving() + kin[0],
                                   "end_value": max(abs(e.lo[-1]), abs(e.hi[-1]))}})
    with_end_maxima(channels, sig_lists, sources)
    f_stepped = channels[axis]["extra"]["F"]
    end = channels[axis]["extra"]["end_value"]
    return {"name": name, "level": "L5", "klass": klass_of(name), "status": "final", "m_s": m_s, "conf": conf,
            "N": n_seeds, "z": z, "channels": channels,
            "points": f"the three components at n = 0..{n_att - 1} (run execution a0 + n, a0 = {a0}); F = TOL + H + kin "
                      "is the stepped channel's for all three, Q the stepped channel's (0 for the other two), as the "
                      "frozen predicate",
            "notes": [f"segment_executions {h}", f"omega H {env['omega'].halving()!r}", f"omega Q {q['omega']['q']!r}",
                      f"TOL_omega {tol[1]!r}", f"kinematics_change_omega {kin[1]!r}",
                      f"quantiser saturated executions {saturated}",
                      f"theta Q rule {q['theta']['rule']} q_corners {q['theta']['q_corners']!r} q_grid "
                      f"{q['theta']['q_grid']!r}",
                      f"settle check (the T3 property): end {end!r} < F {f_stepped!r}: {'yes' if end < f_stepped else 'NO'}"]
            + checks}


def l5_yaw(inp, name, l5noise, procs):
    su = inp.su5
    plan = l5_plan(inp, name)
    h = l5_h_seg(plan)
    stick = plan.segments[0][2][2]
    d_amp = 0.0
    if plan.disturbance is not None:
        d_amp = plan.disturbance["yaw_nm"]
        if d_amp != inp.p5["tau_held_yaw_nm"] or plan.disturbance["start_attitude_execution"] != 1 + h:
            sys.exit(f"{name}: the disturbance is not the T3 fallback script's")
    env, info, q, saturated = m.yaw_envelopes(su, h, stick, d_amp, inp.chain5, inp.qfix, procs)
    _, tol, kin, checks = l5_golden(inp, "yaw_release", h, procs)
    n_att = su.attitude_executions(h)
    a_r = info["a_r"]
    a0 = plan.origin
    run_ex = [a0 + n for n in range(n_att)]
    lock_lo, lock_hi = info["lock_range"]
    if name == "yaw_release":
        m_s = n_att + (n_att - lock_lo) + (n_att - a_r) + 2
        pts = (f"omega at n = 0..{n_att - 1}, heading_lock from the earliest lock {lock_lo}, heading_release from n_r = "
               f"{a_r}, the two at-zero items at n = {n_att - 1}")
    else:
        fb_lo, fb_hi = info["fallback_range"]
        m_s = (fb_hi - a_r + 1) + 2 * (n_att - fb_lo) + 2
        pts = (f"sigma_r w at n = {a_r}..{fb_hi} (the latest fallback), omega and heading_lock from the earliest "
               f"fallback {fb_lo}, the two at-zero items; heading_release reported, not counted")
    conf, n_seeds, z = inp.quantile(klass_of(name), m_s)
    sources = l5_sources(l5noise)

    def sig_lists(src):
        th = [_sig(src, 2, "theta", k) for k in run_ex]
        om = [_sig(src, 2, "omega", k) for k in run_ex]
        s_lock = max(th[lock_lo:lock_hi + 1])
        return [om, [None] * a_r + [th[k] + th[a_r] for k in range(a_r, n_att)],
                [None] * lock_lo + [th[k] + s_lock for k in range(lock_lo, n_att)]]

    rules = {"omega": "sigma_omega,yaw(n)", "heading_release": "sigma_theta,yaw(n) + sigma_theta,yaw(n_r)",
             "heading_lock": "sigma_theta,yaw(n) + max sigma_theta,yaw over the lock range"}
    channels = []
    for ch, sig in zip(m.YAW_CHANNELS, sig_lists(sources[0])):
        e = env[ch]
        first = 0 if ch == "omega" else (a_r if ch == "heading_release" else lock_lo)
        t_c, k_c = (tol[1], kin[1]) if ch == "omega" else (tol[0], kin[0])
        channels.append({"name": ch, "first": first, "lo": e.lo[first:], "hi": e.hi[first:], "run": run_ex[first:],
                         "sigma": sig[first:], "H": e.halving(), "TOL": t_c, "Q": q[ch]["q"], "kin": k_c,
                         "extra": {"sigma_rule": rules[ch], "F": t_c + e.halving() + k_c,
                                   "end_value": max(abs(e.lo[-1]), abs(e.hi[-1])), "Q_rule": q[ch]["rule"]}})
    with_end_maxima(channels, sig_lists, sources)
    fb = info["fallback_range"]
    notes = [f"segment_executions {h}", f"release_execution {a_r}", f"lock_execution_min {lock_lo}",
             f"lock_execution_max {lock_hi}", f"lock_execution_range_coarse {info['lock_range_coarse'][0]} "
             f"{info['lock_range_coarse'][1]}", f"fallback_execution_range {fb[0] if fb else None} {fb[1] if fb else None}",
             f"min_sigma_omega_to_fallback {info['min_sigma']!r}", f"lock_by {' '.join(info['lock_by'])}",
             f"stick_scale {stick!r}", f"disturbance_nm {d_amp!r}", f"quantiser saturated executions {saturated}",
             "TOL and kin are the yaw_release golden's for both yaw scripts (the frozen predicate's F)"]
    for ch in channels:
        if ch["name"] != "heading_release":
            ok = ch["extra"]["end_value"] < ch["extra"]["F"]
            notes.append(f"settle check (the T3 property) {ch['name']}: end {ch['extra']['end_value']!r} < F "
                         f"{ch['extra']['F']!r}: {'yes' if ok else 'NO'}")
    if name == "yaw_fallback" and not info["min_sigma"] > 0:
        notes.append("T3 property FAILS on T4e: a member reverses before its fallback at the scenario's stick")
    return {"name": name, "level": "L5", "klass": klass_of(name), "status": "final", "m_s": m_s, "conf": conf,
            "N": n_seeds, "z": z, "channels": channels, "points": pts + f" (run execution a0 + n, a0 = {a0})",
            "notes": notes + checks}


def l5_recovery_t3_terms(inp, procs):
    """The frozen recovery test's T3 term on T4e (test_t4_recovery.recorded_t3_terms's rule, INFERRED there and here):
    angle = the largest TOL + kin of the step scripts' theta and the yaw scripts' heading; rate = of the yaw omega."""
    angle = rate = 0.0
    for name in L5_STEPS:
        _, tol, kin, _ = l5_golden(inp, name, l5_h_seg(l5_plan(inp, name)), procs)
        angle = max(angle, tol[0] + kin[0])
    _, tol, kin, _ = l5_golden(inp, "yaw_release", l5_h_seg(l5_plan(inp, "yaw_release")), procs)
    return max(angle, tol[0] + kin[0]), tol[1] + kin[1]


def l5_recovery(inp, name, l5noise, procs):
    su = inp.su5
    vals = l5s.values(l5s.load(L5_SCEN / f"{name}.yaml"))
    st = vals["initial_state"]
    count = vals["script"]["end_attitude_execution"] + 1
    fn = rm.alpha_channel if name == R1X else None
    res = m.recovery_envelope(su, inp.qfix, st["attitude_q_wxyz"], st["body_rates_frd_rad_s"], count, inp.chain5, procs,
                              fn)
    t3_angle, t3_rate = l5_recovery_t3_terms(inp, procs)
    sources = l5_sources(l5noise)
    if name == R1X:
        names = ("alpha",)

        def sig_lists(src):
            return [[math.hypot(_sig(src, 0, "theta", k), _sig(src, 1, "theta", k)) for k in range(count)]]
        m_s = count
        pts = f"alpha at n = 0..{count - 1} (run execution n)"
    else:
        names = rm.CHANNELS

        def sig_lists(src):
            return [[_sig(src, c % 3, "theta" if c < 3 else "omega", k) for k in range(count)] for c in range(6)]
        m_s = len(names) * count - 3
        pts = f"six channels at n = 0..{count - 1} (run execution n), the rates not at n = 0"
    conf, n_seeds, z = inp.quantile(klass_of(name), m_s)
    channels = []
    for c, (ch, sig) in enumerate(zip(names, sig_lists(sources[0]))):
        e = res["env"][c]
        t3 = t3_angle if (fn is not None or c < 3) else t3_rate
        channels.append({"name": ch, "first": 0, "lo": e.lo, "hi": e.hi, "run": list(range(count)), "sigma": sig,
                         "H": res["halving"][c], "TOL": t3, "Q": res["q"][c], "kin": res["kin"][c],
                         "extra": {"F": t3 + res["halving"][c] + res["kin"][c], "q_corners": res["q_corners"][c],
                                   "end_value": max(abs(e.lo[-1]), abs(e.hi[-1]))}})
    if name == R1X:
        channels[0]["extra"]["sigma_rule"] = "sqrt(sigma_theta,roll^2 + sigma_theta,pitch^2)"
    with_end_maxima(channels, sig_lists, sources)
    notes = [f"quantiser saturated executions {res['saturated']}",
             "TOL is the frozen recovery test's T3 term on T4e: the largest TOL + kin of the step and yaw goldens' angle "
             f"channels ({t3_angle!r}) for the angles, of the yaw omega ({t3_rate!r}) for the rates; INFERRED to bound the "
             "float32 rounding of a recovery (unverified), as test_t4_recovery.py"]
    for ch in channels:
        ok = ch["extra"]["end_value"] < ch["extra"]["F"]
        notes.append(f"settle check (the T3 property) {ch['name']}: end {ch['extra']['end_value']!r} < F "
                     f"{ch['extra']['F']!r}: {'yes' if ok else 'NO'}")
    return {"name": name, "level": "L5", "klass": klass_of(name), "status": R2_PROVISIONAL if name == R2 else "final",
            "m_s": m_s, "conf": conf, "N": n_seeds, "z": z, "channels": channels, "points": pts, "notes": notes}


def l5_noise_horizon(inp):
    """The largest run execution any L5 scenario checks, plus one."""
    n = 0
    for name in (*L5_STEPS, *L5_YAW):
        plan = l5_plan(inp, name)
        n = max(n, plan.origin + plan.end + 1)
    for name in L5_RECOVERIES:
        n = max(n, l5s.values(l5s.load(L5_SCEN / f"{name}.yaml"))["script"]["end_attitude_execution"] + 1)
    return n


# ---- the files ----------------------------------------------------------------------------------------------------

def fmt(v):
    return "-" if v is None else repr(v)


def header(inp):
    t4 = inp.t4
    ch = inp.chain4
    return [
        "# quad/L06/t4e, the stage (e) T4e reference (decision 0019; the noise suite memo, C6a). Written by "
        "t4e_reference.py; see its docstring.",
        f"model T4e linear design model: latency_samples {ch.latency}, {len(ch.stages) - 1} notches at omega_hover "
        f"{t4['omega_hover']!r} rad/s, the low-pass; tau x J grid {m.GRID} x {m.GRID} (H from its "
        f"{(m.GRID + 1) // 2} x {(m.GRID + 1) // 2} subset); "
        "nominal corner (no bias, no clock error)",
        f"noise sigma_d {inp.noise.sigma_d!r} lsb {inp.noise.lsb!r} k_rw {inp.noise.k_rw!r} dt {inp.noise.dt!r} "
        f"white_var {inp.noise.white_var!r} rw_var {inp.noise.rw_var!r}",
    ]


def write_scenario(path, inp, res):
    """The scenario file: a header, `channel` summary lines, then per channel `rows <name> first <n0> count <k>` and one
    row per execution: n, the run's execution, lo, hi, sigma_c(n), W(n) = z sigma_c(n) ('-' where not checked)."""
    lines = header(inp) + [
        f"scenario {res['name']}",
        f"level {res['level']}",
        f"class {res['klass']}",
        f"status {res['status']}",
        f"quantile noise_term_confidence {res['conf']!r} N {res['N']} M_s {res['m_s']} z {res['z']!r}",
        f"points {res['points']}",
    ]
    lines += [f"note {x}" for x in res["notes"]]
    z = res["z"]
    for ch in res["channels"]:
        sig = [s for s in ch["sigma"] if s is not None]
        i = max(range(len(ch["sigma"])), key=lambda k: -1.0 if ch["sigma"][k] is None else ch["sigma"][k])
        smax = max(sig)
        extra = " ".join(f"{k} {v!r}" if not isinstance(v, str) else f"{k} '{v}'" for k, v in ch["extra"].items())
        lines.append(f"channel {ch['name']} H {ch['H']!r} TOL {ch['TOL']!r} Q {'none' if ch['Q'] is None else repr(ch['Q'])} "
                     f"kin {'none' if ch['kin'] is None else repr(ch['kin'])} sigma_max {smax!r} sigma_max_at "
                     f"{ch['first'] + i} W_max {z * smax!r} {extra}")
    for ch in res["channels"]:
        n = len(ch["lo"])
        lines.append(f"rows {ch['name']} first {ch['first']} count {n} columns n run_execution lo hi sigma W")
        for k in range(n):
            s = ch["sigma"][k]
            lines.append(f"{ch['first'] + k} {ch['run'][k]} {ch['lo'][k]!r} {ch['hi'][k]!r} {fmt(s)} "
                         f"{fmt(None if s is None else z * s)}")
    Path(path).write_text("\n".join(lines) + "\n")


def write_index(path, inp, results):
    lines = header(inp) + ["# per scenario: file, class, status, N, M_s, z; per channel H, TOL, Q, kin, sigma_max, "
                           "W_max = z sigma_max"]
    for level, name in SCENARIOS:
        sid = scenario_id(level, name)
        if name in (L4_CHIRPS if level == "L4" else L5_CHIRPS):
            lines.append(f"scenario {sid} file - class {nt.CLASS_TRACKING} status {CHIRP_UNKNOWN}")
            continue
        r = results[sid]
        lines.append(f"scenario {sid} file {sid}.txt class {r['klass']} status {r['status']} N {r['N']} M_s {r['m_s']} "
                     f"z {r['z']!r}")
        for ch in r["channels"]:
            smax = max(s for s in ch["sigma"] if s is not None)
            lines.append(f"  channel {ch['name']} H {ch['H']!r} TOL {ch['TOL']!r} "
                         f"Q {'none' if ch['Q'] is None else repr(ch['Q'])} "
                         f"kin {'none' if ch['kin'] is None else repr(ch['kin'])} sigma_max {smax!r} "
                         f"W_max {r['z'] * smax!r}")
    Path(path).write_text("\n".join(lines) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dir", default=str(HERE), help="directory of the inputs and the outputs")
    ap.add_argument("--procs", type=int, default=1, help="worker processes (the output does not depend on it)")
    ap.add_argument("--only", nargs="+", metavar="ID", help="write only these outputs (ids of --list without .txt)")
    ap.add_argument("--list", action="store_true", help="print the output file names and stop")
    ap.add_argument("--refresh-inputs", action="store_true", help=f"rewrite {T4E_INPUTS} in --dir and stop")
    args = ap.parse_args(argv)
    if args.list:
        print("\n".join(output_names()))
        return 0
    d = Path(args.dir)
    if args.refresh_inputs:
        write_t4e_inputs(d / T4E_INPUTS, t4e_values(lo4.read_inputs(str(d / L4_FIXTURE))))
        return 0
    known = [n[:-len(".txt")] for n in output_names()]
    wanted = known if not args.only else args.only
    bad = [x for x in wanted if x not in known]
    if bad:
        ap.error(f"--only {bad}: not outputs of the set (known: {' '.join(known)})")
    if INDEX in wanted:
        wanted = known
    inp = Inputs(d)
    t_all = time.time()
    results = {}
    l5noise = None
    if any(x.startswith("l5_") for x in wanted):
        t0 = time.time()
        l5noise = m.l5_sigma_series(inp.su5, inp.chain5, inp.noise, l5_noise_horizon(inp), m.GRID, args.procs)
        print(f"t4e_reference: L5 noise series {time.time() - t0:.1f} s", file=sys.stderr, flush=True)
    for level, name in SCENARIOS:
        sid = scenario_id(level, name)
        if sid not in wanted:
            continue
        t0 = time.time()
        if level == "L4":
            res = l4_acro(inp, args.procs) if name == "acro" else l4_step(inp, name, args.procs)
        elif name in L5_STEPS:
            res = l5_step(inp, name, l5noise, args.procs)
        elif name in L5_YAW:
            res = l5_yaw(inp, name, l5noise, args.procs)
        else:
            res = l5_recovery(inp, name, l5noise, args.procs)
        results[sid] = res
        write_scenario(d / f"{sid}.txt", inp, res)
        print(f"t4e_reference: {sid} {time.time() - t0:.1f} s", file=sys.stderr, flush=True)
    if INDEX in wanted:
        write_index(d / f"{INDEX}.txt", inp, results)
    print(f"t4e_reference: total {time.time() - t_all:.1f} s on {args.procs} processes", file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
