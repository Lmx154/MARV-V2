#!/usr/bin/env python3
"""Runner of the quad L5 T4 scenarios (quad spec section 4 L5, docs/decisions/0006 B "Plugin", "Runner" and F).

  run_l5.py --card <card> --scenario scenarios/quad/L05/<name>.yaml --out-dir <dir> --m <int> [--plugin-dir <dir>]

As a module: plan(), run_step(), run_sequence(), write_report(). Every run is labelled truth-fed, perfect-model: the rate
loop's gyro sample and the attitude state are the plant's truth (<gyro_source>truth</gyro_source> and
<attitude_source>truth</attitude_source>), truth and firmware share one card (core section 6): not a validation run.

The L4 runner's machinery is reused by import (tools/sim/run_l4.py: the parameter-table reader, the L2-path world
generation, the overrides). This module adds the L5 schema (tools/sim/l5_scenario.py), the attitude-divisor plan, the
second world element and the TRUTH records of the log.

Live parameters. The plugin build's parameter table (MARV_GZ_PARAMS must be marv_params_l5_attitude_scripted, the
host-gz-l5 build). Refused before any gz process starts: a scenario tick different from the build's
tick_period_num_us / tick_period_den, an m that does not divide the attitude divisor rate_loop_divisor * att_loop_ratio
(decision 0006 B, Runner: an attitude sample would then be held from an earlier tick), and stamps that do not fit the
composition's i32 parameters.

Plan (derived values, each with its rule). D_a = rate_loop_divisor * att_loop_ratio, t = the tick period (exact rational),
T_a = D_a t. The attitude group runs at ticks D_a a (tick 0 included) and the SIL stamps tick j with floor(j num / den).
  origin execution a0   the smallest a >= 1 with D_a a t >= settle_s and D_a a num = 0 (mod den): the stamps of executions
                        a0, a0 + 1, ... are the T3 oracle's stamps of executions 0, 1, ... plus one integer, so every dt of
                        the window is the oracle's dt. Oracle execution n is the run's attitude execution a0 + n.
  segment stamps        l5_seg<i>_t_us = floor(D_a (a0 + start_i) num / den): the segment applies at the execution a0 + start_i
                        (its stamp is >= t_us) and not at the one before.
  end                   attitude execution a0 + end_attitude_execution, the last of the window.
  duration_ticks        the smallest multiple of every m of m_sequence above D_a (a0 + end), the last window tick.
  l5_thrust_n           float32 of m g(phi, h0), as run_l4.hover_thrust.
"""

from __future__ import annotations

import argparse
import cmath
import dataclasses
import functools
import math
import re
import sys
from fractions import Fraction
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "card"))
import attitude  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import run_l4 as l4  # noqa: E402
import run_scenario  # noqa: E402
import scenario as scn  # noqa: E402
import schema  # noqa: E402

DEFAULT_PLUGIN_DIR = ROOT / "build" / "host-gz-l5" / "sim" / "gz" / "plugin"
COMPOSITION_PARAMS = "marv_params_l5_attitude_scripted"
LABEL = "truth-fed, perfect-model"
NAME_LABEL = l4.NAME_LABEL
LABEL_LINE = (f"label: {LABEL}. The rate loop's gyro sample and the attitude state are the plant's truth (<gyro_source>"
              "truth</gyro_source>, <attitude_source>truth</attitude_source>, decision 0006 B) and truth and firmware "
              "share one card (core section 6): not a validation run")
ATTITUDE_ELEMENT = "<attitude_source>truth</attitude_source>"
F32, I32 = l4.F32, l4.I32
I32_LIMIT_US = 2 ** 31
CHIRP_AXIS_CODE = {"roll": 1, "pitch": 2, "yaw": 3}  # l5_script.hpp ChirpAxis: None, Roll, Pitch, Yaw
PlanError = l4.PlanError


def build_parameters(plugin_dir):
    """(parameter set name, path of its param_defaults.cpp) of the build that holds plugin_dir; refuses a build that is
    not the host-gz-l5 build."""
    for d in [Path(plugin_dir).resolve(), *Path(plugin_dir).resolve().parents]:
        cache = d / "CMakeCache.txt"
        if cache.exists():
            m = l4.CACHE_PARAMS.search(cache.read_text(encoding="utf-8", errors="replace"))
            if not m:
                raise PlanError([f"{cache}: no MARV_GZ_PARAMS entry (not a gz build)"])
            if m.group(1) != COMPOSITION_PARAMS:
                raise PlanError([f"{cache}: MARV_GZ_PARAMS is {m.group(1)!r}, not {COMPOSITION_PARAMS!r}: the plugin is "
                                 "not the host-gz-l5 build"])
            return m.group(1), d / "generated" / m.group(1) / "marv" / "params" / "param_defaults.cpp"
    raise PlanError([f"{plugin_dir}: no CMakeCache.txt above the plugin directory"])


# ---- the plan -------------------------------------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Plan:
    num_us: int
    den: int
    divisor: int
    ratio: int
    tick_s: Fraction
    period_s: Fraction  # T_a
    settle_s: float
    origin: int  # a0
    end: int  # last window execution, counted from the origin
    segments: tuple  # (start, stamp_us, (s_roll, s_pitch, s_yaw)) per segment, start counted from the origin
    chirp: dict | None
    disturbance: dict | None
    duration_ticks: int
    m_sequence: tuple
    thrust_n: float
    thrust_n_double: float

    @property
    def att_divisor(self):
        return self.divisor * self.ratio

    def tick_of(self, a):
        return self.att_divisor * a

    def stamp_us(self, a):
        return (self.att_divisor * a * self.num_us) // self.den

    def run_execution(self, n):
        """The run's attitude execution of oracle execution n."""
        return self.origin + n

    @property
    def window(self):
        return range(self.origin, self.origin + self.end + 1)

    def overrides(self):
        """The composition parameters of the run: name -> (type, text), the text exact for its type."""
        out = {"l5_thrust_n": (F32, repr(self.thrust_n)), "l5_seg_count": (I32, str(len(self.segments)))}
        for i, (start, stamp, stick) in enumerate(self.segments, 1):
            out[f"l5_seg{i}_t_us"] = (I32, str(stamp))
            for name, s in zip(l5s.AXES, stick):
                out[f"l5_seg{i}_{name}"] = (F32, repr(l4.r32(s)))
        if self.chirp:
            c = self.chirp
            out.update({"l5_chirp_axis": (I32, str(CHIRP_AXIS_CODE[c["axis"]])),
                        "l5_chirp_amp_rad_s": (F32, repr(l4.r32(c["amp_rad_s"]))),
                        "l5_chirp_w_lo": (F32, repr(l4.r32(c["w_lo_rad_s"]))),
                        "l5_chirp_w_hi": (F32, repr(l4.r32(c["w_hi_rad_s"]))),
                        "l5_chirp_t0_us": (I32, str(c["t0_us"])), "l5_chirp_dur_us": (I32, str(c["dur_us"]))})
        if self.disturbance:
            d = self.disturbance
            out.update({"l5_dist_yaw_nm": (F32, repr(l4.r32(d["yaw_nm"]))), "l5_dist_t0_us": (I32, str(d["t0_us"]))})
        return out

    def report(self):
        return {
            "tick period": f"{self.num_us}/{self.den} us (build and scenario)",
            "rate_loop_divisor D": self.divisor, "att_loop_ratio": self.ratio, "attitude divisor": self.att_divisor,
            "attitude period T_a": f"{self.period_s} s", "settle_s": self.settle_s,
            "origin attitude execution a0": self.origin, "origin tick": self.tick_of(self.origin),
            "segments (start, stamp us, stick)": [list(s) for s in self.segments],
            "window attitude executions": f"{self.window.start}..{self.window.stop - 1}",
            "duration_ticks": self.duration_ticks, "m_sequence": list(self.m_sequence),
            "l5_thrust_n (float32)": self.thrust_n, "m g(phi, h0) (double)": self.thrust_n_double,
        }


def plan(doc, params, card, root=ROOT):
    """The derived plan of a valid L5 scenario against the build's parameters (module docstring). Raises PlanError."""
    vals = l5s.values(doc)
    script = vals["script"]
    need = ("tick_period_num_us", "tick_period_den", "rate_loop_divisor", "att_loop_ratio")
    missing = [k for k in need if k not in params]
    if missing:
        raise PlanError([f"the build's parameter table lacks {missing}"])
    num, den = params["tick_period_num_us"], params["tick_period_den"]
    divisor, ratio = params["rate_loop_divisor"], params["att_loop_ratio"]
    fails = []
    if (vals["tick_period_num_us"], vals["tick_period_den"]) != (num, den):
        fails.append(f"tick period {vals['tick_period_num_us']}/{vals['tick_period_den']} us differs from the build's "
                     f"{num}/{den} us: the rate loop would panic on the execution spacing")
    if divisor < 1 or ratio < 1:
        fails.append(f"the build's rate_loop_divisor {divisor} or att_loop_ratio {ratio} is below 1")
    else:
        for m in vals["m_sequence"]:
            if (divisor * ratio) % m:
                fails.append(f"m = {m} does not divide the attitude divisor rate_loop_divisor * att_loop_ratio = "
                             f"{divisor * ratio}: an attitude sample would be held from an earlier tick")
    if fails:
        raise PlanError(fails)

    att_div = divisor * ratio
    tick = Fraction(num, den * scn.MICROSECONDS_PER_SECOND)
    period = att_div * tick
    a0 = max(1, math.ceil(Fraction(script["settle_s"]) / period))
    for _ in range(den):
        if (att_div * a0 * num) % den == 0:
            break
        a0 += 1
    else:
        raise PlanError([f"no origin execution keeps the T3 stamp phase within {den} executions of settle_s"])

    def stamp(n):
        return (att_div * (a0 + n) * num) // den

    end = script["end_attitude_execution"]
    segments = tuple((s["start_attitude_execution"], stamp(s["start_attitude_execution"]), tuple(s["stick"]))
                     for s in script["segments"])
    chirp = disturbance = None
    if "chirp" in script:
        c = script["chirp"]
        dur = Fraction(c["duration_s"]) * scn.MICROSECONDS_PER_SECOND
        chirp = dict(c, t0_us=stamp(c["start_attitude_execution"]), dur_us=int(dur))
        if dur.denominator != 1:
            fails.append(f"chirp duration_s {c['duration_s']!r} is not a whole number of microseconds")
    if "disturbance" in script:
        disturbance = dict(script["disturbance"], t0_us=stamp(script["disturbance"]["start_attitude_execution"]))
    stamps = [s[1] for s in segments] + [x["t0_us"] for x in (chirp, disturbance) if x] + ([chirp["dur_us"]] if chirp else [])
    if any(s >= I32_LIMIT_US for s in stamps):
        fails.append(f"a stamp or duration does not fit the composition's i32 parameters (limit {I32_LIMIT_US} us)")
    if fails:
        raise PlanError(fails)
    lcm = math.lcm(*vals["m_sequence"])
    last_tick = att_div * (a0 + end)
    duration = (last_tick // lcm + 1) * lcm
    thrust = l4.hover_thrust(card, vals, root)
    return Plan(num_us=num, den=den, divisor=divisor, ratio=ratio, tick_s=tick, period_s=period,
                settle_s=script["settle_s"], origin=a0, end=end, segments=segments, chirp=chirp,
                disturbance=disturbance, duration_ticks=duration, m_sequence=tuple(vals["m_sequence"]),
                thrust_n=l4.r32(thrust), thrust_n_double=thrust)


# ---- the world ------------------------------------------------------------------------------------------------------

def l2_scenario_doc(doc, p, stem, source):
    """The L2-schema scenario gen_world reads (run_l4.l2_scenario_doc's content for an L5 document)."""
    vals = l5s.values(doc)

    def entry(value, unit, key):
        return {"value": value, "unit": unit, "label": "derived", "rule": f"{key} of {source} (tools/sim/run_l5.py)"}

    st = vals["initial_state"]
    return {
        "scenario": stem,
        "site_latitude_rad": entry(vals["site_latitude_rad"], "rad", "site_latitude_rad"),
        "site_height_m": entry(vals["site_height_m"], "m", "site_height_m"),
        "seed": entry(vals["seed"], "1", "seed"),
        "tick_period_num_us": entry(vals["tick_period_num_us"], "us", "tick_period_num_us"),
        "tick_period_den": entry(vals["tick_period_den"], "1", "tick_period_den"),
        "m_sequence": entry(list(p.m_sequence), "1", "m_sequence"),
        "duration_ticks": {"value": p.duration_ticks, "unit": "1", "label": "derived",
                           "rule": "the plan's duration_ticks (tools/sim/run_l5.py plan)"},
        "initial_state": {k: entry(st[k], doc["initial_state"][k]["unit"], f"initial_state.{k}")
                          for k in scn.STATE_FIELDS},
        "command": {"dshot": {"value": l4.L2_PLACEHOLDER_DSHOT, "unit": "1", "label": "scenario",
                              "rationale": "placeholder: world_edit removes the L2 ol_dshot_m* overrides; the "
                                           "l5_attitude_scripted composition commands the motors"}},
    }


def world_edit(overrides, extra_edit=None, *, gyro_source=True, attitude_source=True):
    """The sdf_edit of an L5 run: drop the four L2 DShot overrides, add `overrides` (name -> (type, text)) and the
    <gyro_source> and <attitude_source> truth elements (each only when asked: the controls leave one out) inside the
    lockstep plugin element, then apply `extra_edit` (a callable on the text)."""
    def edit(text):
        text, n = l4.L2_DSHOT_OVERRIDE.subn("", text)
        if n != scn.MOTORS:
            raise run_scenario.RunError(f"the world holds {n} L2 ol_dshot_m* overrides, expected {scn.MOTORS}")
        blocks = l4.PLUGIN_BLOCK.findall(text)
        if len(blocks) != 1:
            raise run_scenario.RunError(f"the world holds {len(blocks)} marv_gz_lockstep plugin elements, expected 1")
        add = "".join(f'\n  <sil_override param="{k}" type="{t}">{v}</sil_override>' for k, (t, v) in overrides.items())
        if gyro_source:
            add += f"\n  {l4.GYRO_ELEMENT}"
        if attitude_source:
            add += f"\n  {ATTITUDE_ELEMENT}"
        text = text.replace(blocks[0], blocks[0].replace("</plugin>", f"{add}\n</plugin>"), 1)
        return extra_edit(text) if extra_edit is not None else text
    return edit


# ---- running --------------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class StepRun:
    run: run_scenario.RunResult
    plan: Plan
    l5_scenario: str
    l5_scenario_sha256: str
    params_path: str
    overrides: dict
    harness_overrides: dict
    executions: list  # per attitude execution a: {"a", "tick", "t_us", "imu", "truth" (the TRUTH record or None), "fresh"}
    window_steps: range  # host steps whose reads cover the window's attitude executions
    window_stale: list  # host steps of window_steps whose raw read equals the previous step's


def executions(log, p, m):
    fresh = set(run_scenario.fresh_steps(log))
    truths = {t["tick"]: t for t in log["truths"]}
    out = []
    for tick in range(0, len(log["ticks"]), p.att_divisor):
        t = log["ticks"][tick]
        out.append({"a": tick // p.att_divisor, "tick": t["tick"], "t_us": t["sil_t_us"], "imu": t["imu"],
                    "truth": truths.get(tick), "fresh": tick // m in fresh})
    return out


def run_step(card, scenario, m, out_dir, plugin_dir=DEFAULT_PLUGIN_DIR, *, overrides=None, extra_edit=None, seed=None,
             root=ROOT, timeout_s=run_scenario.TIMEOUT_S, gyro_source=True, attitude_source=True, doc_edit=None):
    """One gz process of an L5 scenario at m ticks per host step; writes the world, the log and the run report into
    out_dir. `overrides` are harness sil_overrides, name -> (type, text). The two source flags leave the plugin element
    out of the world (the controls). `doc_edit`, when given, is called on the loaded scenario document before the plan
    (the chirp runs' amplitude and duration variants)."""
    doc = l5s.load(scenario)
    if doc_edit is not None:
        doc_edit(doc)
    _, defaults = build_parameters(plugin_dir)
    params = l4.read_param_defaults(defaults)
    harness = dict(overrides or {})
    if "att_loop_ratio" in harness:
        params["att_loop_ratio"] = int(harness["att_loop_ratio"][1])
    p = plan(doc, params, card, root)
    if m not in p.m_sequence:
        raise PlanError([f"m = {m} is not in the scenario's m_sequence {list(p.m_sequence)}"])
    applied = p.overrides()
    twice = sorted(set(applied) & set(harness))
    if twice:
        raise PlanError([f"harness overrides {twice} are plan parameters already"])
    applied.update(harness)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{doc['scenario']}_{NAME_LABEL}"
    l2_path = out_dir / f"{stem}.yaml"
    l2_path.write_text(yaml.safe_dump(l2_scenario_doc(doc, p, stem, Path(scenario).name), sort_keys=False),
                       encoding="utf-8")
    r = run_scenario._run(card, l2_path, seed, m, "test", None, out_dir, plugin_dir,
                          world_edit(applied, extra_edit, gyro_source=gyro_source, attitude_source=attitude_source),
                          None, root, Path(root) / "design" / "budget.yaml", timeout_s)
    ex = executions(r.log, p, m)
    fresh = set(run_scenario.fresh_steps(r.log))
    steps = range(p.tick_of(p.window.start) // m, p.tick_of(p.window.stop - 1) // m + 1)
    s = StepRun(run=r, plan=p, l5_scenario=str(scenario), l5_scenario_sha256=run_scenario.sha256_file(scenario),
                params_path=str(defaults), overrides=applied, harness_overrides=harness, executions=ex,
                window_steps=steps, window_stale=[i for i in steps if i not in fresh])
    r.report_path = str(Path(r.log_path).with_suffix(".report.txt"))
    write_report([s], None, r.report_path)
    return s


def run_sequence(card, scenario, out_dir, plugin_dir=DEFAULT_PLUGIN_DIR, *, overrides=None, extra_edit=None, seed=None,
                 root=ROOT, timeout_s=run_scenario.TIMEOUT_S):
    """run_step for every m of the scenario's m_sequence, one gz process (one SIL) each; one sequence report."""
    doc = l5s.load(scenario)
    runs = [run_step(card, scenario, m, out_dir, plugin_dir, overrides=overrides, extra_edit=extra_edit, seed=seed,
                     root=root, timeout_s=timeout_s) for m in l5s.values(doc)["m_sequence"]]
    stem = re.sub(r"_m\d+(_seed\d+)$", r"\1", Path(runs[0].run.world_path).stem)
    path = str(Path(out_dir) / f"{stem}_sequence.report.txt")
    write_report(runs, None, path)
    return runs, path


# ---- the run report -------------------------------------------------------------------------------------------------

def render_report(runs, evaluation=None):
    """run_scenario's run report of the runs, relabelled L5 truth-fed, with the plan, the overrides, the window's
    freshness and `evaluation` (a mapping) in its halving section."""
    s0 = runs[0]
    section = {
        "l5 plan (tools/sim/run_l5.py)": s0.plan.report(),
        "l5 parameter table (build)": s0.params_path,
        "l5 runs": {f"m{s.run.m}": {
            "sil overrides": {k: f"{t} {v}" for k, (t, v) in s.overrides.items()},
            "harness overrides": {k: f"{t} {v}" for k, (t, v) in s.harness_overrides.items()} or "(none)",
            "window host steps": f"{s.window_steps.start}..{s.window_steps.stop - 1}",
            "stale reads in the window": len(s.window_stale),
            "truth records": len(s.run.log["truths"]),
        } for s in runs},
        "t4": evaluation if evaluation else "(not evaluated)",
    }
    text = run_scenario.render_report([s.run for s in runs], section)
    first, rest = text.split("\n", 1)
    if first != "MARV L2 run report":
        raise run_scenario.RunError(f"unexpected run report header {first!r}")
    head = [f"MARV L5 run report: {LABEL}", "", LABEL_LINE, f"l5 scenario: {s0.l5_scenario}",
            f"l5 scenario sha256: {s0.l5_scenario_sha256}",
            "(the scenario line below is the L2-schema world input the runner wrote from it)"]
    return "\n".join(head) + "\n" + rest


def write_report(runs, evaluation=None, path=None):
    text = render_report(runs, evaluation)
    path = path or runs[0].run.report_path
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(text, encoding="utf-8", newline="\n")
    return text


# ---- T4 attitude chirp (decision 0006 F "T4 attitude chirp"; decision 0005 "T4 chirp margins") --------------------------
#
# Design quantities (chirp_design), from tools/card/attitude.py design() on the card, the budget, the scenario register and
# the committed SIM-7 uncertainty file SIM7_U; the build's att_kp, att_yaw_weight and att_loop_ratio must equal the design's
# (else the build is stale and refused):
#   band [w_lo, w_hi]   [min over the box loops of w_c / a, a max over the box loops of w_c]: w_c = theta / T_a the design
#                       crossover of the nominal loop and the four tau x J corners, on every axis; a = rate.design()["a"].
#   tau_held,a          as run_l4's: the largest single-axis torque the mixer delivers at the hover thrust request with the
#                       collective held (run_l4.tau_held).
#   G_tau(w)            max over the box loops and the rate executions i of the attitude period of |torque request| per unit
#                       of a steady sinusoid d of 1 rad/s added to the rate setpoint at every rate execution, in closed
#                       loop with the attitude law (gain k, held for N rate executions), in N m (steady_torque_peak).
#   A_a                 tau_held,a / max over the band of G_tau: the largest amplitude for which the design model's peak
#                       torque request over the band and the box stays inside the held-collective envelope.
# Plan: run_step's (origin execution a0, the chirp start and duration from the scenario, the window of executions
# a0 .. a0 + end).
#
# Identification (chirp_margin), from the log alone, as run_l4's (indirect closed-loop identification): y is the attitude
# error variable the law multiplies by its gain, recomputed in binary64 from the TRUTH quaternion of each attitude
# execution (attitude_error: -2 imag of the law's recomposed error quaternion, yaw over f32(w), so that u = -C y with C the
# law's linear gain k: f32(k) on roll and pitch, f32(f32(k)/f32(w)) f32(w) on yaw), d the chirp recomputed in binary64 at
# the execution's stamp. The loop is at T_a: G_m = Y/D, L = C G_m / (1 - C G_m), crossover |L| = 1 inside the band
# (run_l4.identify with kp = C, ki = 0 and the period T_a), PM = pi + arg L. The chirp is added at every rate execution
# while the sample is taken at the attitude executions; the half rate period this leaves between the two is common to the
# runs of one axis, so it cancels in E_H and U_A and is part of the reported measured - design nominal difference.

SIM7_U = ROOT / "design" / "measured" / "sim7_u" / "u.yaml"
CHIRP_NEED = ("att_kp", "att_yaw_weight", "att_loop_ratio", *l4.CHIRP_NEED)
MIXER_COLUMNS = l4.MIXER_COLUMNS
PEAK_GRID_POINTS = l4.rate.GRID_POINTS  # the log grid of the peak torque search (a method constant: rate.py's grid)


@functools.lru_cache(maxsize=None)
def attitude_design(card, root=str(ROOT), u_path=None):
    """attitude.design() on the card with the SIM-7 uncertainty file `u_path` (default SIM7_U)."""
    card_doc = schema.load_yaml(card)
    budget = schema.load_yaml(Path(root) / "design" / "budget.yaml")
    register = schema.load_yaml(Path(root) / "design" / "scenario_values.yaml")
    u = attitude.read_u(str(u_path or SIM7_U))
    return attitude.design(card_doc, budget, register, str(card), u)


def steady_torque_peak(rm, axis, jt, tau, k, n, omega):
    """max over the n rate executions of one attitude period of |J_a u_i| (N m) per unit steady sinusoid d at omega
    (rad/s) for the loop of `axis` at the corner (jt, tau) with the attitude gain k (section comment)."""
    T = rm["T"]
    kp, ki = attitude.axis_gains(rm, axis)
    j_a = rm["axes"][axis]["J"]
    a, b = attitude.rate_model(kp, ki, T, tau, jt)
    p, bs = attitude.lift(a, b, n)
    zt = cmath.exp(1j * omega * T)
    size = attitude.STATE_N
    v = [0j] * size
    for i in range(n):
        v = [sum(a[r][c] * v[c] for c in range(size)) + b[r] * zt ** i for r in range(size)]
    za = cmath.exp(1j * omega * T * n)
    m = [[(za if r == c else 0) - p[r][c] + (k * bs[r] if c == attitude.THETA else 0) for c in range(size)]
         for r in range(size)]
    x = attitude.csolve(m, v)
    r_hold = -k * x[attitude.THETA]
    best = 0.0
    for i in range(n):
        r_i = r_hold + zt ** i
        best = max(best, abs(j_a * (kp * (r_i - x[1]) + x[3] + ki * T * x[4])))
        x = [sum(a[r][c] * x[c] for c in range(size)) + b[r] * r_i for r in range(size)]
    return best


def peak_torque_gain(rm, axis, k, n, band, loops):
    """(max over the band and the box of steady_torque_peak, at omega, loop name): a log grid of PEAK_GRID_POINTS over the
    band, then a golden-section refinement around the largest grid point (run_l4.max_sensitivity's search)."""
    lo, hi = band
    grid = [lo * (hi / lo) ** (i / (PEAK_GRID_POINTS - 1)) for i in range(PEAK_GRID_POINTS)]
    best = (-math.inf, None, None)
    for name, jt, tau in loops:
        def f(w):
            return steady_torque_peak(rm, axis, jt, tau, k, n, w)
        vals = [f(w) for w in grid]
        i = max(range(PEAK_GRID_POINTS), key=vals.__getitem__)
        a_, b_ = grid[max(i - 1, 0)], grid[min(i + 1, PEAK_GRID_POINTS - 1)]
        c, d = b_ - l4.GOLDEN * (b_ - a_), a_ + l4.GOLDEN * (b_ - a_)
        fc, fd = f(c), f(d)
        for _ in range(l4.REFINE_ITERATIONS):
            if not a_ < c < d < b_:
                break
            if fc >= fd:
                b_, d, fd = d, c, fc
                c = b_ - l4.GOLDEN * (b_ - a_)
                fc = f(c)
            else:
                a_, c, fc = c, d, fd
                d = a_ + l4.GOLDEN * (b_ - a_)
                fd = f(d)
        for v, w in ((vals[i], grid[i]), (fc, c), (fd, d)):
            if v > best[0]:
                best = (v, w, name)
    return best


def chirp_design(card, params, thrust_n, root=ROOT, u_path=None):
    """The design quantities of the section comment, per axis, as a dict. Raises PlanError on a stale build."""
    res = attitude_design(str(card), str(root), u_path)
    if (params.get("att_kp"), params.get("att_yaw_weight"), params.get("att_loop_ratio")) != (
            res["k32"], res["w32"], res["N"]):
        raise PlanError([f"the build's att_kp, att_yaw_weight, att_loop_ratio {params.get('att_kp')!r}, "
                         f"{params.get('att_yaw_weight')!r}, {params.get('att_loop_ratio')!r} are not the design's "
                         f"{res['k32']!r}, {res['w32']!r}, {res['N']!r} (tools/card/attitude.py on {card}, "
                         f"{u_path or SIM7_U}): the build is stale"])
    rm = res["rate"]
    t_a, n = res["T_a"], res["N"]
    inputs = rm["inputs"]
    loops = l4.rate.corner_list(inputs["tau"], inputs["b_tau"], inputs["b_J"])
    crossover = {a: {} for a in l5s.AXES}
    design_pm = {a: {} for a in l5s.AXES}
    for axis, name, pm, theta, _, _ in res["final"]["detail"]:
        crossover[axis][name], design_pm[axis][name] = theta / t_a, pm
    every = [w for c in crossover.values() for w in c.values()]
    band = (min(every) / rm["a"], rm["a"] * max(every))
    f_min = params["rotor_thrust_coeff"] * params["idle_speed"] ** 2
    f_max = params["rotor_thrust_coeff"] * params["rotor_speed_max"] ** 2
    rows = l4._mixer_rows(params)
    axes = {}
    for axis in l5s.AXES:
        kk = attitude.axis_k(axis, params["att_kp"], params["att_yaw_weight"])
        g, g_at, g_loop = peak_torque_gain(rm, axis, kk, n, band, loops)
        held = l4.tau_held(rows, MIXER_COLUMNS.index(axis), thrust_n, f_min, f_max)
        if held is None or not held > 0:
            raise PlanError([f"the hover thrust {thrust_n!r} N leaves no held-collective torque on {axis}"])
        axes[axis] = {"k": kk, "peak_torque_per_rad_s": g, "peak_at_rad_s": g_at, "peak_loop": g_loop,
                      "tau_held_nm": held, "amplitude_rad_s": held / g}
    return {"T_a": t_a, "N": n, "a": rm["a"], "band": band, "design_pm": design_pm, "design_crossover": crossover,
            "pm_min": inputs["PM_min"], "loops": loops, "axes": axes, "result": res}


def canonical(q):
    return tuple(-x for x in q) if q[0] < 0 else tuple(q)


def attitude_error(q, w32):
    """(y_roll, y_pitch, y_yaw) of a TRUTH quaternion q [w, x, y, z] against the level setpoint q_sp = [1, 0, 0, 0]: minus
    the vector the attitude law (fw/attitude/include/marv/attitude/attitude_law.hpp) multiplies by its gain, in binary64:
    -2 imag(q_c) on roll and pitch, -2 imag(q_c).z / w32 on yaw, q_c the tilt-prioritised, yaw-weighted error quaternion."""
    n = math.sqrt(sum(x * x for x in q))
    w, x, y, z = canonical((q[0] / n, -q[1] / n, -q[2] / n, -q[3] / n))
    rho = math.hypot(w, z)
    if rho > 0:
        a, b = (w * x - y * z) / rho, (w * y + x * z) / rho
        qz = canonical((w / rho, 0.0, 0.0, z / rho))
        half = w32 * 2 * math.atan2(qz[3], qz[0]) / 2
        c, s = math.cos(half), math.sin(half)
        w, x, y, z = canonical((rho * c, c * a + s * b, c * b - s * a, rho * s))
    return -2 * x, -2 * y, -2 * z / w32


@dataclasses.dataclass
class ChirpRun:
    step: StepRun
    axis: str
    axis_index: int
    amp: float  # l5_chirp_amp_rad_s as flown (float32), rad/s
    w_lo: float
    w_hi: float
    t0_us: int
    dur_us: int
    k: float  # the loop gain C flown: the axis's effective linear gain
    design: dict
    stamps: list  # window attitude executions' stamps, us
    y: list  # the axis's attitude error variable at those executions
    d: list  # the chirp recomputed at those stamps, rad/s
    dshot_end_executions: int  # window attitude executions with some motor at the idle bound or kDshotThrottleMax

    @property
    def period_s(self):
        return float(self.step.plan.period_s)

    @property
    def window_stale(self):
        return self.step.window_stale


def flown_gain(params, overrides, axis):
    """The effective linear gain of `axis` the run flew: att_kp (or its harness override) on roll and pitch; on yaw the
    float law's f32(f32(k)/f32(w)) f32(w)."""
    k = float(overrides["att_kp"][1]) if "att_kp" in overrides else params["att_kp"]
    return attitude.axis_k(axis, k, params["att_yaw_weight"])


def run_chirp(card, scenario, m, out_dir, plugin_dir=DEFAULT_PLUGIN_DIR, *, amp_scale=1.0, halved=False, overrides=None,
              extra_edit=None, seed=None, root=ROOT, timeout_s=run_scenario.TIMEOUT_S, design_u=None):
    """One gz process of an L5 chirp scenario at m ticks per host step (run_step), with the amplitude scaled by amp_scale in
    (0, 1] and the duration halved when `halved`; extracts the window's y and d. `overrides` are harness sil_overrides.

    `design_u`, a SIM-7 uncertainty file (attitude.read_u), runs the chirp at the loop rate that file gives WITHOUT regenerating
    the product: attitude.design() on that U gives N and k (f32); the run flies att_loop_ratio = N and att_kp = k through
    sil_override, and the chirp's amplitude, band and window are the design's at that N (chirp_design; the window is the
    scenario's (duration + tail) in seconds, re-counted in attitude periods)."""
    if not 0 < amp_scale <= 1:
        raise PlanError([f"amp_scale {amp_scale!r} is not in (0, 1]"])
    vals = l5s.values(l5s.load(scenario))
    c = vals["script"]["chirp"]
    _, defaults = build_parameters(plugin_dir)
    params = l4.read_param_defaults(defaults)
    eff, harness, amp0, band, end = dict(params), dict(overrides or {}), c["amp_rad_s"], None, None
    if design_u is not None:
        res = attitude_design(str(card), str(root), str(design_u))
        eff.update(att_loop_ratio=res["N"], att_kp=res["k32"])
        harness.update({"att_loop_ratio": (I32, str(res["N"])), "att_kp": (F32, repr(res["k32"]))})
        design = chirp_design(card, eff, l4.r32(l4.hover_thrust(card, vals, root)), root, str(design_u))
        amp0, band = design["axes"][c["axis"]]["amplitude_rad_s"], design["band"]
        old = params["att_loop_ratio"] * vals["script"]["end_attitude_execution"]
        if old % res["N"]:
            raise PlanError([f"the scenario's window of {old} rate-period units is not a whole number of attitude "
                             f"periods at N = {res['N']}"])
        end = old // res["N"]
    doc_edit = None
    if amp_scale != 1 or halved or design_u is not None:
        def doc_edit(doc):
            ch = doc["script"]["chirp"]
            ch["amp_rad_s"]["value"] = l4.r32(l4.r32(amp0) * amp_scale)
            ch["duration_s"]["value"] = c["duration_s"] / 2 if halved else c["duration_s"]
            if band:
                ch["w_lo_rad_s"]["value"], ch["w_hi_rad_s"]["value"] = band
                doc["script"]["end_attitude_execution"]["value"] = end
    s = run_step(card, scenario, m, out_dir, plugin_dir, overrides=harness, extra_edit=extra_edit, seed=seed, root=root,
                 timeout_s=timeout_s, doc_edit=doc_edit)
    p = s.plan
    idx = l5s.AXES.index(c["axis"])
    ch = p.chirp
    amp, w_lo, w_hi = (l4.r32(ch[key]) for key in ("amp_rad_s", "w_lo_rad_s", "w_hi_rad_s"))
    lo, hi = l4.dshot_idle_bound(params)
    stamps, y, ends = [], [], 0
    for a in p.window:
        e = s.executions[a]
        if e["truth"] is None:
            raise run_scenario.RunError(f"no TRUTH record at attitude execution {a}")
        stamps.append(e["t_us"])
        y.append(attitude_error(e["truth"]["q_wxyz"], params["att_yaw_weight"])[idx])
        ends += any(x in (lo, hi) for x in s.run.log["ticks"][e["tick"]]["dshot"])
    d = [l4.chirp_value(t, amp, w_lo, w_hi, ch["t0_us"], ch["dur_us"]) for t in stamps]
    return ChirpRun(step=s, axis=c["axis"], axis_index=idx, amp=amp, w_lo=w_lo, w_hi=w_hi, t0_us=ch["t0_us"],
                    dur_us=ch["dur_us"], k=flown_gain(params, s.overrides, c["axis"]),
                    design=chirp_design(card, eff, p.thrust_n, root, str(design_u) if design_u is not None else None),
                    stamps=stamps, y=y, d=d, dshot_end_executions=ends)


# ---- the margin and U of a chirp axis (owner decision 20) --------------------------------------------------------------
#
# Yaw (and the L4 rule): PM is the (m = 1, A) run's, E_H = |PM(m = 1) - PM(m = 2)|, U_A = |PM(A) - PM(A/2)|.
# Roll and pitch (owner decision 20: the torque-envelope amplitude A_env leaves the linear regime, peak attitude error 2 at
# A_env): runs k = 1..5 at m = 1 and amplitude A_env / 2^k; PM = the minimum over the k that give a unique crossover, U_A =
# max - min of the PM over those k (the plateau's spread), E_H = |PM - PM(m = 2)| of the run at the minimum's k and amplitude,
# U_d that of that run. U = E_H + U_A + U_d on every axis.
PLATEAU_K = tuple(range(1, 6))
ROLL_PITCH = ("roll", "pitch")


def plateau_name(k):
    return f"k{k}"


def u_terms(axis, pm):
    """{"margin_run", "pm", "E_H", "U_A"} (rad) from `pm`, run name -> PM (rad) or None (no unique crossover): yaw's runs are
    m1, m2, half_amplitude; roll and pitch's are k1 .. k5 (m = 1) and m2 (m = 2, at the minimum's amplitude)."""
    if axis not in ROLL_PITCH:
        return {"margin_run": "m1", "pm": pm["m1"], "E_H": abs(pm["m1"] - pm["m2"]),
                "U_A": abs(pm["m1"] - pm["half_amplitude"])}
    plateau = {plateau_name(k): pm[plateau_name(k)] for k in PLATEAU_K if pm.get(plateau_name(k)) is not None}
    if len(plateau) < 2:
        raise PlanError([f"{axis}: {len(plateau)} of the amplitudes A_env/2^k give a unique crossover, the plateau needs two"])
    low = min(plateau, key=plateau.get)
    return {"margin_run": low, "pm": plateau[low], "E_H": abs(plateau[low] - pm["m2"]),
            "U_A": max(plateau.values()) - plateau[low]}


def chirp_margin(s):
    """run_l4.identify on one chirp run: C = the flown gain, the period T_a, the swept float32 band."""
    return l4.identify(s.y, s.d, s.period_s, s.k, 0.0, (s.w_lo, s.w_hi))


def float_input_term(s, margin):
    """U_d of a chirp run (run_l4's section "the chirp's float32 input term", on the attitude period): the PM error of
    recomputing d in binary64 while the composition computes it in float32, at the crossover of `margin`."""
    w = margin["crossover"]
    delta = [l4.chirp_value_f32(t, s.amp, s.w_lo, s.w_hi, s.t0_us, s.dur_us) - d for t, d in zip(s.stamps, s.d)]
    _, dm = l4.dtft_pair(s.y, s.d, w, s.period_s)
    rho = sum(abs(x) for x in delta) / abs(dm)
    e = rho * abs(1 + margin["L"])
    h = w * 2.0 ** l4.DIFF_LOG2_STEP
    up = l4.measured_loop(s.y, s.d, w + h, s.period_s, s.k, 0.0)
    down = l4.measured_loop(s.y, s.d, w - h, s.period_s, s.k, 0.0)
    kappa = abs(cmath.phase(up / down)) / abs(math.log(abs(up) / abs(down)))
    return {"U_d": math.asin(min(e, 1.0)) + e * kappa, "max |delta| / A": max(abs(x) for x in delta) / s.amp,
            "rho": rho, "e": e, "kappa": kappa}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", required=True)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--m", type=int, help="ticks per host step; omitted runs the scenario's m_sequence")
    ap.add_argument("--plugin-dir", default=str(DEFAULT_PLUGIN_DIR))
    args = ap.parse_args(argv)
    try:
        if args.m is None:
            runs, path = run_sequence(args.card, args.scenario, args.out_dir, args.plugin_dir)
        else:
            s = run_step(args.card, args.scenario, args.m, args.out_dir, args.plugin_dir)
            runs, path = [s], s.run.report_path
    except (scn.ScenarioError, run_scenario.RunError) as e:
        for line in getattr(e, "lines", [str(e)]):
            print(f"run_l5: {line}", file=sys.stderr)
        return 1
    print(f"report: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
