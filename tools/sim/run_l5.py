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
import dataclasses
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
import l5_scenario as l5s  # noqa: E402
import run_l4 as l4  # noqa: E402
import run_scenario  # noqa: E402
import scenario as scn  # noqa: E402

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
             root=ROOT, timeout_s=run_scenario.TIMEOUT_S, gyro_source=True, attitude_source=True):
    """One gz process of an L5 scenario at m ticks per host step; writes the world, the log and the run report into
    out_dir. `overrides` are harness sil_overrides, name -> (type, text). The two source flags leave the plugin element
    out of the world (the controls)."""
    doc = l5s.load(scenario)
    _, defaults = build_parameters(plugin_dir)
    params = l4.read_param_defaults(defaults)
    p = plan(doc, params, card, root)
    if m not in p.m_sequence:
        raise PlanError([f"m = {m} is not in the scenario's m_sequence {list(p.m_sequence)}"])
    harness = dict(overrides or {})
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
