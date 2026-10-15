#!/usr/bin/env python3
"""Runner of the quad L11 T4 scenarios (quad spec section 4 L11, docs/decisions/0020 section 5 "T4").

  run_l11.py --card <card> --scenario scenarios/quad/L11/<name>.yaml --out-dir <dir> [--m <int>] [--plugin-dir <dir>]

As a module: plan(), run_step(), run_sequence(), write_report(). Every run is labelled truth-fed, perfect-model, as run_l5's:
the rate loop's gyro sample, the attitude state and the vertical state are the plant's truth (<gyro_source>, <attitude_source>
and <vertical_source> truth), and truth and firmware share one card (core section 6): not a validation run.

The L5 runner's machinery is reused by import (tools/sim/run_l5.py: the world-input writer, the world edit, the L2-path run);
this module adds the L11 schema (tools/sim/l11_scenario.py), the plan on vertical executions and the VERTICAL records of the
log. m = 2 and 1 only, and only a divisor of the vertical divisor.

Live parameters. The plugin build's parameter table (MARV_GZ_PARAMS must be marv_params_l11_vertical_scripted, the host-gz-l11
build). Refused before any gz process starts: a scenario tick different from the build's, an m that does not divide the
vertical divisor rate_loop_divisor * vert_loop_ratio (a vertical sample would then be held from an earlier tick), and stamps
that do not fit the composition's i32 parameters.

Plan. D_v = rate_loop_divisor * vert_loop_ratio, t = the tick period (exact rational). The vertical group runs at ticks D_v n
(tick 0 included) and the SIL stamps tick j with floor(j num / den), so the run's vertical execution n is the T3 oracle's
execution n (the run starts at the hover trim: the rotors start at the card's hover speed, no settle phase) and its stamp is
the oracle's.
  segment stamps        l11_seg<i>_t_us = floor(D_v n_i num / den): the segment applies at execution n_i and not at the one before
  thrust disturbance    l11_dist_thrust_t_us = the stamp of its start execution; thrust_n is the number or, for "mass_conflict",
                        float32 of (m_card - m_conflict) g(phi, h0) (mass_conflict_thrust_n; the oracle's thrust_offset_n)
  torque pulse          l11_dist_torque_t_us = the stamp of its start execution, l11_dist_torque_dur_us = duration_s in us
  end                   vertical execution `end_vertical_execution`, the last of the window
  duration_ticks        the smallest multiple of every m of m_sequence above D_v * end, the last window tick
  vert_hover_thrust_n   float32 of m g(phi, h0), as run_l4.hover_thrust
  rotor_speed_rad_s     the card's hover rotor speed per motor (run_l5.hover_rotor_speeds)
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
import gen_plant_config as gpc  # noqa: E402
import l11_scenario as l11s  # noqa: E402
import run_l4 as l4  # noqa: E402
import run_l5  # noqa: E402
import run_scenario  # noqa: E402
import scenario as scn  # noqa: E402

DEFAULT_PLUGIN_DIR = ROOT / "build" / "host-gz-l11" / "sim" / "gz" / "plugin"
COMPOSITION_PARAMS = "marv_params_l11_vertical_scripted"
LABEL = "truth-fed, perfect-model"
LABEL_LINE = ("label: truth-fed, perfect-model. The rate loop's gyro sample, the attitude state and the vertical state are the "
              "plant's truth (<gyro_source>, <attitude_source> and <vertical_source> truth, decisions 0006 B and 0020) and "
              "truth and firmware share one card (core section 6): not a validation run")
VERTICAL_ELEMENT = "<vertical_source>truth</vertical_source>"
F32, I32 = l4.F32, l4.I32
I32_LIMIT_US = run_l5.I32_LIMIT_US
AXIS_CODE = {name: i for i, name in enumerate(l11s.AXES)}  # l11_dist_torque_axis: 0 roll, 1 pitch, 2 yaw
NAME_LABEL = l4.NAME_LABEL
PlanError = l4.PlanError


def build_parameters(plugin_dir):
    """(parameter set name, path of its param_defaults.cpp) of the build that holds plugin_dir; refuses a build that is not the
    host-gz-l11 build."""
    for d in [Path(plugin_dir).resolve(), *Path(plugin_dir).resolve().parents]:
        cache = d / "CMakeCache.txt"
        if cache.exists():
            m = l4.CACHE_PARAMS.search(cache.read_text(encoding="utf-8", errors="replace"))
            if not m:
                raise PlanError([f"{cache}: no MARV_GZ_PARAMS entry (not a gz build)"])
            if m.group(1) != COMPOSITION_PARAMS:
                raise PlanError([f"{cache}: MARV_GZ_PARAMS is {m.group(1)!r}, not {COMPOSITION_PARAMS!r}: the plugin is "
                                 "not the host-gz-l11 build"])
            return m.group(1), d / "generated" / m.group(1) / "marv" / "params" / "param_defaults.cpp"
    raise PlanError([f"{plugin_dir}: no CMakeCache.txt above the plugin directory"])


def mass_conflict_thrust_n(card, vals, root=ROOT):
    """(m_card - m_conflict) g(phi, h0) in double: the card's mass minus its open conflict value (exactly one), times the
    normal gravity at the scenario site (run_l4.hover_thrust's mass and gravity). The oracle's thrust_offset_n before its
    float32 cast (vertical_t3_oracle.py refresh_inputs)."""
    card_doc, profile = gpc.load_linted(card, root)
    m_card = gpc.config_from(card_doc, profile, card, root)[0]["mass_kg"]
    conflicts = {float(c["value"]) for c in card_doc["mass"].get("conflict", []) or [] if c.get("state") == "open"}
    if len(conflicts) != 1:
        raise PlanError([f"{card}: the mass must carry exactly one open conflict value, found {sorted(conflicts)!r}"])
    (m_conflict,) = conflicts
    return (m_card - m_conflict) * l4.hover.normal_gravity(vals["site_latitude_rad"], vals["site_height_m"])


# ---- the plan -------------------------------------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Plan:
    num_us: int
    den: int
    divisor: int  # rate_loop_divisor
    ratio: int  # vert_loop_ratio
    tick_s: Fraction
    period_s: Fraction  # T_v
    end: int  # last window execution
    segments: tuple  # (start, stamp_us, z_ref_m) per segment
    thrust_disturbance: dict | None  # {thrust_n (float), t0_us}
    torque_pulse: dict | None  # {axis (code), torque_nm, t0_us, dur_us}
    duration_ticks: int
    m_sequence: tuple
    thrust_n: float
    thrust_n_double: float
    rotor_speed_rad_s: tuple

    @property
    def vert_divisor(self):
        return self.divisor * self.ratio

    def tick_of(self, n):
        return self.vert_divisor * n

    def stamp_us(self, n):
        return (self.vert_divisor * n * self.num_us) // self.den

    @property
    def window(self):
        return range(0, self.end + 1)

    def overrides(self):
        """The composition parameters of the run: name -> (type, text), the text exact for its type."""
        out = {"vert_hover_thrust_n": (F32, repr(self.thrust_n)), "l11_seg_count": (I32, str(len(self.segments)))}
        for i, (_, stamp, z_ref) in enumerate(self.segments, 1):
            out[f"l11_seg{i}_t_us"] = (I32, str(stamp))
            out[f"l11_seg{i}_z_ref_m"] = (F32, repr(l4.r32(z_ref)))
        if self.thrust_disturbance:
            d = self.thrust_disturbance
            out.update({"l11_dist_thrust_n": (F32, repr(l4.r32(d["thrust_n"]))), "l11_dist_thrust_t_us": (I32, str(d["t0_us"]))})
        if self.torque_pulse:
            d = self.torque_pulse
            out.update({"l11_dist_torque_axis": (I32, str(d["axis"])), "l11_dist_torque_nm": (F32, repr(l4.r32(d["torque_nm"]))),
                        "l11_dist_torque_t_us": (I32, str(d["t0_us"])), "l11_dist_torque_dur_us": (I32, str(d["dur_us"]))})
        return out

    def report(self):
        return {
            "tick period": f"{self.num_us}/{self.den} us (build and scenario)",
            "rate_loop_divisor": self.divisor, "vert_loop_ratio": self.ratio, "vertical divisor": self.vert_divisor,
            "vertical period T_v": f"{self.period_s} s",
            "segments (start, stamp us, z_ref m)": [list(s) for s in self.segments],
            "thrust disturbance": self.thrust_disturbance or "(none)", "torque pulse": self.torque_pulse or "(none)",
            "window vertical executions": f"{self.window.start}..{self.window.stop - 1}",
            "duration_ticks": self.duration_ticks, "m_sequence": list(self.m_sequence),
            "vert_hover_thrust_n (float32)": self.thrust_n, "m g(phi, h0) (double)": self.thrust_n_double,
            "initial rotor speed (rad/s)": list(self.rotor_speed_rad_s),
        }


def plan(doc, params, card, root=ROOT):
    """The derived plan of a valid L11 scenario against the build's parameters (module docstring). Raises PlanError."""
    vals = l11s.values(doc)
    script = vals["script"]
    need = ("tick_period_num_us", "tick_period_den", "rate_loop_divisor", "vert_loop_ratio")
    missing = [k for k in need if k not in params]
    if missing:
        raise PlanError([f"the build's parameter table lacks {missing}"])
    num, den = params["tick_period_num_us"], params["tick_period_den"]
    divisor, ratio = params["rate_loop_divisor"], params["vert_loop_ratio"]
    fails = []
    if (vals["tick_period_num_us"], vals["tick_period_den"]) != (num, den):
        fails.append(f"tick period {vals['tick_period_num_us']}/{vals['tick_period_den']} us differs from the build's "
                     f"{num}/{den} us: the rate loop would panic on the execution spacing")
    if divisor < 1 or ratio < 1:
        fails.append(f"the build's rate_loop_divisor {divisor} or vert_loop_ratio {ratio} is below 1")
    else:
        for m in vals["m_sequence"]:
            if (divisor * ratio) % m:
                fails.append(f"m = {m} does not divide the vertical divisor rate_loop_divisor * vert_loop_ratio = "
                             f"{divisor * ratio}: a vertical sample would be held from an earlier tick")
    if fails:
        raise PlanError(fails)

    vert_div = divisor * ratio
    tick = Fraction(num, den * scn.MICROSECONDS_PER_SECOND)

    def stamp(n):
        return (vert_div * n * num) // den

    end = script["end_vertical_execution"]
    segments = tuple((s["start_vertical_execution"], stamp(s["start_vertical_execution"]), s["z_ref_m"])
                     for s in script["segments"])
    disturbance = pulse = None
    thrust = l4.hover_thrust(card, vals, root)
    if "thrust_disturbance" in script:
        d = script["thrust_disturbance"]
        n_dist = mass_conflict_thrust_n(card, vals, root) if d["thrust_n"] == l11s.MASS_CONFLICT else d["thrust_n"]
        disturbance = {"thrust_n": n_dist, "t0_us": stamp(d["start_vertical_execution"])}
    if "torque_pulse" in script:
        d = script["torque_pulse"]
        dur = Fraction(repr(d["duration_s"])) * scn.MICROSECONDS_PER_SECOND
        if dur.denominator != 1:
            fails.append(f"torque pulse duration_s {d['duration_s']!r} is not a whole number of microseconds")
        else:
            pulse = {"axis": AXIS_CODE[d["axis"]], "torque_nm": d["torque_nm"], "t0_us": stamp(d["start_vertical_execution"]),
                     "dur_us": int(dur)}
    stamps = [s[1] for s in segments] + [x["t0_us"] for x in (disturbance, pulse) if x] + ([pulse["dur_us"]] if pulse else [])
    if any(s >= I32_LIMIT_US for s in stamps):
        fails.append(f"a stamp or duration does not fit the composition's i32 parameters (limit {I32_LIMIT_US} us)")
    if fails:
        raise PlanError(fails)
    lcm = math.lcm(*vals["m_sequence"])
    duration = ((vert_div * end) // lcm + 1) * lcm
    return Plan(num_us=num, den=den, divisor=divisor, ratio=ratio, tick_s=tick, period_s=vert_div * tick, end=end,
                segments=segments, thrust_disturbance=disturbance, torque_pulse=pulse, duration_ticks=duration,
                m_sequence=tuple(vals["m_sequence"]), thrust_n=l4.r32(thrust), thrust_n_double=thrust,
                rotor_speed_rad_s=tuple(run_l5.hover_rotor_speeds(card, vals, root)))


# ---- the world ------------------------------------------------------------------------------------------------------

def add_vertical(text):
    """The world text with <vertical_source>truth</vertical_source> inside the lockstep plugin element."""
    blocks = l4.PLUGIN_BLOCK.findall(text)
    if len(blocks) != 1:
        raise run_scenario.RunError(f"the world holds {len(blocks)} marv_gz_lockstep plugin elements, expected 1")
    return text.replace(blocks[0], blocks[0].replace("</plugin>", f"  {VERTICAL_ELEMENT}\n</plugin>"), 1)


# ---- running --------------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class StepRun:
    run: run_scenario.RunResult
    plan: Plan
    l11_scenario: str
    l11_scenario_sha256: str
    params_path: str
    overrides: dict
    harness_overrides: dict
    executions: list  # per vertical execution n: {"n", "tick", "t_us", "vertical" (the VERTICAL record or None), "fresh"}
    window_steps: range  # host steps whose reads cover the window's vertical executions
    window_stale: list  # host steps of window_steps whose raw read equals the previous step's


def executions(log, p, m):
    fresh = set(run_scenario.fresh_steps(log))
    out = []
    for tick in range(0, len(log["ticks"]), p.vert_divisor):
        t = log["ticks"][tick]
        out.append({"n": tick // p.vert_divisor, "tick": t["tick"], "t_us": t["sil_t_us"], "vertical": t.get("vertical"),
                    "fresh": tick // m in fresh})
    return out


def run_step(card, scenario, m, out_dir, plugin_dir=DEFAULT_PLUGIN_DIR, *, overrides=None, extra_edit=None, seed=None,
             root=ROOT, timeout_s=run_scenario.TIMEOUT_S, doc_edit=None):
    """One gz process of an L11 scenario at m ticks per host step; writes the world, the log and the run report into
    out_dir. `overrides` are harness sil_overrides, name -> (type, text) (the controls: the loop disabled). `doc_edit`, when
    given, is called on the loaded scenario document before the plan. `extra_edit` is a callable on the world text."""
    doc = l11s.load(scenario)
    if doc_edit is not None:
        doc_edit(doc)
    _, defaults = build_parameters(plugin_dir)
    params = l4.read_param_defaults(defaults)
    harness = dict(overrides or {})
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
    l2_path.write_text(yaml.safe_dump(run_l5.l2_scenario_doc(l11s.l5_view(doc), p, stem, Path(scenario).name, card),
                                      sort_keys=False), encoding="utf-8")

    def edit(text):
        text = add_vertical(text)
        return extra_edit(text) if extra_edit is not None else text

    r = run_scenario._run(card, l2_path, seed, m, "test", None, out_dir, plugin_dir, run_l5.world_edit(applied, edit),
                          None, root, Path(root) / "design" / "budget.yaml", timeout_s, None)
    ex = executions(r.log, p, m)
    fresh = set(run_scenario.fresh_steps(r.log))
    steps = range(p.tick_of(p.window.start) // m, p.tick_of(p.window.stop - 1) // m + 1)
    s = StepRun(run=r, plan=p, l11_scenario=str(scenario), l11_scenario_sha256=run_scenario.sha256_file(scenario),
                params_path=str(defaults), overrides=applied, harness_overrides=harness, executions=ex,
                window_steps=steps, window_stale=[i for i in steps if i not in fresh])
    r.report_path = str(Path(r.log_path).with_suffix(".report.txt"))
    write_report([s], None, r.report_path)
    return s


def run_sequence(card, scenario, out_dir, plugin_dir=DEFAULT_PLUGIN_DIR, *, overrides=None, extra_edit=None, seed=None,
                 root=ROOT, timeout_s=run_scenario.TIMEOUT_S):
    """run_step for every m of the scenario's m_sequence, one gz process (one SIL) each; one sequence report."""
    doc = l11s.load(scenario)
    runs = [run_step(card, scenario, m, out_dir, plugin_dir, overrides=overrides, extra_edit=extra_edit, seed=seed,
                     root=root, timeout_s=timeout_s) for m in l11s.values(doc)["m_sequence"]]
    stem = re.sub(r"_m\d+(_seed\d+)$", r"\1", Path(runs[0].run.world_path).stem)
    path = str(Path(out_dir) / f"{stem}_sequence.report.txt")
    write_report(runs, None, path)
    return runs, path


# ---- the run report -------------------------------------------------------------------------------------------------

def render_report(runs, evaluation=None):
    """run_scenario's run report of the runs, relabelled L11 truth-fed, with the plan, the overrides, the window's freshness
    and `evaluation` (a mapping) in its halving section."""
    s0 = runs[0]
    section = {
        "l11 plan (tools/sim/run_l11.py)": s0.plan.report(),
        "l11 parameter table (build)": s0.params_path,
        "l11 runs": {f"m{s.run.m}": {
            "sil overrides": {k: f"{t} {v}" for k, (t, v) in s.overrides.items()},
            "harness overrides": {k: f"{t} {v}" for k, (t, v) in s.harness_overrides.items()} or "(none)",
            "window host steps": f"{s.window_steps.start}..{s.window_steps.stop - 1}",
            "stale reads in the window": len(s.window_stale),
            "truth records": len(s.run.log["truths"]),
            "vertical records": sum(1 for t in s.run.log["ticks"] if "vertical" in t),
        } for s in runs},
        "t4": evaluation if evaluation else "(not evaluated)",
    }
    text = run_scenario.render_report([s.run for s in runs], section)
    first, rest = text.split("\n", 1)
    if first != "MARV L2 run report":
        raise run_scenario.RunError(f"unexpected run report header {first!r}")
    head = [f"MARV L11 run report: {LABEL}", "", LABEL_LINE, f"l11 scenario: {s0.l11_scenario}",
            f"l11 scenario sha256: {s0.l11_scenario_sha256}",
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
            print(f"run_l11: {line}", file=sys.stderr)
        return 1
    print(f"report: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
