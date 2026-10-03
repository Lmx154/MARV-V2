#!/usr/bin/env python3
"""Runner of the quad L4 T4 step scenarios (quad spec section 4 L4, docs/decisions/0005 "T4" and "Truth gyro").

  run_l4.py --card <card> --scenario scenarios/quad/L04/<step>.yaml --out-dir <dir> [--m <int>] [--plugin-dir <dir>]

As a module: plan(), run_step(), run_sequence(), write_report(). Every run is labelled truth-fed, perfect-model: the
rate loop's gyro sample is the plant's truth body rate and truth and firmware share one card (core section 6), so no
run of this runner is a validation run. The label is in every log and world name (<vehicle>_<scenario>_truth_fed_
perfect_model_test_m<m>_seed<seed>) and heads every run report.

World. The L2 path is reused unchanged: the L4 scenario is written out as an L2 scenario (tools/sim/scenario.py; its
command is the placeholder DShot 0 on every motor) beside the run's files, and run_scenario's world generation and gz
process (gen_world.generate, run_gz_process, the exit-code rule, the log reader and the stale-step report) run it. An
sdf_edit then removes the four L2 `ol_dshot_m*` sil_override elements that gen_world always writes (the l4_rate_scripted
parameter set has no such parameter), and adds <gyro_source>truth</gyro_source> and one sil_override per composition
parameter of the plan (below), then the caller's harness overrides (the negative controls; a name given twice is
refused). The plugin is the host-gz-l4 build (MARV_GZ_SIL = marv_sil_l4_rate_scripted).

Live parameters. The plugin build's parameter table, <build>/generated/<MARV_GZ_PARAMS>/marv/params/param_defaults.cpp
(MARV_GZ_PARAMS from that build's CMakeCache.txt, which must be marv_params_l4_rate_scripted), gives the values the
firmware in the plugin runs with. Refused before any gz process starts: a scenario tick different from the build's
tick_period_num_us / tick_period_den (the rate loop panics when an execution's spacing differs from its design period),
an m that does not divide rate_loop_divisor (an execution would then read a sample held from an earlier tick), and a
step rate whose float32 value is not the build's rate_max_<axis>.

Plan (derived values, each with its rule). D = rate_loop_divisor, t = the tick period (exact rational), T = D t. The rate
group runs at ticks D k (tick 0 included, the composition), and the SIL stamps tick j with floor(j num / den) us.
  step execution k0   the smallest k >= 2 with D k t >= settle_s and D (k - 1) num = 0 (mod den). The second condition
                      makes the stamps of executions k0 - 1, k0, ... the T3 oracle's stamps of executions 1, 2, ... plus
                      one integer, so every dt of the window is the oracle's dt.
  segment stamp       l4_seg1_t_us = floor(D k0 num / den): the setpoint is first seen at execution k0 (the composition
                      applies a segment for stamps >= its t_us, and execution k0 - 1's stamp is smaller).
  window              oracle execution n = 1..N is execution k = k0 + n - 2: n = 1 is the last execution before the step
                      (the oracle's seed execution, zero output), n = 2 the first that sees it.
  N                   1 + ceil(horizon_tau_ref tau_ref / T), tau_ref = the build's rate_tau_ref_<axis>, in exact rational
                      arithmetic (the T3 horizon rule).
  duration_ticks      the smallest multiple of every m of m_sequence above D (k0 + N - 2), the last window tick.
  l4_thrust_n         float32 of m g(phi, h0), the card's mass and tools/sim/hover.py normal_gravity: the hover thrust as
                      L2's hover command derives it (hover.bracket).
  l4_seg1_<axis>      float32 of the step rate on the stepped axis, 0 on the others; l4_seg_count = 1.
"""

from __future__ import annotations

import argparse
import cmath
import dataclasses
import functools
import math
import re
import struct
import subprocess
import sys
from fractions import Fraction
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "card"))
import gen_plant_config as gpc  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import hover  # noqa: E402
import l4_scenario as l4s  # noqa: E402
import lint  # noqa: E402
import lockstep_log  # noqa: E402
import prim_constants  # noqa: E402
import rate  # noqa: E402
import rate_lead  # noqa: E402
import run_scenario  # noqa: E402
import scenario as scn  # noqa: E402
import schema  # noqa: E402

DEFAULT_PLUGIN_DIR = ROOT / "build" / "host-gz-l4" / "sim" / "gz" / "plugin"
COMPOSITION_PARAMS = "marv_params_l4_rate_scripted"
LABEL = "truth-fed, perfect-model"
NAME_LABEL = "truth_fed_perfect_model"
LABEL_LINE = (f"label: {LABEL}. The rate loop's gyro sample is the plant's truth body rate (<gyro_source>truth"
              "</gyro_source>, decision 0005 'Truth gyro') and truth and firmware share one card (core section 6): "
              "not a validation run")
GYRO_ELEMENT = "<gyro_source>truth</gyro_source>"
L2_DSHOT_OVERRIDE = re.compile(r'\s*<sil_override param="ol_dshot_m\d+" type="i32">[^<]*</sil_override>')
PLUGIN_BLOCK = re.compile(r'<plugin filename="marv_gz_lockstep".*?</plugin>', flags=re.S)
PARAM_ENTRY = re.compile(r"// (\w+)\n\s*\{\{ParamType::(F32|I32), ([^,]+), (-?\d+)\}")
CACHE_PARAMS = re.compile(r"^MARV_GZ_PARAMS:[A-Z]+=(.*)$", flags=re.M)
F32, I32 = "f32", "i32"
IMU = struct.Struct("<7fI")  # marv_imu_meas: gyro xyz, accel xyz, temperature, flags (marv_sil.h)
L2_PLACEHOLDER_DSHOT = [0] * scn.MOTORS


class PlanError(scn.ScenarioError):
    """A scenario that does not fit the plugin build; `lines` are the findings."""


def r32(x):
    """x rounded to float32 (round to nearest), as a Python float."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


# ---- the build under test -------------------------------------------------------------------------------------------

def build_parameters(plugin_dir):
    """(parameter set name, path of its param_defaults.cpp) of the build that holds plugin_dir."""
    for d in [Path(plugin_dir).resolve(), *Path(plugin_dir).resolve().parents]:
        cache = d / "CMakeCache.txt"
        if cache.exists():
            m = CACHE_PARAMS.search(cache.read_text(encoding="utf-8", errors="replace"))
            if not m:
                raise PlanError([f"{cache}: no MARV_GZ_PARAMS entry (not a gz build)"])
            name = m.group(1)
            if name != COMPOSITION_PARAMS:
                raise PlanError([f"{cache}: MARV_GZ_PARAMS is {name!r}, not {COMPOSITION_PARAMS!r}: the plugin is not "
                                 "the host-gz-l4 build"])
            return name, d / "generated" / name / "marv" / "params" / "param_defaults.cpp"
    raise PlanError([f"{plugin_dir}: no CMakeCache.txt above the plugin directory"])


def read_param_defaults(path):
    """dict name -> value of a generated parameter table: int for I32, the float32 value (as a Python float) for F32."""
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as e:
        raise PlanError([f"{path}: {e}"]) from e
    out = {}
    for name, kind, f32, i32 in PARAM_ENTRY.findall(text):
        out[name] = int(i32) if kind == "I32" else r32(float(f32.rstrip("f")))
    if not out:
        raise PlanError([f"{path}: no parameter entries"])
    return out


# ---- the plan -------------------------------------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Plan:
    axis: str
    axis_index: int
    num_us: int
    den: int
    divisor: int
    tick_s: Fraction
    period_s: Fraction
    settle_s: float
    horizon_tau_ref: float
    tau_ref_s: float
    step_execution: int
    seg_t_us: int
    n_exec: int
    duration_ticks: int
    m_sequence: tuple
    rate_rad_s: float
    thrust_n: float
    thrust_n_double: float

    def tick_of(self, k):
        return self.divisor * k

    def stamp_us(self, k):
        return (self.divisor * k * self.num_us) // self.den

    def execution_of(self, n):
        """The execution k of oracle execution n (1-based)."""
        return self.step_execution + n - 2

    @property
    def window(self):
        """Executions of oracle n = 1..N."""
        return range(self.execution_of(1), self.execution_of(self.n_exec) + 1)

    def overrides(self):
        """The composition parameters of the run: name -> (type, text), the text exact for its type."""
        out = {"l4_thrust_n": (F32, repr(self.thrust_n)), "l4_seg_count": (I32, "1"),
               "l4_seg1_t_us": (I32, str(self.seg_t_us))}
        for i, a in enumerate(l4s.AXES):
            out[f"l4_seg1_{a}"] = (F32, repr(self.rate_rad_s if i == self.axis_index else 0.0))
        return out

    def report(self):
        return {
            "axis": self.axis, "tick period": f"{self.num_us}/{self.den} us (build and scenario)",
            "rate_loop_divisor D": self.divisor, "rate-loop period T": f"{self.period_s} s",
            "settle_s": self.settle_s, "horizon_tau_ref": self.horizon_tau_ref, "tau_ref_s (build)": self.tau_ref_s,
            "step execution k0": self.step_execution, "step tick": self.tick_of(self.step_execution),
            "l4_seg1_t_us": self.seg_t_us, "window executions": f"{self.window.start}..{self.window.stop - 1}",
            "N (oracle executions)": self.n_exec, "duration_ticks": self.duration_ticks,
            "m_sequence": list(self.m_sequence), "step rate (float32)": self.rate_rad_s,
            "l4_thrust_n (float32)": self.thrust_n, "m g(phi, h0) (double)": self.thrust_n_double,
        }


def hover_thrust(card, vals, root=ROOT):
    """m g(phi, h0) in double: the card's mass and hover.normal_gravity, as hover.bracket derives the hover command."""
    card_doc, profile = gpc.load_linted(card, root)
    cfg, _ = gpc.config_from(card_doc, profile, card, root)
    return cfg["mass_kg"] * hover.normal_gravity(vals["site_latitude_rad"], vals["site_height_m"])


def plan(doc, params, card, root=ROOT):
    """The derived plan of a valid L4 scenario against the build's parameters (module docstring). Raises PlanError."""
    vals = l4s.values(doc)
    step = vals["step"]
    axis = step["axis"]
    fails = []
    need = ("tick_period_num_us", "tick_period_den", "rate_loop_divisor", f"rate_tau_ref_{axis}", f"rate_max_{axis}")
    missing = [k for k in need if k not in params]
    if missing:
        raise PlanError([f"the build's parameter table lacks {missing}"])
    num, den, divisor = params["tick_period_num_us"], params["tick_period_den"], params["rate_loop_divisor"]
    if (vals["tick_period_num_us"], vals["tick_period_den"]) != (num, den):
        fails.append(f"tick period {vals['tick_period_num_us']}/{vals['tick_period_den']} us differs from the build's "
                     f"{num}/{den} us: the rate loop would panic on the execution spacing")
    if divisor < 1:
        fails.append(f"the build's rate_loop_divisor {divisor} is below 1")
    for m in vals["m_sequence"]:
        if divisor >= 1 and divisor % m:
            fails.append(f"m = {m} does not divide rate_loop_divisor = {divisor}: an execution would read a held sample")
    rate = r32(step["rate_rad_s"])
    if rate != params[f"rate_max_{axis}"]:
        fails.append(f"step rate float32 {rate!r} differs from the build's rate_max_{axis} {params[f'rate_max_{axis}']!r}")
    if fails:
        raise PlanError(fails)

    tick = Fraction(num, den * scn.MICROSECONDS_PER_SECOND)
    period = divisor * tick
    k0 = max(2, math.ceil(Fraction(step["settle_s"]) / period))
    for _ in range(den):
        if (divisor * (k0 - 1) * num) % den == 0:
            break
        k0 += 1
    else:
        raise PlanError([f"no step execution keeps the T3 stamp phase within {den} executions of settle_s"])
    tau_ref = params[f"rate_tau_ref_{axis}"]
    n_exec = 1 + math.ceil(Fraction(step["horizon_tau_ref"]) * Fraction(tau_ref) / period)
    lcm = math.lcm(*vals["m_sequence"])
    last_tick = divisor * (k0 + n_exec - 2)
    duration = (last_tick // lcm + 1) * lcm
    thrust = hover_thrust(card, vals, root)
    return Plan(axis=axis, axis_index=l4s.AXES.index(axis), num_us=num, den=den, divisor=divisor, tick_s=tick,
                period_s=period, settle_s=step["settle_s"], horizon_tau_ref=step["horizon_tau_ref"], tau_ref_s=tau_ref,
                step_execution=k0, seg_t_us=(divisor * k0 * num) // den, n_exec=n_exec, duration_ticks=duration,
                m_sequence=tuple(vals["m_sequence"]), rate_rad_s=rate, thrust_n=r32(thrust), thrust_n_double=thrust)


# ---- the world ------------------------------------------------------------------------------------------------------

def l2_scenario_doc(doc, p, stem, source):
    """The L2-schema scenario that gen_world reads: the L4 scenario's site, seed, tick and initial state, m_sequence and
    the plan's duration, and a placeholder DShot 0 command (removed from the world by world_edit)."""
    vals = l4s.values(doc)

    def entry(value, unit, key):
        return {"value": value, "unit": unit, "label": "derived", "rule": f"{key} of {source} (tools/sim/run_l4.py)"}

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
                           "rule": "the plan's duration_ticks (tools/sim/run_l4.py plan)"},
        "initial_state": {k: entry(st[k] if k in st else doc["initial_state"][k]["value"],
                                   doc["initial_state"][k]["unit"], f"initial_state.{k}")
                          for k in scn.STATE_FIELDS + tuple(k for k in scn.OPTIONAL_STATE if k in doc["initial_state"])},
        "command": {"dshot": {"value": L2_PLACEHOLDER_DSHOT, "unit": "1", "label": "scenario",
                              "rationale": "placeholder: world_edit removes the L2 ol_dshot_m* overrides; the "
                                           "l4_rate_scripted composition commands the motors"}},
    }


def world_edit(overrides, extra_edit=None):
    """The sdf_edit of an L4 run: drop the four L2 DShot overrides, add the truth-gyro element and `overrides`
    (name -> (type, text)) inside the lockstep plugin element, then apply `extra_edit` (a callable on the text)."""
    def edit(text):
        text, n = L2_DSHOT_OVERRIDE.subn("", text)
        if n != scn.MOTORS:
            raise run_scenario.RunError(f"the world holds {n} L2 ol_dshot_m* overrides, expected {scn.MOTORS}")
        blocks = PLUGIN_BLOCK.findall(text)
        if len(blocks) != 1:
            raise run_scenario.RunError(f"the world holds {len(blocks)} marv_gz_lockstep plugin elements, expected 1")
        add = "".join(f'\n  <sil_override param="{k}" type="{t}">{v}</sil_override>' for k, (t, v) in overrides.items())
        block = blocks[0].replace("</plugin>", f"{add}\n  {GYRO_ELEMENT}\n</plugin>")
        text = text.replace(blocks[0], block, 1)
        return extra_edit(text) if extra_edit is not None else text
    return edit


# ---- running --------------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class StepRun:
    run: run_scenario.RunResult
    plan: Plan
    l4_scenario: str
    l4_scenario_sha256: str
    params_path: str
    overrides: dict
    harness_overrides: dict
    executions: list  # per execution k: {"k", "tick", "t_us", "gyro" (3 floats), "flags", "fresh"}
    window_steps: range  # host steps whose reads cover the window's executions
    window_stale: list  # host steps of window_steps whose raw read equals the previous step's


def executions(log, p, m):
    fresh = set(run_scenario.fresh_steps(log))
    out = []
    for tick in range(0, len(log["ticks"]), p.divisor):
        k = tick // p.divisor
        t = log["ticks"][tick]
        v = IMU.unpack(t["imu"])
        out.append({"k": k, "tick": t["tick"], "t_us": t["sil_t_us"], "gyro": v[:3], "flags": v[7],
                    "fresh": p.tick_of(k) // m in fresh})
    return out


def run_step(card, scenario, m, out_dir, plugin_dir=DEFAULT_PLUGIN_DIR, *, overrides=None, extra_edit=None,
             seed=None, root=ROOT, timeout_s=run_scenario.TIMEOUT_S):
    """One gz process of an L4 step scenario at m ticks per host step; writes the world, the log and the run report
    (the report's evaluation section empty) into out_dir. `overrides` are harness sil_overrides, name -> (type, text)."""
    doc = l4s.load(scenario)
    _, defaults = build_parameters(plugin_dir)
    params = read_param_defaults(defaults)
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
    r = run_scenario._run(card, l2_path, seed, m, "test", None, out_dir, plugin_dir, world_edit(applied, extra_edit),
                          None, root, Path(root) / "design" / "budget.yaml", timeout_s)
    ex = executions(r.log, p, m)
    fresh = set(run_scenario.fresh_steps(r.log))
    steps = range(p.tick_of(p.window.start) // m, p.tick_of(p.window.stop - 1) // m + 1)
    s = StepRun(run=r, plan=p, l4_scenario=str(scenario), l4_scenario_sha256=run_scenario.sha256_file(scenario),
                params_path=str(defaults), overrides=applied, harness_overrides=harness, executions=ex,
                window_steps=steps, window_stale=[i for i in steps if i not in fresh])
    r.report_path = str(Path(r.log_path).with_suffix(".report.txt"))
    write_report([s], None, r.report_path)
    return s


def run_sequence(card, scenario, out_dir, plugin_dir=DEFAULT_PLUGIN_DIR, *, overrides=None, extra_edit=None, seed=None,
                 root=ROOT, timeout_s=run_scenario.TIMEOUT_S):
    """run_step for every m of the scenario's m_sequence, one gz process (one SIL) each; one sequence report."""
    doc = l4s.load(scenario)
    runs = [run_step(card, scenario, m, out_dir, plugin_dir, overrides=overrides, extra_edit=extra_edit, seed=seed,
                     root=root, timeout_s=timeout_s) for m in l4s.values(doc)["m_sequence"]]
    stem = re.sub(r"_m\d+(_seed\d+)$", r"\1", Path(runs[0].run.world_path).stem)
    path = str(Path(out_dir) / f"{stem}_sequence.report.txt")
    write_report(runs, None, path)
    return runs, path


# ---- the run report -------------------------------------------------------------------------------------------------

def render_report(runs, evaluation=None):
    """run_scenario's run report of the runs, relabelled L4 truth-fed, with the plan, the overrides, the window's
    freshness and `evaluation` (the T4 step verdicts, a mapping) in its halving section."""
    s0 = runs[0]
    section = {
        "l4 plan (tools/sim/run_l4.py)": s0.plan.report(),
        "l4 parameter table (build)": s0.params_path,
        "l4 runs": {f"m{s.run.m}": {
            "sil overrides": {k: f"{t} {v}" for k, (t, v) in s.overrides.items()},
            "harness overrides": {k: f"{t} {v}" for k, (t, v) in s.harness_overrides.items()} or "(none)",
            "window host steps": f"{s.window_steps.start}..{s.window_steps.stop - 1}",
            "stale reads in the window": len(s.window_stale),
        } for s in runs},
        "t4 steps": evaluation if evaluation else "(not evaluated)",
    }
    text = run_scenario.render_report([s.run for s in runs], section)
    first, rest = text.split("\n", 1)
    if first != "MARV L2 run report":
        raise run_scenario.RunError(f"unexpected run report header {first!r}")
    head = [
        f"MARV L4 run report: {LABEL}",
        "",
        LABEL_LINE,
        f"l4 scenario: {s0.l4_scenario}",
        f"l4 scenario sha256: {s0.l4_scenario_sha256}",
        "(the scenario line below is the L2-schema world input the runner wrote from it)",
    ]
    return "\n".join(head) + "\n" + rest


def write_report(runs, evaluation=None, path=None):
    text = render_report(runs, evaluation)
    path = path or runs[0].run.report_path
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(text, encoding="utf-8", newline="\n")
    return text


# ---- T4 chirp margins (quad spec 4 L4 T4: "chirp injection measures margins that meet QF-3") --------------------------
#
# Design quantities (chirp_design), from tools/card/rate_lead.py design() (decision 0014: the stage (c) rule the build's
# rate parameters come from) on the card, the budget, the scenario register and the sensor profile the card names; the
# build's f32 kp, ki, kd and T_f of every axis and its gyro_lpf_cutoff_hz must equal the design's (else the build is
# stale and refused). The design loop is rate_lead's exact discrete loop (rate_lead.Model: the firmware's PID with the D
# low-pass) in the stage (c) T4 configuration (decision 0014 "Wiring"): the truth gyro, no latency, and the chain with
# every notch bypassed, its low-pass at the design's cutoff (gyro_chain_design.chain_stages).
#   PM_design           the phase margin of the design's gains (design()["gains"], normalised) at the nominal loop and
#                       the four tau x J corners (rate_lead.Model.margins: the smallest over each loop's crossovers).
#   band [w_lo, w_hi]   [min over the box loops of w_c / a, a max over the box loops of w_c]: w_c every crossover of
#                       those loops, a = design()["inputs"]["a"], the loop-shaping spacing of PM_min.
#   tau_held,a          the largest single-axis torque the mixer delivers at the hover thrust request with the
#                       collective held: the largest t >= 0 with f_min <= c_h M[i,thrust] + s t M[i,a] <= f_max for every
#                       motor i and both signs s, c_h the plan's float32 hover thrust, M, f_min = k idle^2 and
#                       f_max = k speed_max^2 from the build's parameters.
#   max|S|              the largest |S(e^{jwT})| = |1/(1 + L)| of the design loop over the nominal loop and the four
#                       corners, the axis's f32 gains normalised by its J (rate_lead.Model.sensitivity:
#                       gyro_chain_design's log grid of angles up to pi, then a golden-section refinement around the
#                       largest grid point).
#   A_a                 tau_held,a / max|S|: with sp = 0 the plant-input torque is S d, so a steady sinusoid keeps it
#                       inside the unsaturated hover envelope.
# Plan (plan_chirp): execution k0 = ceil(settle_s / T) (k T >= settle_s), t0 = its stamp; the chirp runs D = duration_s
# (or D/2 when halved) from t0 in whole microseconds; the identification window is executions
# k0 .. k0 + ceil((D + tail_s) / T); duration_ticks the smallest multiple of every m above the window's last tick.
#
# Identification (identify), from the log alone (indirect closed-loop identification, Ljung 1999 section 13.5). With
# sp = 0 and the prefilter seeded from a zero first gyro sample, r is exactly 0, so the loop is u = -C y, the plant input
# v = u + d and y = P v:  G_m = Y/D = P/(1 + CP), L = C G_m / (1 - C G_m), C (controller) the firmware's law on the
# chain's output with the gains the run flew: (kp + ki T/(z - 1) + kd alpha (z - 1)/(T (z - beta))) H(e^{j w h}), the
# forward-Euler PI and the D path on the measurement through its low-pass at the period T (alpha = 1 - e^(-T/T_f),
# beta = e^(-T/T_f), rate_lead's rule step 2), H the chain's low-pass at the tick h from the build's gyro_lpf_cutoff_hz
# (gyro_chain_design.lowpass_coeffs; every notch bypassed); with kd = 0 and no chain, L4's PI. The rate loop reads the
# chain's output every D ticks while y is the execution-rate sample, so C H(e^{j w h}) leaves out the images the
# decimation folds back. chirp_design evaluates that omission on the T4 design loop, max |C H(e^{j w h}) P_1 / L - 1|
# over the box loops at the band's ends and their crossovers (P_1 the loop's plant part without the chain), and the run
# report prints it (decimation images, reported). y(n) is the gyro sample the firmware received at execution n,
# d(n) the chirp recomputed in binary64 from the composition's float32 parameters at the logged stamp (chirp_value).
# Y and D are the DTFTs over the window, evaluated at any w (dtft_pair, Horner in binary64). Estimator: the plain DTFT
# ratio over one record, not a cross-spectral (Welch) average. The run is deterministic with no stochastic noise to
# average; the input is zero outside [t0, t0 + D) and the loop starts at rest, so for a linear loop Y(w) = G_m(w) D(w)
# holds exactly up to the response left after the window (tail_s) and rounding, while segment averaging would add
# window leakage. What is not linear (DShot quantisation, rotor-thrust curvature, gyroscopic coupling) is measured by
# the amplitude sensitivity U_A = |PM(A) - PM(A/2)|, and the integration error by E_H = |PM(m = 1) - PM(m = 2)|
# (decision 0003 E). Crossover: |L| = 1 on a CROSSOVER_GRID_POINTS log grid over the swept band, exactly one sign change
# required (a tangential double root would escape the grid), then bisection in w on the DTFT itself; PM = pi + arg L,
# arg the principal value (a loop with phase below -pi at crossover is unstable and has no margin to measure).

CHIRP_AXIS_CODE = {"roll": 1, "pitch": 2, "yaw": 3}  # l4_script.hpp ChirpAxis: None, Roll, Pitch, Yaw
MIXER_COLUMNS = ("thrust", "roll", "pitch", "yaw")
# Search-method constants (resolution and termination only, as tools/card/rate.py's): the max|S| grid, the golden-section
# and crossover-bisection iteration caps, and the crossover grid over the swept band.
SENSITIVITY_GRID_POINTS = 4096
GRID_LOG2_LOW = rate.GRID_LOG2_LOW
REFINE_ITERATIONS = 200
CROSSOVER_GRID_POINTS = 64
GOLDEN = (math.sqrt(5) - 1) / 2  # the golden-section ratio


def design_loop(omega, T, kp, ki, jt, tau):
    """L(e^{j omega T}) of the design model (tools/card/rate.py rule, step 2): C(z) = kp + ki T/(z - 1) and the
    ZOH-sampled P(z) = T/(z - 1) - tau (1 - e)/(z - e), e = exp(-T/tau), for J = 1; kp, ki normalised by the axis's J,
    jt the true inertia over the design inertia."""
    z = cmath.exp(1j * omega * T)
    e, one_minus_e, _, _ = rate.plant_constants(tau, T)
    p = T / (z - 1) - tau * one_minus_e / (z - e)
    return (kp + ki * T / (z - 1)) * p / jt


def max_sensitivity(T, kp, ki, loops):
    """(max |1/(1 + L)|, at omega, loop name) over `loops` [(name, jt, tau)] and omega in (0, pi/T] (section comment)."""
    best = (-math.inf, None, None)
    n = SENSITIVITY_GRID_POINTS
    grid = [math.pi / T * 2.0 ** (GRID_LOG2_LOW * (1 - i / (n - 1))) for i in range(n)]
    for name, jt, tau in loops:
        def s(w):
            return abs(1 / (1 + design_loop(w, T, kp, ki, jt, tau)))
        vals = [s(w) for w in grid]
        i = max(range(n), key=vals.__getitem__)
        lo, hi = grid[max(i - 1, 0)], grid[min(i + 1, n - 1)]
        a, b = hi - GOLDEN * (hi - lo), lo + GOLDEN * (hi - lo)
        sa, sb = s(a), s(b)
        for _ in range(REFINE_ITERATIONS):
            if not lo < a < b < hi:
                break
            if sa >= sb:
                hi, b, sb = b, a, sa
                a = hi - GOLDEN * (hi - lo)
                sa = s(a)
            else:
                lo, a, sa = a, b, sb
                b = lo + GOLDEN * (hi - lo)
                sb = s(b)
        for v, w in ((vals[i], grid[i]), (sa, a), (sb, b)):
            if v > best[0]:
                best = (v, w, name)
    return best


def tau_held(m_rows, column, c_h, f_min, f_max):
    """The largest t >= 0 with f_min <= c_h M[i,thrust] + s t M[i,column] <= f_max for every row and s = +-1; None when
    the held collective alone leaves [f_min, f_max]."""
    best = math.inf
    for row in m_rows:
        base = c_h * row[0]
        if not f_min <= base <= f_max:
            return None
        g = abs(row[column])
        if g > 0:
            best = min(best, (f_max - base) / g, (base - f_min) / g)
    return best


def _mixer_rows(params):
    return [[params[f"mixer_m{i}_{c}"] for c in MIXER_COLUMNS] for i in range(1, scn.MOTORS + 1)]


@functools.lru_cache(maxsize=None)
def _rate_design(card, root):
    card_doc = schema.load_yaml(card)
    budget = schema.load_yaml(Path(root) / "design" / "budget.yaml")
    register = schema.load_yaml(Path(root) / "design" / "scenario_values.yaml")
    return rate.design(card_doc, budget, register, card)


LEAD_GAINS = ("kp", "ki", "kd", "d_filter_tau")  # rate_lead.design()["axes32"] columns, as rate_<q>_<axis>


@functools.lru_cache(maxsize=None)
def _lead_design(card, root):
    """tools/card/rate_lead.py design() on the card, the budget, the scenario register and the sensor profile the card
    names (decision 0014: the rule the build's rate parameters come from)."""
    card_doc = schema.load_yaml(card)
    budget = schema.load_yaml(Path(root) / "design" / "budget.yaml")
    register = schema.load_yaml(Path(root) / "design" / "scenario_values.yaml")
    profile = Path(root) / lint.PROFILE_DIR / f"{card_doc['sensor_profile']}.yaml"
    return rate_lead.design(card_doc, budget, register, schema.load_yaml(profile), card, profile)


def chirp_design(card, params, thrust_n, root=ROOT):
    """The design quantities of the section comment, per axis, as a dict. Raises PlanError on a stale build."""
    res = _lead_design(str(card), str(root))
    ch, inputs = res["chain"], res["inputs"]
    fails = []
    for a, gains32 in zip(l4s.AXES, res["axes32"]):
        for q, v in zip(LEAD_GAINS, gains32):
            if params.get(f"rate_{q}_{a}") != v:
                fails.append(f"the build's rate_{q}_{a} {params.get(f'rate_{q}_{a}')!r} is not the design's {v!r} "
                             f"(tools/card/rate_lead.py on {card}): the build is stale")
    if params.get("gyro_lpf_cutoff_hz") != r32(ch["f_c"]):
        fails.append(f"the build's gyro_lpf_cutoff_hz {params.get('gyro_lpf_cutoff_hz')!r} is not the design's "
                     f"{r32(ch['f_c'])!r} (tools/card/rate_lead.py on {card}): the build is stale")
    if fails:
        raise PlanError(fails)
    T = inputs["T"]
    a = inputs["a"]
    loops = rate.corner_list(inputs["tau"], inputs["b_tau"], inputs["b_J"])
    stages = [c for c in gcd.chain_stages(ch["t_s"], ch["f_c"], ch["q"], ch["omega_th"], [ch["omega_th"]] * gcd.MOTORS,
                                          bypass_all=True) if c != gcd.IDENTITY]
    model = rate_lead.Model(ch["t_s"], ch["divisor"], 0, stages, loops, a, inputs["inertia"])  # truth gyro: latency 0
    margins = model.margins(res["gains"])
    if any(m[2] is None for m in margins):
        raise PlanError([f"the T4 design loop has no valid crossover at {[m[0] for m in margins if m[2] is None]}"])
    crossovers = [w for m in margins for w in m[3]]
    band = (min(crossovers) / a, a * max(crossovers))
    bare = rate_lead.Model(ch["t_s"], ch["divisor"], 0, None, loops, a, inputs["inertia"])
    images = 0.0
    for (_, jt, tau), m in zip(loops, margins):
        loop, plain = model.loop(res["gains"], jt, tau), bare.loop(res["gains"], jt, tau)
        for w in (*band, *m[3]):
            th = w * T
            images = max(images, abs(gcd.chain_response(stages, w * ch["t_s"]) * plain.plant(th) / loop.plant(th) - 1))
    k = params["rotor_thrust_coeff"]
    f_min, f_max = k * params["idle_speed"] ** 2, k * params["rotor_speed_max"] ** 2
    rows = _mixer_rows(params)
    axes = {}
    for axis, j in zip(l4s.AXES, inputs["inertia"]):
        kp, ki, kd, tf = (params[f"rate_{q}_{axis}"] for q in LEAD_GAINS)
        s_loop, s_max, s_at = max(model.sensitivity((kp / j, ki / j, kd / j, tf)), key=lambda r: r[1])
        held = tau_held(rows, MIXER_COLUMNS.index(axis), thrust_n, f_min, f_max)
        if held is None or not held > 0:
            raise PlanError([f"the hover thrust {thrust_n!r} N leaves no held-collective torque on {axis}"])
        axes[axis] = {"J": j, "kp": kp, "ki": ki, "kd": kd, "d_filter_tau": tf, "max_sensitivity": s_max,
                      "max_sensitivity_at_rad_s": s_at, "max_sensitivity_loop": s_loop, "tau_held_nm": held,
                      "amplitude_nm": held / s_max}
    return {"T": T, "a": a, "band": band, "design_pm": {m[0]: m[1] for m in margins},
            "design_crossover": {m[0]: m[2] for m in margins}, "pm_min": inputs["PM_min"],
            "tau": inputs["tau"], "loops": loops, "f_min": f_min, "f_max": f_max, "axes": axes,
            "lowpass": gcd.lowpass_coeffs(params["gyro_lpf_cutoff_hz"], ch["t_s"]), "tick_s": ch["t_s"],
            "decimation_images": images}


def chirp_value(t_us, amp, w_lo, w_hi, t0_us, dur_us):
    """The chirp d (N m) at stamp t_us, the composition's chirp_value (fw/compositions/l4_rate_scripted/include/marv/
    l4_script.hpp) evaluated in binary64 from its float32 parameters: 0 outside [t0, t0 + D), else A sin(phi),
    phi = w_lo D / L (exp(L tau / D) - 1), L = ln(w_hi / w_lo)."""
    if t_us < t0_us or t_us - t0_us >= dur_us:
        return 0.0
    sweep = math.log1p((w_hi - w_lo) / w_lo)
    x = sweep * ((t_us - t0_us) / dur_us)
    return amp * math.sin(w_lo * (dur_us / scn.MICROSECONDS_PER_SECOND) / sweep * math.expm1(x))


def dtft_pair(y, d, omega, T):
    """(sum_n y[n] z^n, sum_n d[n] z^n), z = e^{-j omega T}, n from 0: Horner in binary64."""
    z = cmath.exp(-1j * omega * T)
    ay = ad = 0j
    for a, b in zip(reversed(y), reversed(d)):
        ay = ay * z + a
        ad = ad * z + b
    return ay, ad


def controller(omega, T, kp, ki, kd=0.0, d_filter_tau=0.0, lowpass=None, tick_s=None):
    """C(e^{j omega T}) = kp + ki T/(e^{j omega T} - 1), the firmware's PI at the design period T; with kd, plus the D
    path kd alpha (z - 1)/(T (z - beta)) through its low-pass T_f (rate_lead.lowpass_pole: T_f = 0 unfiltered); with
    lowpass = (b0, b1, b2, a1, a2), times the chain's low-pass H(e^{j omega tick_s}) (section comment)."""
    c = kp + ki * T / (cmath.exp(1j * omega * T) - 1)
    if kd:
        c += kd * rate_lead.derivative(cmath.exp(1j * omega * T), T, *rate_lead.lowpass_pole(T, d_filter_tau))
    if lowpass is not None:
        c *= gcd.chain_response([lowpass], omega * tick_s)
    return c


def measured_loop(y, d, omega, T, kp, ki, **law):
    """L = C G_m / (1 - C G_m), G_m = Y/D (section comment); `law` the controller's terms beyond kp, ki."""
    ym, dm = dtft_pair(y, d, omega, T)
    cg = controller(omega, T, kp, ki, **law) * ym / dm
    return cg / (1 - cg)


def identify(y, d, T, kp, ki, band, grid_points=CROSSOVER_GRID_POINTS, **law):
    """{"pm", "crossover", "L", "grid", "reason"}: the measured loop's crossover and phase margin in the band (rad,
    rad/s). pm is None, with the reason, when |L| - 1 does not change sign exactly once on the grid, from above 1.
    `law`: the controller's terms beyond kp, ki (controller's kd, d_filter_tau, lowpass, tick_s)."""
    lo, hi = band
    ws = [lo * (hi / lo) ** (i / (grid_points - 1)) for i in range(grid_points)]
    mags = [abs(measured_loop(y, d, w, T, kp, ki, **law)) for w in ws]
    changes = [i for i in range(grid_points - 1) if (mags[i] > 1) != (mags[i + 1] > 1)]
    out = {"pm": None, "crossover": None, "L": None, "grid": list(zip(ws, mags)), "reason": ""}
    if len(changes) != 1 or not mags[0] > 1:
        out["reason"] = (f"|L| = 1 has {len(changes)} sign changes on the grid (|L| at the band edges {mags[0]!r}, "
                         f"{mags[-1]!r})")
        return out
    a, b = ws[changes[0]], ws[changes[0] + 1]
    for _ in range(REFINE_ITERATIONS):
        mid = (a + b) / 2
        if not a < mid < b:
            break
        if abs(measured_loop(y, d, mid, T, kp, ki, **law)) > 1:
            a = mid
        else:
            b = mid
    w = (a + b) / 2
    loop = measured_loop(y, d, w, T, kp, ki, **law)
    out.update(pm=math.pi + cmath.phase(loop), crossover=w, L=loop)
    return out


def dshot_idle_bound(params, consts=None):
    """ceil(D(omega_idle)) as thrust_to_dshot computes it (fw/mixer/include/marv/mixer/mixer.hpp, decision 0004 item 5):
    d_min + (idle - omega_min) / (omega_max - omega_min) (d_max - d_min) in float32, each operation rounded, then
    ceil."""
    c = consts or prim_constants.load()
    d_min, d_max = r32(float(c["kDshotThrottleMin"])), r32(float(c["kDshotThrottleMax"]))
    span = r32(params["rotor_speed_max"] - params["rotor_speed_min"])
    x = r32(params["idle_speed"] - params["rotor_speed_min"])
    return math.ceil(r32(d_min + r32(r32(x / span) * r32(d_max - d_min)))), c["kDshotThrottleMax"]


@dataclasses.dataclass(frozen=True)
class ChirpPlan:
    axis: str
    axis_index: int
    num_us: int
    den: int
    divisor: int
    tick_s: Fraction
    period_s: Fraction
    settle_s: float
    duration_s: float
    halved: bool
    tail_s: float
    amp_scale: float
    start_execution: int
    t0_us: int
    dur_us: int
    end_execution: int
    duration_ticks: int
    m_sequence: tuple
    thrust_n: float
    thrust_n_double: float
    amp_nm: float
    w_lo: float
    w_hi: float
    kp: float
    ki: float
    kd: float
    d_filter_tau: float
    design: dict

    def tick_of(self, k):
        return self.divisor * k

    def stamp_us(self, k):
        return (self.divisor * k * self.num_us) // self.den

    @property
    def window(self):
        return range(self.start_execution, self.end_execution + 1)

    def overrides(self):
        return {"l4_thrust_n": (F32, repr(self.thrust_n)), "l4_seg_count": (I32, "0"),
                "l4_chirp_axis": (I32, str(CHIRP_AXIS_CODE[self.axis])), "l4_chirp_amp_nm": (F32, repr(self.amp_nm)),
                "l4_chirp_w_lo": (F32, repr(self.w_lo)), "l4_chirp_w_hi": (F32, repr(self.w_hi)),
                "l4_chirp_t0_us": (I32, str(self.t0_us)), "l4_chirp_dur_us": (I32, str(self.dur_us))}

    def d(self, t_us):
        return chirp_value(t_us, self.amp_nm, self.w_lo, self.w_hi, self.t0_us, self.dur_us)

    def report(self):
        ax = self.design["axes"][self.axis]
        return {
            "axis": self.axis, "tick period": f"{self.num_us}/{self.den} us (build and scenario)",
            "rate_loop_divisor D": self.divisor, "rate-loop period T": f"{self.period_s} s",
            "settle_s": self.settle_s, "chirp duration (s)": self.duration_s, "halved (D/2 run)": self.halved,
            "tail_s": self.tail_s, "chirp start execution k0": self.start_execution, "l4_chirp_t0_us": self.t0_us,
            "l4_chirp_dur_us": self.dur_us, "window executions": f"{self.window.start}..{self.window.stop - 1}",
            "duration_ticks": self.duration_ticks, "m_sequence": list(self.m_sequence),
            "l4_thrust_n (float32)": self.thrust_n, "m g(phi, h0) (double)": self.thrust_n_double,
            "band (design, rad/s)": list(self.design["band"]), "band (float32, swept)": [self.w_lo, self.w_hi],
            "tau_held (N m)": ax["tau_held_nm"], "max|S| (design box)": ax["max_sensitivity"],
            "max|S| at (rad/s), loop": f"{ax['max_sensitivity_at_rad_s']!r}, {ax['max_sensitivity_loop']}",
            "A = tau_held / max|S| (N m)": ax["amplitude_nm"], "amplitude scale": self.amp_scale,
            "l4_chirp_amp_nm (float32)": self.amp_nm, "C gains kp, ki (build float32)": [self.kp, self.ki],
            "C D path kd, T_f (build float32)": [self.kd, self.d_filter_tau],
            "C chain low-pass b0 b1 b2 a1 a2 (build cutoff, tick)": list(self.design["lowpass"]),
            "decimation images (T4 design loop, reported)": self.design["decimation_images"],
        }


def _plan_common(vals, params, need):
    fails = []
    missing = [k for k in need if k not in params]
    if missing:
        raise PlanError([f"the build's parameter table lacks {missing}"])
    num, den, divisor = params["tick_period_num_us"], params["tick_period_den"], params["rate_loop_divisor"]
    if (vals["tick_period_num_us"], vals["tick_period_den"]) != (num, den):
        fails.append(f"tick period {vals['tick_period_num_us']}/{vals['tick_period_den']} us differs from the build's "
                     f"{num}/{den} us: the rate loop would panic on the execution spacing")
    if divisor < 1:
        fails.append(f"the build's rate_loop_divisor {divisor} is below 1")
    for m in vals["m_sequence"]:
        if divisor >= 1 and divisor % m:
            fails.append(f"m = {m} does not divide rate_loop_divisor = {divisor}: an execution would read a held sample")
    if fails:
        raise PlanError(fails)
    tick = Fraction(num, den * scn.MICROSECONDS_PER_SECOND)
    return num, den, divisor, tick, divisor * tick


def _duration_ticks(divisor, last_execution, m_sequence):
    lcm = math.lcm(*m_sequence)
    return (divisor * last_execution // lcm + 1) * lcm


CHIRP_NEED = ("tick_period_num_us", "tick_period_den", "rate_loop_divisor", "rotor_thrust_coeff", "idle_speed",
              "rotor_speed_min", "rotor_speed_max", *(f"rate_{q}_{a}" for q in ("kp", "ki") for a in l4s.AXES),
              *(f"mixer_m{i}_{c}" for i in range(1, scn.MOTORS + 1) for c in MIXER_COLUMNS))


def plan_chirp(doc, params, card, root=ROOT, *, amp_scale=1.0, halved=False):
    """The derived plan of a valid chirp scenario against the build's parameters (section comment). amp_scale in (0, 1]
    scales the float32 amplitude (1/2 for U_A); halved runs D/2 (the convergence check). Raises PlanError."""
    vals = l4s.chirp_values(doc)
    c = vals["chirp"]
    axis = c["axis"]
    num, den, divisor, tick, period = _plan_common(vals, params, CHIRP_NEED)
    if not 0 < amp_scale <= 1:
        raise PlanError([f"amp_scale {amp_scale!r} is not in (0, 1]"])
    thrust = hover_thrust(card, vals, root)
    design = chirp_design(card, params, r32(thrust), root)
    duration = c["duration_s"] / 2 if halved else c["duration_s"]
    dur_us = Fraction(duration) * scn.MICROSECONDS_PER_SECOND
    if dur_us.denominator != 1 or not 0 < dur_us < l4s.I32_LIMIT_US:
        raise PlanError([f"chirp duration {duration!r} s is not a whole number of microseconds in (0, 2^31)"])
    k0 = max(1, math.ceil(Fraction(c["settle_s"]) / period))
    t0 = (divisor * k0 * num) // den
    end = k0 + math.ceil((Fraction(duration) + Fraction(c["tail_s"])) / period)
    if t0 >= l4s.I32_LIMIT_US:
        raise PlanError([f"the chirp start stamp {t0} us does not fit the composition's i32 l4_chirp_t0_us"])
    lo, hi = design["band"]
    ax = design["axes"][axis]
    return ChirpPlan(axis=axis, axis_index=l4s.AXES.index(axis), num_us=num, den=den, divisor=divisor, tick_s=tick,
                     period_s=period, settle_s=c["settle_s"], duration_s=duration, halved=halved, tail_s=c["tail_s"],
                     amp_scale=amp_scale, start_execution=k0, t0_us=t0, dur_us=int(dur_us), end_execution=end,
                     duration_ticks=_duration_ticks(divisor, end, vals["m_sequence"]),
                     m_sequence=tuple(vals["m_sequence"]), thrust_n=r32(thrust), thrust_n_double=thrust,
                     amp_nm=r32(r32(ax["amplitude_nm"]) * amp_scale), w_lo=r32(lo), w_hi=r32(hi), kp=ax["kp"],
                     ki=ax["ki"], kd=ax["kd"], d_filter_tau=ax["d_filter_tau"], design=design)


def _log_counts(log):
    """The parsed log reduced to what the run report and complete_trailer read (header, trailer and the three record
    counts as sized ranges), so that a long run's records are not kept."""
    return {"header": log["header"], "trailer": log["trailer"], "steps": range(len(log["steps"])),
            "ticks": range(len(log["ticks"])), "applied": range(len(log["applied"]))}


@dataclasses.dataclass
class ChirpRun:
    run: run_scenario.RunResult
    plan: ChirpPlan
    l4_scenario: str
    l4_scenario_sha256: str
    params_path: str
    overrides: dict
    harness_overrides: dict
    stamps: list  # window executions' stamps, us
    y: list  # window executions' gyro sample on the chirp axis, rad/s
    d: list  # the chirp recomputed at those stamps, N m
    gyro_first: tuple  # execution 0's gyro sample (the prefilter's seed)
    flags: set  # the distinct IMU flag words of the window's samples
    other_axes_peak: float  # max |gyro| of the other two axes in the window, rad/s
    dshot_end_executions: int  # window executions with some motor at the idle bound or kDshotThrottleMax
    window_steps: range
    window_stale: list


def _l2_doc_of(doc, p, stem, source):
    """l2_scenario_doc for the chirp and acro kinds (their shared fields are the step kind's)."""
    step_like = {**doc, "step": {k: {"value": None} for k in l4s.STEP_FIELDS}}
    return l2_scenario_doc(step_like, p, stem, source)


def _run_l4_world(card, doc, p, stem, scenario, m, out_dir, plugin_dir, applied, extra_edit, seed, root, timeout_s):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    l2_path = out_dir / f"{stem}.yaml"
    l2_path.write_text(yaml.safe_dump(_l2_doc_of(doc, p, stem, Path(scenario).name), sort_keys=False),
                       encoding="utf-8")
    return run_scenario._run(card, l2_path, seed, m, "test", None, out_dir, plugin_dir, world_edit(applied, extra_edit),
                             None, root, Path(root) / "design" / "budget.yaml", timeout_s)


def run_chirp(card, scenario, m, out_dir, plugin_dir=DEFAULT_PLUGIN_DIR, *, amp_scale=1.0, halved=False, overrides=None,
              extra_edit=None, seed=None, root=ROOT, timeout_s=run_scenario.TIMEOUT_S):
    """One gz process of a chirp scenario at m ticks per host step; writes the world, the log and the run report.
    `overrides` are harness sil_overrides (the negative control's gains), name -> (type, text)."""
    doc = l4s.load_chirp(scenario)
    _, defaults = build_parameters(plugin_dir)
    params = read_param_defaults(defaults)
    p = plan_chirp(doc, params, card, root, amp_scale=amp_scale, halved=halved)
    if m not in p.m_sequence:
        raise PlanError([f"m = {m} is not in the scenario's m_sequence {list(p.m_sequence)}"])
    harness = dict(overrides or {})
    applied = p.overrides()
    twice = sorted(set(applied) & set(harness))
    if twice:
        raise PlanError([f"harness overrides {twice} are plan parameters already"])
    applied.update(harness)
    tag = ("_halfamp" if amp_scale != 1 else "") + ("_halfdur" if halved else "") + ("_harness" if harness else "")
    stem = f"{doc['scenario']}{tag}_{NAME_LABEL}"
    r = _run_l4_world(card, doc, p, stem, scenario, m, out_dir, plugin_dir, applied, extra_edit, seed, root, timeout_s)
    log = r.log
    fresh = set(run_scenario.fresh_steps(log))
    lo, hi = dshot_idle_bound(params)
    stamps, y, flags, other, ends = [], [], set(), 0.0, 0
    for k in p.window:
        t = log["ticks"][p.tick_of(k)]
        v = IMU.unpack(t["imu"])
        stamps.append(t["sil_t_us"])
        y.append(v[p.axis_index])
        flags.add(v[7])
        other = max(other, *(abs(v[i]) for i in range(3) if i != p.axis_index))
        ends += any(x in (lo, hi) for x in t["dshot"])
    first = IMU.unpack(log["ticks"][0]["imu"])[:3]
    steps = range(p.tick_of(p.window.start) // m, p.tick_of(p.window.stop - 1) // m + 1)
    r.log = _log_counts(log)
    del log
    s = ChirpRun(run=r, plan=p, l4_scenario=str(scenario), l4_scenario_sha256=run_scenario.sha256_file(scenario),
                 params_path=str(defaults), overrides=applied, harness_overrides=harness, stamps=stamps, y=y,
                 d=[p.d(t) for t in stamps], gyro_first=first, flags=flags, other_axes_peak=other,
                 dshot_end_executions=ends, window_steps=steps, window_stale=[i for i in steps if i not in fresh])
    r.report_path = str(Path(r.log_path).with_suffix(".report.txt"))
    write_t4_report([s], "t4 chirp", None, r.report_path)
    return s


def flown_gains(s):
    """(kp, ki) of the chirp axis that the run flew: a harness sil_override of rate_kp_<axis> / rate_ki_<axis> (the
    negative control) if there is one, else the build's."""
    p = s.plan
    return tuple(float(s.overrides[f"rate_{q}_{p.axis}"][1]) if f"rate_{q}_{p.axis}" in s.overrides else getattr(p, q)
                 for q in ("kp", "ki"))


def flown_law(s):
    """The controller's terms beyond kp, ki that the run flew (controller's keywords): kd and T_f of the chirp axis, a
    harness sil_override if there is one, else the build's, and the chain's low-pass at the build's cutoff and tick."""
    p = s.plan
    kd, tf = (float(s.overrides[f"rate_{q}_{p.axis}"][1]) if f"rate_{q}_{p.axis}" in s.overrides else getattr(p, q)
              for q in ("kd", "d_filter_tau"))
    return {"kd": kd, "d_filter_tau": tf, "lowpass": p.design["lowpass"], "tick_s": p.design["tick_s"]}


def chirp_margin(s):
    """identify() on one chirp run: C with the law the run flew, the swept float32 band."""
    p = s.plan
    kp, ki = flown_gains(s)
    return identify(s.y, s.d, float(p.period_s), kp, ki, (p.w_lo, p.w_hi), **flown_law(s))


# ---- T4 acro (quad spec 4 L4 T4: "acro with a scripted stick sequence completes without saturation beyond the mixer's
# documented behaviour") ------------------------------------------------------------------------------------------------
#
# Plan (plan_acro): segment k applies from execution k_k = ceil(start_s / T), l4_seg<k>_t_us = its stamp; the setpoint
# l4_seg<k>_<axis> = float32(stick_a rate_max_a), rate_max_a the build's float32; the segments must fit the build's
# capacity (its number of l4_seg<k>_t_us parameters) and their stamps be strictly increasing; the run ends at the first
# execution at or after end_s, duration_ticks the smallest multiple of every m above its tick. Harness overrides may
# replace plan parameters (the negative control); the report lists them.

SEGMENT_T_US = re.compile(r"^l4_seg(\d+)_t_us$")


@dataclasses.dataclass(frozen=True)
class AcroPlan:
    num_us: int
    den: int
    divisor: int
    tick_s: Fraction
    period_s: Fraction
    segments: tuple  # ((execution, t_us, (roll, pitch, yaw) float32 rad/s, stick), ...)
    end_s: float
    end_execution: int
    duration_ticks: int
    m_sequence: tuple
    thrust_n: float
    thrust_n_double: float
    rate_max: tuple
    capacity: int

    def tick_of(self, k):
        return self.divisor * k

    def stamp_us(self, k):
        return (self.divisor * k * self.num_us) // self.den

    @property
    def window(self):
        return range(self.segments[0][0], self.end_execution + 1)

    def overrides(self):
        out = {"l4_thrust_n": (F32, repr(self.thrust_n)), "l4_seg_count": (I32, str(len(self.segments)))}
        for i, (_, t_us, sp, _) in enumerate(self.segments, start=1):
            out[f"l4_seg{i}_t_us"] = (I32, str(t_us))
            for a, v in zip(l4s.AXES, sp):
                out[f"l4_seg{i}_{a}"] = (F32, repr(v))
        return out

    def report(self):
        return {
            "tick period": f"{self.num_us}/{self.den} us (build and scenario)", "rate_loop_divisor D": self.divisor,
            "rate-loop period T": f"{self.period_s} s", "segment capacity (build)": self.capacity,
            "segments (execution, t_us, setpoint rad/s, stick)": [list(s) for s in self.segments],
            "end_s": self.end_s, "end execution": self.end_execution, "duration_ticks": self.duration_ticks,
            "m_sequence": list(self.m_sequence), "rate_max (build float32)": list(self.rate_max),
            "l4_thrust_n (float32)": self.thrust_n, "m g(phi, h0) (double)": self.thrust_n_double,
        }


def plan_acro(doc, params, card, root=ROOT):
    """The derived plan of a valid acro scenario against the build's parameters (section comment). Raises PlanError."""
    vals = l4s.acro_values(doc)
    need = ("tick_period_num_us", "tick_period_den", "rate_loop_divisor", *(f"rate_max_{a}" for a in l4s.AXES))
    num, den, divisor, tick, period = _plan_common(vals, params, need)
    capacity = sum(1 for k in params if SEGMENT_T_US.match(k))
    segs = vals["acro"]["segments"]
    fails = []
    if len(segs) > capacity:
        fails.append(f"{len(segs)} segments exceed the build's capacity of {capacity}")
    rate_max = tuple(params[f"rate_max_{a}"] for a in l4s.AXES)
    out, prev = [], -1
    for i, seg in enumerate(segs, start=1):
        k = math.ceil(Fraction(seg["start_s"]) / period)
        t_us = (divisor * k * num) // den
        if not t_us > prev:
            fails.append(f"segment {i} starts at stamp {t_us} us, not after the previous segment's {prev} us")
        if t_us >= l4s.I32_LIMIT_US:
            fails.append(f"segment {i}'s stamp {t_us} us does not fit the composition's i32 l4_seg<k>_t_us")
        prev = t_us
        out.append((k, t_us, tuple(r32(s * r) for s, r in zip(seg["stick"], rate_max)), tuple(seg["stick"])))
    if fails:
        raise PlanError(fails)
    end = math.ceil(Fraction(vals["acro"]["end_s"]) / period)
    thrust = hover_thrust(card, vals, root)
    return AcroPlan(num_us=num, den=den, divisor=divisor, tick_s=tick, period_s=period, segments=tuple(out),
                    end_s=vals["acro"]["end_s"], end_execution=end,
                    duration_ticks=_duration_ticks(divisor, end, vals["m_sequence"]),
                    m_sequence=tuple(vals["m_sequence"]), thrust_n=r32(thrust), thrust_n_double=thrust,
                    rate_max=rate_max, capacity=capacity)


@dataclasses.dataclass
class AcroRun:
    run: run_scenario.RunResult
    plan: AcroPlan
    l4_scenario: str
    l4_scenario_sha256: str
    params_path: str
    overrides: dict
    harness_overrides: dict
    executions: list  # per execution k: {"k", "tick", "t_us", "gyro" (3 floats), "flags", "fresh"}
    dshot: list  # per tick: the four DShot commands
    dshot_bounds: tuple  # (ceil(D(omega_idle)), kDshotThrottleMax) from the build's parameters
    window_steps: range
    window_stale: list


def run_acro(card, scenario, m, out_dir, plugin_dir=DEFAULT_PLUGIN_DIR, *, overrides=None, extra_edit=None, seed=None,
             root=ROOT, timeout_s=run_scenario.TIMEOUT_S):
    """One gz process of the acro scenario at m ticks per host step; writes the world, the log and the run report.
    `overrides` are harness sil_overrides, name -> (type, text); they may replace plan parameters."""
    doc = l4s.load_acro(scenario)
    _, defaults = build_parameters(plugin_dir)
    params = read_param_defaults(defaults)
    p = plan_acro(doc, params, card, root)
    if m not in p.m_sequence:
        raise PlanError([f"m = {m} is not in the scenario's m_sequence {list(p.m_sequence)}"])
    harness = dict(overrides or {})
    applied = {**p.overrides(), **harness}
    stem = f"{doc['scenario']}{'_harness' if harness else ''}_{NAME_LABEL}"
    r = _run_l4_world(card, doc, p, stem, scenario, m, out_dir, plugin_dir, applied, extra_edit, seed, root, timeout_s)
    ex = executions(r.log, p, m)
    fresh = set(run_scenario.fresh_steps(r.log))
    steps = range(p.tick_of(p.window.start) // m, p.tick_of(p.window.stop - 1) // m + 1)
    s = AcroRun(run=r, plan=p, l4_scenario=str(scenario), l4_scenario_sha256=run_scenario.sha256_file(scenario),
                params_path=str(defaults), overrides=applied, harness_overrides=harness, executions=ex,
                dshot=[t["dshot"] for t in r.log["ticks"]], dshot_bounds=dshot_idle_bound(params), window_steps=steps,
                window_stale=[i for i in steps if i not in fresh])
    r.log = _log_counts(r.log)
    r.report_path = str(Path(r.log_path).with_suffix(".report.txt"))
    write_t4_report([s], "t4 acro", None, r.report_path)
    return s


def render_t4_report(runs, key, evaluation=None):
    """render_report's text for chirp and acro runs: the plan, the overrides and the window's freshness, and
    `evaluation` under `key`."""
    s0 = runs[0]
    section = {
        "l4 plan (tools/sim/run_l4.py)": s0.plan.report(),
        "l4 parameter table (build)": s0.params_path,
        "l4 runs": {f"run {i}: m{s.run.m}": {
            "log": s.run.log_path,
            "sil overrides": {k: f"{t} {v}" for k, (t, v) in s.overrides.items()},
            "harness overrides": {k: f"{t} {v}" for k, (t, v) in s.harness_overrides.items()} or "(none)",
            "window host steps": f"{s.window_steps.start}..{s.window_steps.stop - 1}",
            "stale reads in the window": len(s.window_stale),
        } for i, s in enumerate(runs)},
        key: evaluation if evaluation else "(not evaluated)",
    }
    text = run_scenario.render_report([s.run for s in runs], section)
    first, rest = text.split("\n", 1)
    if first != "MARV L2 run report":
        raise run_scenario.RunError(f"unexpected run report header {first!r}")
    head = [f"MARV L4 run report: {LABEL}", "", LABEL_LINE, f"l4 scenario: {s0.l4_scenario}",
            f"l4 scenario sha256: {s0.l4_scenario_sha256}",
            "(the scenario line below is the L2-schema world input the runner wrote from it)"]
    return "\n".join(head) + "\n" + rest


def write_t4_report(runs, key, evaluation=None, path=None):
    text = render_t4_report(runs, key, evaluation)
    path = path or runs[0].run.report_path
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(text, encoding="utf-8", newline="\n")
    return text


# ---- the chirp's float32 input term U_d ------------------------------------------------------------------------------
#
# The identification recomputes d in binary64 (chirp_value); the composition computes it in float32. With
# delta_n = d32(n) - d64(n), d32 the composition's chirp_value emulated in float32 (chirp_value_f32: every operation of
# l4_script.hpp rounded to nearest float32, the libm results taken correctly rounded), the firmware's input is
# D64 + Delta, so G_m is estimated with the relative error Delta(w)/D64(w), and |Delta(w)| <= sum_n |delta_n| at every w
# (triangle inequality). Rule (first order in the error, float_input_term):
#   rho = sum_n |delta_n| / |D64(w_c)|, e = rho |1 + L(w_c)|   (L = C G_m / (1 - C G_m) and 1 - C G_m = 1/(1 + L), so a
#                                                                relative error rho of G_m is one of rho |1 + L| in L)
#   U_d = asin(min(e, 1)) + e kappa,  kappa = |d arg L / dw| / |d ln|L| / dw| at w_c (central differences of the measured
#   loop, step w_c 2^DIFF_LOG2_STEP): the phase error of L plus the phase change over the crossover shift that a
#   relative magnitude error e causes.
DIFF_LOG2_STEP = -10


def chirp_value_f32(t_us, amp, w_lo, w_hi, t0_us, dur_us):
    """The composition's chirp_value (l4_script.hpp) with each float32 operation emulated by rounding to nearest."""
    if t_us < t0_us or t_us - t0_us >= dur_us:
        return 0.0
    sweep = r32(math.log1p(r32(r32(w_hi - w_lo) / w_lo)))
    x = r32(sweep * r32(r32(float(t_us - t0_us)) / r32(float(dur_us))))
    dur_s = r32(r32(float(dur_us)) / r32(float(scn.MICROSECONDS_PER_SECOND)))
    phi = r32(r32(r32(w_lo * dur_s) / sweep) * r32(math.expm1(x)))
    return r32(amp * r32(math.sin(phi)))


def float_input_term(s, margin):
    """U_d of a chirp run (section comment) at the crossover of its identify() result `margin`, with its terms."""
    p = s.plan
    T = float(p.period_s)
    kp, ki = flown_gains(s)
    law = flown_law(s)
    w = margin["crossover"]
    delta = [chirp_value_f32(t, p.amp_nm, p.w_lo, p.w_hi, p.t0_us, p.dur_us) - d for t, d in zip(s.stamps, s.d)]
    _, dm = dtft_pair(s.y, s.d, w, T)
    rho = sum(abs(x) for x in delta) / abs(dm)
    e = rho * abs(1 + margin["L"])
    h = w * 2.0 ** DIFF_LOG2_STEP
    up, down = measured_loop(s.y, s.d, w + h, T, kp, ki, **law), measured_loop(s.y, s.d, w - h, T, kp, ki, **law)
    kappa = abs(cmath.phase(up / down)) / abs(math.log(abs(up) / abs(down)))
    return {"U_d": math.asin(min(e, 1.0)) + e * kappa, "max |delta| / A": max(abs(x) for x in delta) / p.amp_nm,
            "rho": rho, "e": e, "kappa": kappa}


# ---- the acro rate bound: the linear design model on the script ------------------------------------------------------
#
# script_response is the T3 oracle's closed_loop (tests/regression/quad/L04/t3/reference/rate_t3_oracle.py) with a
# setpoint per execution instead of one step, operation for operation: the stage (c) rate path of decision 0014 in
# binary64, the gyro chain's low-pass on the plant omega at every tick (direct form I, seeded with the first sample;
# every notch bypassed, the T4 (c) configuration), then on the execution ticks the firmware's law on the chain output
# (prefilter seeded at the first execution, forward-Euler integral deferred one execution, D on the measurement through
# its low-pass T_f, no anti-windup, no mixer), the plant advanced tick by tick by the exact per-tick map
# (a12, b1, a22, b2) of J w' = u_m, tau u_m' = u - u_m, u held over the D ticks of an execution; with want_rho, the
# oracle's first-order float32 rounding injections at its seven nodes. chain = (D, the low-pass's (b0, b1, b2, a1, a2),
# their f32 error bounds): the oracle's Setup.divisor, Setup.lowpass and Setup.lowpass_error. script_envelope is
# band_envelope's grid over the tau x J box on it.
RHO_NODES = ("x", "f", "r", "e", "I", "D", "u")
UNIT_ROUNDOFF_F32 = 2.0 ** -24


def _ulp32(x):
    return 2.0 ** (math.frexp(abs(x))[1] - 24)


def script_response(kp, ki, kd, d_filter_tau, tau_ref, chain, tick_map, setpoints, stamps_us, want_rho=False):
    """w at each execution of the linear design model driven by `setpoints` at `stamps_us` (section comment)."""
    divisor, (c0, c1, c2, c3, c4), (dc0, dc1, dc2, dc3, dc4) = chain
    a12, b1, a22, b2 = tick_map
    u32 = UNIT_ROUNDOFF_F32
    w = um = 0.0
    x1 = x2 = f1 = f2 = 0.0
    r = integral = e_prev = y_prev = d_f = u = 0.0
    y_out = []
    rho = dict.fromkeys(RHO_NODES, 0.0)
    for n, sp in enumerate(setpoints):
        y_out.append(w)
        for tick in range(divisor):
            x = w
            if n == 0 and tick == 0:
                x1 = x2 = f1 = f2 = x
            p0, p1, p2, p3, p4 = c0 * x, c1 * x1, c2 * x2, c3 * f1, c4 * f2
            s1 = p0 + p1
            s2 = s1 + p2
            s3 = s2 - p3
            f = s3 - p4
            if want_rho:
                rho["x"] = max(rho["x"], u32 * abs(w))
                rho["f"] = max(rho["f"], u32 * (abs(p0) + abs(p1) + abs(p2) + abs(p3) + abs(p4)
                                                + abs(s1) + abs(s2) + abs(s3) + abs(f))
                               + dc0 * abs(x) + dc1 * abs(x1) + dc2 * abs(x2) + dc3 * abs(f1) + dc4 * abs(f2))
            x2, x1, f2, f1 = x1, x, f1, f
            if tick == 0:
                y = f
                if n == 0:
                    r, e_prev, integral, d_f, u = y, 0.0, 0.0, 0.0, 0.0
                else:
                    dt = (stamps_us[n] - stamps_us[n - 1]) * 1e-6  # us to s, as the oracle's MICROSECOND
                    xr = dt / tau_ref
                    alpha = -math.expm1(-xr)
                    r_prev = r
                    r = r + alpha * (sp - r)
                    e = r - y
                    i_prev_term = ki * e_prev * dt
                    integral = integral + i_prev_term
                    d = -kd * (y - y_prev) / dt
                    d_prev = d_f
                    if d_filter_tau > 0.0:
                        xd = dt / d_filter_tau
                        alpha_d = -math.expm1(-xd)
                        d_f = d_f + alpha_d * (d - d_f)
                    else:
                        d_f = d
                    pi_sum = kp * e + integral
                    u = pi_sum + d_f
                    if want_rho:
                        d_alpha = math.exp(-xr) * xr * 2 * u32 + _ulp32(math.exp(-xr)) + u32 * alpha
                        s = abs(sp - r_prev)
                        rho["r"] = max(rho["r"], d_alpha * s + 2 * u32 * alpha * s + u32 * abs(r))
                        rho["e"] = max(rho["e"], u32 * abs(e))
                        rho["I"] = max(rho["I"], 3 * u32 * abs(i_prev_term) + u32 * abs(integral))
                        rho_d = 4 * u32 * abs(d)
                        if d_filter_tau > 0.0:
                            d_alpha_d = math.exp(-xd) * xd * 2 * u32 + _ulp32(math.exp(-xd)) + u32 * alpha_d
                            s_d = abs(d - d_prev)
                            rho_d = alpha_d * rho_d + d_alpha_d * s_d + 2 * u32 * alpha_d * s_d + u32 * abs(d_f)
                        rho["D"] = max(rho["D"], rho_d)
                        rho["u"] = max(rho["u"], u32 * (abs(kp * e) + abs(pi_sum) + abs(u)))
                    e_prev = e
                y_prev = y
            w, um = w + a12 * um + b1 * u, a22 * um + b2 * u
    return (y_out, rho) if want_rho else y_out


def script_envelope(kp, ki, kd, d_filter_tau, tau_ref, chain, tick_map_of, inertia, tau, j_band, tau_band, setpoints,
                    stamps_us, points):
    """(lo, hi) per execution over the points x points grid of J (1 + s j_band), tau (1 + t tau_band), s, t in [-1, 1]
    (the oracle's band_envelope grid, corners included); tick_map_of(J, tau) gives the per-tick plant map."""
    lo = [math.inf] * len(setpoints)
    hi = [-math.inf] * len(setpoints)
    for i in range(points):
        for j in range(points):
            s = -1.0 + 2.0 * i / (points - 1)
            t = -1.0 + 2.0 * j / (points - 1)
            y = script_response(kp, ki, kd, d_filter_tau, tau_ref, chain,
                                tick_map_of(inertia * (1 + s * j_band), tau * (1 + t * tau_band)), setpoints, stamps_us)
            lo = [min(a, b) for a, b in zip(lo, y)]
            hi = [max(a, b) for a, b in zip(hi, y)]
    return lo, hi


def acro_setpoints(p, axis_index):
    """(setpoint of the axis, stamp) at executions 0 .. end_execution of an acro plan: the composition's setpoint_at."""
    sps, stamps = [], []
    for k in range(p.end_execution + 1):
        t = p.stamp_us(k)
        sp = 0.0
        for _, t_us, rates, _ in p.segments:
            if t >= t_us:
                sp = rates[axis_index]
        sps.append(sp)
        stamps.append(t)
    return sps, stamps


# ---- the acro replay and the acro checks by segment (Option 1, Luis 2026-09-30) ------------------------------------
#
# Replay (replay_acro). tests/regression/quad/L04/replay/l4_acro_replay, built in the plugin's build tree (replay_tool),
# re-executes the composition's due-tick sequence on the IMU samples and stamps of the run's log with the run's
# sil_overrides (replay_input) and writes per execution the setpoint, the torque request, the allocation (achieved
# torque and thrust, saturation flags), s and t derived as achieved / requested, and DShot. Its DShot must equal the
# log's at every execution (replay_fidelity): a replay that does not reproduce the run shows nothing about it.
#
# Segment membership (acro_segment_kinds, acro_phases), from the scenario's sticks. A segment is 'zero' (no nonzero
# stick), 'single' (one), 'reversal' (one, of the opposite sign to the previous segment's stick on that axis) or
# 'combined' (two or more). Every execution before the first combined segment is 'linear': the script so far commands
# one axis at a time, the case the per-axis linear design model of the rate bound (iii) describes. A combined segment's
# executions are 'combined'; the segment after it must be a zero segment, whose executions are 'recovery'. A script with
# a combined segment that no zero segment follows, or with a segment after a recovery segment, is refused: the rule
# covers neither.
#
# Recovery (rest_bound, recovery_evaluation; Luis, 2026-09-30), per axis: |w(n)| <= Z(n) + F + E(n) at every fresh
# execution n of the recovery window, Z(n) = max(|lo(n)|, |hi(n)|) the largest |w| over the band box grid of the linear
# design model driven by the exact script from rest (script_envelope, as the rate bound (iii)). A vehicle at rest passes
# by construction; the model is not re-seeded from the measured state (that would define an uncommanded rate as
# correct).
#
# Flag consistency (flag_findings; decision 0004 item 5, decision 0005 anti-windup bound): at every replayed execution
# given and per axis, a set flag has |ach| < |req|, and |req| - |ach| > b_a has the flag set, where
# b_a = (gamma_4 + eps) (|B||M||v|)_a, v = [ach_thrust, ach_roll, ach_pitch, ach_yaw], eps the binary32 epsilon,
# gamma_4 = 4 eps / (1 - 4 eps), B the rate loop's effectiveness and M the mixer from the run's parameters, evaluated in
# binary64 on the binary32 values.
REPLAY_TOOL = ("tests", "regression", "quad", "L04", "replay", "l4_acro_replay")
IMU_WORDS = struct.Struct("<8I")  # marv_imu_meas as words: 7 binary32 bit patterns, then flags
REPLAY_INT_COLUMNS = frozenset({"k", "tick", "t_us", "flag_roll", "flag_pitch", "flag_yaw", "dshot1", "dshot2",
                                "dshot3", "dshot4", "fault"})
EPS_F32 = 2.0 ** -23


def replay_tool(plugin_dir=DEFAULT_PLUGIN_DIR):
    """The l4_acro_replay executable of the build that holds plugin_dir (the build whose parameter table the plugin
    runs; build_parameters)."""
    _, defaults = build_parameters(plugin_dir)
    return Path(defaults).parents[4].joinpath(*REPLAY_TOOL)


def replay_input(log, overrides):
    """The replay tool's input text: every sil_override of the run, then every logged tick's stamp and IMU sample."""
    lines = [f"override {k} {t} {v}" for k, (t, v) in overrides.items()]
    for t in log["ticks"]:
        w = IMU_WORDS.unpack(t["imu"])
        lines.append(f"tick {t['tick']} {t['sil_t_us']} {' '.join(f'{x:08x}' for x in w[:7])} {w[7]}")
    return "\n".join(lines) + "\n"


def read_replay(text):
    """The replay tool's output as one dict per execution (ints for REPLAY_INT_COLUMNS, floats otherwise)."""
    lines = text.splitlines()
    names = lines[0].split()
    rows = []
    for line in lines[1:]:
        vals = line.split()
        if len(vals) != len(names):
            raise run_scenario.RunError(f"replay row has {len(vals)} fields, expected {len(names)}: {line!r}")
        rows.append({n: int(v) if n in REPLAY_INT_COLUMNS else float(v) for n, v in zip(names, vals)})
    return rows


def replay_acro(s, tool, work_dir, overrides=None, tag=""):
    """Replay an AcroRun (its log, its sil_overrides) with `tool`; the input and output files go to work_dir.
    `overrides` (name -> (type, text)) replace or add sil_overrides for the replay only (the fidelity control); `tag`
    names its files apart."""
    log = lockstep_log.read(s.run.log_path)
    stem = Path(s.run.log_path).stem + tag
    inp, out = Path(work_dir) / f"{stem}.replay_in.txt", Path(work_dir) / f"{stem}.replay.txt"
    inp.write_text(replay_input(log, {**s.overrides, **(overrides or {})}), encoding="utf-8", newline="\n")
    del log
    proc = subprocess.run([str(tool), str(inp), str(out)], capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise run_scenario.RunError(f"{tool} exited {proc.returncode}: {proc.stderr.strip()}")
    return read_replay(out.read_text(encoding="utf-8"))


def replay_fidelity(s, rows):
    """None if the replay reproduces the run: one row per logged execution, each with the log's tick and stamp and the
    log's DShot bit for bit; else the first mismatch."""
    if len(rows) != len(s.executions):
        return {"reason": f"{len(rows)} replayed executions, the log has {len(s.executions)}"}
    for row, x in zip(rows, s.executions):
        logged = tuple(s.dshot[x["tick"]])
        replayed = tuple(row[f"dshot{i}"] for i in range(1, scn.MOTORS + 1))
        if (row["k"], row["tick"], row["t_us"], replayed) != (x["k"], x["tick"], x["t_us"], logged):
            return {"execution": x["k"], "tick": x["tick"], "t_us": x["t_us"], "logged DShot": logged,
                    "replayed": {"k": row["k"], "tick": row["tick"], "t_us": row["t_us"], "DShot": replayed}}
    return None


def acro_segment_kinds(p):
    """'zero', 'single', 'reversal' or 'combined' per segment of an acro plan (section comment)."""
    kinds, prev = [], (0,) * len(l4s.AXES)
    for _, _, _, stick in p.segments:
        axes = [a for a, x in enumerate(stick) if x != 0]
        if not axes:
            kinds.append("zero")
        elif len(axes) > 1:
            kinds.append("combined")
        elif prev[axes[0]] * stick[axes[0]] < 0:
            kinds.append("reversal")
        else:
            kinds.append("single")
        prev = stick
    return kinds


def acro_phases(p):
    """(phase per execution 0 .. end_execution, [(segment number, kind, first execution, last execution, phase)]) of an
    acro plan (section comment). Raises PlanError on a script the rule does not cover."""
    kinds = acro_segment_kinds(p)
    starts = [seg[0] for seg in p.segments] + [p.end_execution + 1]
    phases = ["linear"] * (p.end_execution + 1)
    table, state = [], "linear"
    for i, kind in enumerate(kinds):
        first, last = starts[i], min(starts[i + 1], p.end_execution + 1) - 1
        if state == "linear":
            phase = "combined" if kind == "combined" else "linear"
            state = "after combined" if kind == "combined" else "linear"
        elif state == "after combined":
            if kind != "zero":
                raise PlanError([f"segment {i + 1} ({kind}) follows a combined segment: the recovery check needs a zero "
                                 "segment there"])
            phase, state = "recovery", "done"
        else:
            raise PlanError([f"segment {i + 1} ({kind}) follows the recovery segment: neither the rate bound nor the "
                             "recovery check covers it"])
        for k in range(first, last + 1):
            phases[k] = phase
        table.append((i + 1, kind, first, last, phase))
    if state == "after combined":
        raise PlanError(["the last segment is combined: no zero segment follows it for the recovery check"])
    return phases, table


def rest_bound(lo, hi):
    """Z(n) = max(|lo(n)|, |hi(n)|): the largest |w| over the grid whose envelope is (lo, hi)."""
    return [max(abs(a), abs(b)) for a, b in zip(lo, hi)]


def recovery_evaluation(window, w, e, z, f, fresh):
    """Recovery of one axis over the executions `window` (section comment); w, e (E), z (Z) and fresh are indexed by
    execution, f is F. The margin of execution n is Z(n) + F + E(n) - |w(n)|."""
    checked = [k for k in window if fresh[k]]
    if not checked:
        return {"passed": False, "reason": "no fresh execution in the recovery window"}
    margins = [(z[k] + f + e[k] - abs(w[k]), k) for k in checked]
    worst, at = min(margins)
    bad = [k for m, k in margins if m < 0]
    return {"passed": not bad, "window": [window[0], window[-1]], "worst margin": worst, "at execution": at,
            "w there": w[at], "Z + F + E there": z[at] + f + e[at], "violations": len(bad),
            "first violation": bad[0] if bad else None, "last violation": bad[-1] if bad else None,
            "fresh executions checked": len(checked)}


def mixer_abs_bm(params):
    """|B||M| in binary64 (rows and columns thrust, roll, pitch, yaw) from the parameters' binary32 values: B the rate
    loop's effectiveness (column i = [1, -y_i, x_i, s_i c_q]), M = mixer_m<i>_*."""
    motors = range(1, scn.MOTORS + 1)
    b = [[1.0 for _ in motors], [-params[f"rotor_position_m{i}_y"] for i in motors],
         [params[f"rotor_position_m{i}_x"] for i in motors],
         [params[f"rotor_yaw_sign_m{i}"] * params["rotor_torque_ratio"] for i in motors]]
    m = [[params[f"mixer_m{i}_{c}"] for c in MIXER_COLUMNS] for i in motors]
    return [[sum(abs(b[r][k]) * abs(m[k][c]) for k in range(scn.MOTORS)) for c in range(len(MIXER_COLUMNS))]
            for r in range(len(MIXER_COLUMNS))]


def flag_findings(rows, abs_bm):
    """(findings, counts) of the flag-consistency rule (section comment) over the replayed rows given. A finding is
    (execution, axis, rule, requested, achieved, b_a, flag)."""
    gamma = scn.MOTORS * EPS_F32 / (1 - scn.MOTORS * EPS_F32)
    findings = []
    counts = {a: {"executions": 0, "flag set": 0, "reduced beyond b_a": 0} for a in l4s.AXES}
    for row in rows:
        v = [row["ach_thrust"], row["ach_roll"], row["ach_pitch"], row["ach_yaw"]]
        for a, axis in enumerate(l4s.AXES):
            b = (gamma + EPS_F32) * sum(abs_bm[1 + a][c] * abs(v[c]) for c in range(len(v)))
            req, ach, flag = row[f"req_{axis}"], row[f"ach_{axis}"], row[f"flag_{axis}"]
            reduced = abs(req) - abs(ach) > b
            counts[axis]["executions"] += 1
            counts[axis]["flag set"] += flag
            counts[axis]["reduced beyond b_a"] += reduced
            if flag and not abs(ach) < abs(req):
                findings.append((row["k"], axis, "flag set, |achieved| not below |requested|", req, ach, b, flag))
            if reduced and not flag:
                findings.append((row["k"], axis, "|requested| - |achieved| > b_a, flag clear", req, ach, b, flag))
    return findings, counts


# ---- CLI ------------------------------------------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", required=True, metavar="FILE")
    ap.add_argument("--scenario", required=True, metavar="FILE")
    ap.add_argument("--m", type=int, help="one run at this m (default: the scenario's m_sequence)")
    ap.add_argument("--out-dir", required=True, metavar="DIR")
    ap.add_argument("--plugin-dir", default=str(DEFAULT_PLUGIN_DIR), metavar="DIR")
    args = ap.parse_args(argv)
    try:
        if args.m:
            s = run_step(args.card, args.scenario, args.m, args.out_dir, args.plugin_dir)
            print(f"report: {s.run.report_path}\nlog: {s.run.log_path}")
        else:
            runs, path = run_sequence(args.card, args.scenario, args.out_dir, args.plugin_dir)
            print(f"report: {path}")
            for s in runs:
                print(f"m = {s.run.m}: log {s.run.log_path}")
    except (gpc.GenError, scn.ScenarioError) as e:
        for line in e.lines:
            print(line, file=sys.stderr)
        return 1
    except (hover.HoverError, run_scenario.RunError) as e:
        print(f"run_l4: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
