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
  rotor_speed_rad_s     optional (decision 0007), marv_plant_config.initial_omega_rad_s: the scenario's 4 numbers, or for the
                        text "hover" the card's hover rotor speed per motor (hover_rotor_speeds): omega_i = sqrt(T_i / k),
                        T_i = M[i, thrust] m g(phi, h0), the firmware mixer's allocation at the hover collective and zero
                        torque (tools/card/mixer.py), k the card's thrust coefficient; for the text "steady_tumble" the
                        speeds at which marv_plant's torque at t = 0 equals w0 x J w0 (steady_tumble).
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
import attitude_lead  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import run_l4 as l4  # noqa: E402
import mixer  # noqa: E402
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
    rotor_speed_rad_s: tuple | None = None  # initial rotor speed per motor (logical order); None = at rest

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
            **({"initial rotor speed (rad/s)": list(self.rotor_speed_rad_s)} if self.rotor_speed_rad_s else {}),
        }


def hover_rotor_speeds(card, vals, root=ROOT):
    """The card's hover rotor speed per motor, rad/s, logical order: omega_i = sqrt(T_i / k), T_i = M[i, thrust] m g(phi, h0)
    (run_l4.hover_thrust times the mixer's thrust column: the allocation at the hover collective and zero torque), k the
    card's thrust coefficient. Raises PlanError if some omega_i lies outside the card's speed range."""
    card_doc, profile = gpc.load_linted(card, root)
    cfg, _ = gpc.config_from(card_doc, profile, card, root)
    m, _, _ = mixer.mixer_matrix(card_doc, card)
    thrust = l4.hover_thrust(card, vals, root)
    speeds = tuple(math.sqrt(m[i][0] * thrust / cfg["thrust_coeff"]) for i in range(scn.MOTORS))
    lo, hi = cfg["omega_min_rad_s"], cfg["omega_max_rad_s"]
    bad = [i + 1 for i, w in enumerate(speeds) if not lo <= w <= hi]
    if bad:
        raise PlanError([f"the hover rotor speed of motors {bad} is outside the card's speed range [{lo!r}, {hi!r}] rad/s"])
    return speeds


def euler_coupling(inertia, w):
    """w x (J w), J = diag(inertia): the body torque that holds the body rates w constant (Euler's equation
    J w' = tau - w x J w)."""
    jw = [j * x for j, x in zip(inertia, w)]
    return (w[1] * jw[2] - w[2] * jw[1], w[2] * jw[0] - w[0] * jw[2], w[0] * jw[1] - w[1] * jw[0])


def rotor_torque(card, speeds, root=ROOT):
    """The body torque (FRD, N m) of marv_plant v0 at the rotor speeds `speeds` (logical order): (B T)_axis for the roll,
    pitch and yaw rows of the card's effectiveness matrix B (tools/card/mixer.py, the plant's forward map) and
    T_i = k omega_i omega_i, in the plant's order of operations (plant_model.hpp Model::wrench)."""
    card_doc, profile = gpc.load_linted(card, root)
    cfg, _ = gpc.config_from(card_doc, profile, card, root)
    _, b, _ = mixer.mixer_matrix(card_doc, card)
    k = cfg["thrust_coeff"]
    thrust = [k * w * w for w in speeds]
    return tuple(sum(b[a][i] * thrust[i] for i in range(scn.MOTORS)) for a in range(1, len(mixer.AXES)))


def steady_tumble(card, vals, root=ROOT):
    """The steady-tumble rotor state of an L5 scenario (decision 0014, owner decision 5 and second round item 1): the rotor
    speeds at which every torque marv_plant v0 applies at t = 0 holds the body at its initial rates w0, so that
    J w' = tau - w0 x J w0 = 0 (Euler's equation; J the card's diagonal inertia).

      torque     marv_plant v0's body torque is (B T)_(roll, pitch, yaw), T_i = k omega_i^2 and B the card's effectiveness
                 matrix (tools/card/mixer.py; plant_model.hpp Model::wrench): the rotor thrust moments r_i x (0, 0, -T_i)
                 and the rotor yaw reaction s_i c_q T_i. The plant has no rotor inertia (no rotor gyroscopic torque), no
                 drag and no position-dependent torque, and its force acts at the centre of mass (the plugin refuses a
                 link whose CM is not its origin). So the rotors supply all of w0 x J w0 and nothing else is subtracted.
      thrusts    T(c) = M (c, w0 x J w0): the firmware mixer's inverse M = B^-1 (mixer.mixer_matrix) at collective c.
      c*         the hover collective m g(phi, h0) (run_l4.hover_thrust) if every T_i(m g) lies in the card's thrust range
                 [k omega_min^2, k omega_max^2] and every omega_i in [omega_min, omega_max]; otherwise the smallest
                 collective at which every rotor does. The floor is the card's own minimum rotor thrust k omega_min^2.
                 Closed form c_lo = max_i (k omega_min^2 - d_i) / M[i, thrust], d_i = sum_axis M[i, axis] (w0 x J w0)_axis,
                 then moved by single binary64 steps to the smallest double whose T_i(c) and omega_i pass the floor in
                 binary64 (T_i is nondecreasing in c, so the floor test is monotone; the closed form is within a few
                 roundings of that double).
      speeds     omega_i = sqrt(T_i(c*) / k).

    Raises PlanError if no collective keeps every rotor within the range: c_lo > c_hi = min_i (k omega_max^2 - d_i) /
    M[i, thrust], or the ceiling fails at c*. Returns {"speeds", "collective", "hover_collective", "hover_feasible",
    "thrusts", "torque", "thrust_range", "speed_range"}."""
    card_doc, profile = gpc.load_linted(card, root)
    cfg, _ = gpc.config_from(card_doc, profile, card, root)
    m, _, _ = mixer.mixer_matrix(card_doc, card)
    inertia = [float(x) for x in card_doc["inertia_diag"]["value"]]
    w0 = [float(x) for x in vals["initial_state"]["body_rates_frd_rad_s"]]
    torque = euler_coupling(inertia, w0)
    k = cfg["thrust_coeff"]
    w_lo, w_hi = cfg["omega_min_rad_s"], cfg["omega_max_rad_s"]
    f_lo, f_hi = k * w_lo * w_lo, k * w_hi * w_hi
    d = [sum(m[i][a + 1] * torque[a] for a in range(len(torque))) for i in range(scn.MOTORS)]

    def thrusts(c):
        return [m[i][0] * c + d[i] for i in range(scn.MOTORS)]

    def above_floor(c):
        return all(t >= f_lo and math.sqrt(t / k) >= w_lo for t in thrusts(c))

    def below_ceiling(c):
        return all(t <= f_hi and math.sqrt(t / k) <= w_hi for t in thrusts(c))

    hover = l4.hover_thrust(card, vals, root)
    c_lo = max((f_lo - d[i]) / m[i][0] for i in range(scn.MOTORS))
    c_hi = min((f_hi - d[i]) / m[i][0] for i in range(scn.MOTORS))
    if c_lo > c_hi:
        raise PlanError([f"steady tumble: no collective keeps every rotor in the card's thrust range [{f_lo!r}, {f_hi!r}] N "
                         f"(the floor needs c >= {c_lo!r} N, the ceiling c <= {c_hi!r} N)"])
    hover_ok = above_floor(hover) and below_ceiling(hover)
    c = hover
    if not hover_ok:
        c = c_lo
        while not above_floor(c):
            c = math.nextafter(c, math.inf)
        while above_floor(math.nextafter(c, -math.inf)):
            c = math.nextafter(c, -math.inf)
        if not below_ceiling(c):
            raise PlanError([f"steady tumble: at the smallest collective {c!r} N that keeps every rotor above the card's "
                             f"floor, a rotor exceeds the ceiling {f_hi!r} N"])
    t = thrusts(c)
    return {"speeds": tuple(math.sqrt(x / k) for x in t), "collective": c, "hover_collective": hover,
            "hover_feasible": hover_ok, "thrusts": t, "torque": torque, "thrust_range": (f_lo, f_hi),
            "speed_range": (w_lo, w_hi)}


def initial_rotor_speeds(card, vals, root=ROOT):
    """The scenario's initial rotor speed per motor (decision 0007): the 4 numbers, the keyword "hover" resolved by
    hover_rotor_speeds, "steady_tumble" by steady_tumble, or None when absent (the rotors at rest)."""
    rotor = vals["initial_state"].get("rotor_speed_rad_s")
    if rotor == l5s.ROTOR_SPEED_HOVER:
        return hover_rotor_speeds(card, vals, root)
    if rotor == l5s.ROTOR_SPEED_STEADY_TUMBLE:
        return steady_tumble(card, vals, root)["speeds"]
    return None if rotor is None else tuple(rotor)


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
    rotor = initial_rotor_speeds(card, vals, root)
    return Plan(num_us=num, den=den, divisor=divisor, ratio=ratio, tick_s=tick, period_s=period,
                settle_s=script["settle_s"], origin=a0, end=end, segments=segments, chirp=chirp,
                disturbance=disturbance, duration_ticks=duration, m_sequence=tuple(vals["m_sequence"]),
                thrust_n=l4.r32(thrust), thrust_n_double=thrust,
                rotor_speed_rad_s=None if rotor is None else tuple(rotor))


# ---- the world ------------------------------------------------------------------------------------------------------

def l2_scenario_doc(doc, p, stem, source, card=None):
    """The L2-schema scenario gen_world reads (run_l4.l2_scenario_doc's content for an L5 document). Nonzero initial body
    rates need the L2 schema's separatrix_margin_min (tools/sim/scenario.py): the L5 schema has no such field, because the
    check is about a torque-free rotation and an L5 run is not torque-free (the composition commands the motors). The runner
    therefore writes the initial state's own distance mu to the separatrix (card inertia) as the margin, derived, so the L2
    check passes by construction and the world input records mu."""
    vals = l5s.values(doc)

    def entry(value, unit, key):
        return {"value": value, "unit": unit, "label": "derived", "rule": f"{key} of {source} (tools/sim/run_l5.py)"}

    st = vals["initial_state"]
    rates = st["body_rates_frd_rad_s"]
    margin = {}
    if any(x != 0 for x in rates):
        if card is None:
            raise PlanError(["nonzero initial body rates: the card is needed for the L2 separatrix entry"])
        inertia = [float(x) for x in schema.load_yaml(card)["inertia_diag"]["value"]]
        margin = {"separatrix_margin_min": {
            "value": scn.separatrix_mu(inertia, rates), "unit": "1", "label": "derived",
            "rule": "the initial body rates' own distance mu to the torque-free separatrix of the card inertia "
                    "(tools/sim/scenario.py separatrix_mu): an L5 run is not torque-free, so the L2 margin is not a "
                    "property of it (tools/sim/run_l5.py l2_scenario_doc)"}}
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
        "initial_state": {**{k: entry(st[k], doc["initial_state"][k]["unit"], f"initial_state.{k}")
                             for k in scn.STATE_FIELDS},
                          **({"rotor_speed_rad_s": entry(list(p.rotor_speed_rad_s), "rad/s", "rotor_speed_rad_s")}
                             if getattr(p, "rotor_speed_rad_s", None) is not None else {})},
        **margin,
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
    l2_path.write_text(yaml.safe_dump(l2_scenario_doc(doc, p, stem, Path(scenario).name, card), sort_keys=False),
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
# Design quantities (chirp_design), from tools/card/attitude.py design() on the card, the budget and the scenario
# register; the build's att_kp, att_yaw_weight and att_loop_ratio must equal the design's
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

CHIRP_NEED = ("att_kp", "att_yaw_weight", "att_loop_ratio", *l4.CHIRP_NEED)
MIXER_COLUMNS = l4.MIXER_COLUMNS
PEAK_GRID_POINTS = l4.rate.GRID_POINTS  # the log grid of the peak torque search (a method constant: rate.py's grid)


@functools.lru_cache(maxsize=None)
def attitude_design(card, root=str(ROOT)):
    """attitude.design() on the card (owner decision 21: the attitude loop runs at the rate loop's rate, no U)."""
    card_doc = schema.load_yaml(card)
    budget = schema.load_yaml(Path(root) / "design" / "budget.yaml")
    register = schema.load_yaml(Path(root) / "design" / "scenario_values.yaml")
    return attitude.design(card_doc, budget, register, str(card))


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


def peak_torque_gain(rm, axis, k, n, band, loops, peak=None):
    """(max over the band and the box of steady_torque_peak, at omega, loop name): a log grid of PEAK_GRID_POINTS over the
    band, then a golden-section refinement around the largest grid point (run_l4.max_sensitivity's search). `peak`, when
    given, replaces steady_torque_peak: peak(loop name, omega)."""
    lo, hi = band
    grid = [lo * (hi / lo) ** (i / (PEAK_GRID_POINTS - 1)) for i in range(PEAK_GRID_POINTS)]
    best = (-math.inf, None, None)
    for name, jt, tau in loops:
        def f(w):
            if peak is not None:
                return peak(name, w)
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


def chirp_design(card, params, thrust_n, root=ROOT):
    """The design quantities of the section comment, per axis, as a dict. Raises PlanError on a stale build."""
    res = attitude_design(str(card), str(root))
    if (params.get("att_kp"), params.get("att_yaw_weight"), params.get("att_loop_ratio")) != (
            res["k32"], res["w32"], res["N"]):
        raise PlanError([f"the build's att_kp, att_yaw_weight, att_loop_ratio {params.get('att_kp')!r}, "
                         f"{params.get('att_yaw_weight')!r}, {params.get('att_loop_ratio')!r} are not the design's "
                         f"{res['k32']!r}, {res['w32']!r}, {res['N']!r} (tools/card/attitude.py on {card}): "
                         "the build is stale"])
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


# ---- the stage (c) design reference of the attitude chirp (decision 0014, commit 3) -----------------------------------
#
# From stage (c) on, the build's attitude parameters come from tools/card/attitude_lead.py (flatten.py --out-attitude-lead) on
# the stage (c) rate loop of tools/card/rate_lead.py, so run_chirp and gen_l5_chirp take chirp_design_lead; chirp_design and
# attitude.py stay the PI reference. chirp_design_lead's quantities are chirp_design's, on attitude_lead's loops in the
# configuration the chirp flies, the stage (c) T4 configuration (decision 0014; run_l4's chirp section): the truth gyro
# (latency 0) and the chain with every notch bypassed, its low-pass at the design's cutoff.
#   build check         the build's att_kp, att_yaw_weight and att_loop_ratio must equal attitude_lead.design()'s on the
#                       product inner loop (attitude_lead.lead_inner: notches at omega_th, the profile's latency; flatten.py's
#                       call), and its rate gains and gyro_lpf_cutoff_hz rate_lead's (run_l4.chirp_design's check); else the
#                       build is stale and refused.
#   loops               attitude_lead.build_loops on the T4 inner loop: rate_lead's f32 PID gains, latency 0, the chain's
#                       low-pass; N = 1 (attitude_lead refuses another N).
#   PM_design, w_c      attitude.pm_worst on those loops at the build's gain: the PM and the crossover theta / T_a of the
#                       nominal loop and the four tau x J corners of every axis, each certified stable by attitude_lead's
#                       rule step 5' (refused without a unique crossover or a certificate).
#   band, tau_held, A_a as chirp_design; G_tau(w) is steady_torque_peak_lead on the T4 loops.

@functools.lru_cache(maxsize=None)
def attitude_lead_design(card, root=str(ROOT)):
    """(rate_lead.design(), attitude_lead.design() on the product inner loop): flatten.py --out-attitude-lead's derivation of
    the build's attitude parameters (decision 0014, commit 3)."""
    lead = l4._lead_design(card, root)
    card_doc = schema.load_yaml(card)
    budget = schema.load_yaml(Path(root) / "design" / "budget.yaml")
    register = schema.load_yaml(Path(root) / "design" / "scenario_values.yaml")
    return lead, attitude_lead.design(card_doc, budget, register, str(card), attitude_lead.lead_inner(lead), lead["pi_l4"])


def steady_torque_peak_lead(loop, j_a, jt, k, omega):
    """|torque request| (N m) per unit steady sinusoid d at omega (rad/s) added to the rate setpoint at every rate execution,
    for the attitude_lead.LeadLoop `loop` (N = 1, corner inertia ratio jt, axis inertia j_a) closed with the attitude gain k:
    r = d - k theta, so x' = A_cl x + B d with A_cl = loop.closed(k), B = loop.b, and the steady state is x = X z^i,
    X = (z I - A_cl)^-1 B, z = e^(j omega T). The law's output u = kp e + I + D of an execution is what the execution leaves in
    the next state z X (LeadLoop.step: I, e_prev and the D filter are updated at the rate tick), in LeadLoop's units
    (torque / (j_a jt)); |z| = 1, so the peak is j_a jt |u(X)|."""
    z = cmath.exp(1j * omega * loop.t)
    a = loop.closed(k)
    x = attitude.csolve([[(z if r == c else 0) - a[r][c] for c in range(loop.n)] for r in range(loop.n)], loop.b)
    o = 3 + loop.lat + 2 * len(loop.stages)
    return abs(j_a * jt * (x[o] + loop.kp * x[o + 1] + (x[o + 3] if loop.kd else 0)))


@functools.lru_cache(maxsize=None)
def t4_lead_reference(card, root=str(ROOT)):
    """The design-model part of chirp_design_lead at the product design's gains (section comment): {"inner", "loops",
    "detail" (attitude.pm_worst's), "corners", "band", "design_pm", "design_crossover", "peaks" (axis -> peak_torque_gain)}.
    Raises PlanError."""
    lead, res = attitude_lead_design(card, root)
    m, ch = lead["model"], lead["chain"]
    gcd = l4.gcd
    stages = [s for s in gcd.chain_stages(ch["t_s"], ch["f_c"], ch["q"], ch["omega_th"], [ch["omega_th"]] * gcd.MOTORS,
                                          bypass_all=True) if s != gcd.IDENTITY]
    inner = attitude_lead.Inner(f"stage (c) T4: rate_lead's f32 PID (N* {lead['n_star']!r}), notches bypassed, latency 0",
                                lead["axes32"], m.t_s, m.divisor, 0, stages)
    rr, t_a = res["rate"], res["T_a"]
    loops = attitude_lead.build_loops(rr, inner)
    _, detail = attitude.pm_worst(loops, res["k32"], res["w32"])
    bad = [f"{a} {name}: the phase of F fails attitude_lead's start or branch check" for a, name, lp in loops
           if not (lp.start_ok and lp.branch_ok)]
    bad += [f"{a} {name}: {why}" for a, name, _, _, _, why in detail if why]
    if bad:
        raise PlanError([f"the stage (c) T4 attitude design loop at att_kp {res['k32']!r}: {x}" for x in bad])
    crossover = {a: {} for a in l5s.AXES}
    design_pm = {a: {} for a in l5s.AXES}
    for axis, name, pm, theta, _, _ in detail:
        crossover[axis][name], design_pm[axis][name] = theta / t_a, pm
    every = [w for c in crossover.values() for w in c.values()]
    band = (min(every) / rr["a"], rr["a"] * max(every))
    corners = l4.rate.corner_list(rr["inputs"]["tau"], rr["inputs"]["b_tau"], rr["inputs"]["b_J"])
    jt_of = {name: jt for name, jt, _ in corners}
    by = {(a, name): lp for a, name, lp in loops}
    peaks = {}
    for axis in l5s.AXES:
        kk = attitude.axis_k(axis, res["k32"], res["w32"])
        j_a = rr["axes"][axis]["J"]

        def peak(name, w, axis=axis, kk=kk, j_a=j_a):
            return steady_torque_peak_lead(by[axis, name], j_a, jt_of[name], kk, w)
        peaks[axis] = peak_torque_gain(rr, axis, kk, res["N"], band, corners, peak)
    return {"inner": inner, "loops": loops, "detail": detail, "corners": corners, "band": band, "design_pm": design_pm,
            "design_crossover": crossover, "peaks": peaks}


def chirp_design_lead(card, params, thrust_n, root=ROOT):
    """chirp_design's quantities on the stage (c) design reference (section comment), per axis, as a dict; "result" holds the
    T4 loop's N, T_a, k32, w32, rate (rate.py's result), inner (attitude_lead.Inner), detail, and the product design.
    Raises PlanError on a stale build."""
    lead, res = attitude_lead_design(str(card), str(root))
    fails = []
    if (params.get("att_kp"), params.get("att_yaw_weight"), params.get("att_loop_ratio")) != (
            res["k32"], res["w32"], res["N"]):
        fails.append(f"the build's att_kp, att_yaw_weight, att_loop_ratio {params.get('att_kp')!r}, "
                     f"{params.get('att_yaw_weight')!r}, {params.get('att_loop_ratio')!r} are not the design's "
                     f"{res['k32']!r}, {res['w32']!r}, {res['N']!r} (tools/card/attitude_lead.py on {card}): the build is stale")
    for a, gains32 in zip(l5s.AXES, lead["axes32"]):
        for q, v in zip(l4.LEAD_GAINS, gains32):
            if params.get(f"rate_{q}_{a}") != v:
                fails.append(f"the build's rate_{q}_{a} {params.get(f'rate_{q}_{a}')!r} is not the design's {v!r} "
                             f"(tools/card/rate_lead.py on {card}): the build is stale")
    if params.get("gyro_lpf_cutoff_hz") != l4.r32(lead["chain"]["f_c"]):
        fails.append(f"the build's gyro_lpf_cutoff_hz {params.get('gyro_lpf_cutoff_hz')!r} is not the design's "
                     f"{l4.r32(lead['chain']['f_c'])!r} (tools/card/rate_lead.py on {card}): the build is stale")
    if fails:
        raise PlanError(fails)
    ref = t4_lead_reference(str(card), str(root))
    rr = res["rate"]
    f_min = params["rotor_thrust_coeff"] * params["idle_speed"] ** 2
    f_max = params["rotor_thrust_coeff"] * params["rotor_speed_max"] ** 2
    rows = l4._mixer_rows(params)
    axes = {}
    for axis in l5s.AXES:
        g, g_at, g_loop = ref["peaks"][axis]
        held = l4.tau_held(rows, MIXER_COLUMNS.index(axis), thrust_n, f_min, f_max)
        if held is None or not held > 0:
            raise PlanError([f"the hover thrust {thrust_n!r} N leaves no held-collective torque on {axis}"])
        axes[axis] = {"k": attitude.axis_k(axis, params["att_kp"], params["att_yaw_weight"]), "peak_torque_per_rad_s": g,
                      "peak_at_rad_s": g_at, "peak_loop": g_loop, "tau_held_nm": held, "amplitude_rad_s": held / g}
    result = {"N": res["N"], "T_a": res["T_a"], "k32": res["k32"], "w32": res["w32"], "rate": rr, "inner": ref["inner"],
              "detail": ref["detail"], "product": res}
    return {"T_a": res["T_a"], "N": res["N"], "a": rr["a"], "band": ref["band"], "design_pm": ref["design_pm"],
            "design_crossover": ref["design_crossover"], "pm_min": rr["inputs"]["PM_min"], "loops": ref["corners"],
            "axes": axes, "result": result}


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
              extra_edit=None, seed=None, root=ROOT, timeout_s=run_scenario.TIMEOUT_S):
    """One gz process of an L5 chirp scenario at m ticks per host step (run_step), with the amplitude scaled by amp_scale in
    (0, 1] and the duration halved when `halved`; extracts the window's y and d. `overrides` are harness sil_overrides."""
    if not 0 < amp_scale <= 1:
        raise PlanError([f"amp_scale {amp_scale!r} is not in (0, 1]"])
    c = l5s.values(l5s.load(scenario))["script"]["chirp"]
    _, defaults = build_parameters(plugin_dir)
    params = l4.read_param_defaults(defaults)
    harness = dict(overrides or {})
    doc_edit = None
    if amp_scale != 1 or halved:
        def doc_edit(doc):
            ch = doc["script"]["chirp"]
            ch["amp_rad_s"]["value"] = l4.r32(l4.r32(c["amp_rad_s"]) * amp_scale)
            ch["duration_s"]["value"] = c["duration_s"] / 2 if halved else c["duration_s"]
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
                    design=chirp_design_lead(card, params, p.thrust_n, root),
                    stamps=stamps, y=y, d=d, dshot_end_executions=ends)


# ---- the margin and U of a chirp axis (owner decision 20) --------------------------------------------------------------
#
# Yaw (and the L4 rule): PM is the (m = 1, A) run's, E_H = |PM(m = 1) - PM(m = 2)|, U_A = |PM(A) - PM(A/2)|.
# Roll and pitch (owner decision 20: the torque-envelope amplitude A_env leaves the linear regime, peak attitude error 2 at
# A_env): runs k = 1..5 at m = 1 and amplitude A_env / 2^k; PM = the minimum over the k that give a unique crossover, U_A =
# max - min of the PM over those k (the plateau's spread), E_H = |PM - PM(m = 2)| of the run at the minimum's k and amplitude,
# U_d that of that run. U = E_H + U_A + U_d on every axis.
# The plateau's admission (owner decision of 2026-10-02, decision 0014 third round item 2, amending decision 20's plateau):
# only the k that chirp_admission admits enter the plateau, and the plateau must hold at least PLATEAU_MIN_RUNS of them
# (section "the plateau admission" below).
PLATEAU_K = tuple(range(1, 6))
ROLL_PITCH = ("roll", "pitch")
PLATEAU_MIN_RUNS = 3  # owner decision (decision 0014 third round item 2): "The plateau must hold at least three runs"


def plateau_name(k):
    return f"k{k}"


def u_terms(axis, pm, admitted=None):
    """{"margin_run", "pm", "E_H", "U_A"} (rad) from `pm`, run name -> PM (rad) or None (no unique crossover): yaw's runs are
    m1, m2, half_amplitude; roll and pitch's are k1 .. k5 (m = 1) and m2 (m = 2, at the minimum's amplitude). `admitted`, when
    given, are the run names chirp_admission admits: the plateau is those of them with a crossover, at least
    PLATEAU_MIN_RUNS."""
    if axis not in ROLL_PITCH:
        return {"margin_run": "m1", "pm": pm["m1"], "E_H": abs(pm["m1"] - pm["m2"]),
                "U_A": abs(pm["m1"] - pm["half_amplitude"])}
    plateau = {plateau_name(k): pm[plateau_name(k)] for k in PLATEAU_K if pm.get(plateau_name(k)) is not None
               and (admitted is None or plateau_name(k) in admitted)}
    if admitted is not None and len(plateau) < PLATEAU_MIN_RUNS:
        raise PlanError([f"{axis}: the plateau holds {len(plateau)} admitted runs with a unique crossover, it needs "
                         f"{PLATEAU_MIN_RUNS}"])
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


# ---- the plateau admission of roll and pitch (owner decision of 2026-10-02, decision 0014 third round item 2) ------------
#
# It amends decision 20's plateau. Run k (amplitude A_env / 2^k) enters the plateau iff its modelled nonlinear PM shift is
# smaller than the PM change the fine negative control (attitude gains x 1.1) produces on the design model at the same corner:
# a run whose nonlinearity is as large as the fine fault cannot tell the two apart. Excluded runs are still flown and reported.
#   corner              the plant the chirp flies: the nominal card plant (the build's f32 J and tau), in the stage (c) T4
#                       configuration (truth gyro, latency 0, every notch bypassed: the chain's low-pass alone), at the
#                       build's f32 rate and attitude gains.
#   shift_k             PM_nl(k) - PM_lin (chirp_model_pm), each identified from the model's own window as chirp_margin
#                       identifies a flown run (run_l4.identify: C the axis gain, T_a, the f32 band; the chirp at the plan's
#                       stamps from the window's first execution; the window's length). PM_nl(k): the design model of the T4
#                       recovery envelope (tests/regression/quad/L05/gz/recovery_model.py on the L5 T3 oracle's pieces: per
#                       axis the exact ZOH plant J w' = u_m, tau u_m' = u - u_m; the chain's low-pass seeded with the first
#                       sample; the rate law in bypass with its D low-pass at the stamp dt; recovery_model.law, the attitude
#                       law on the full quaternion; the quaternion integrated per tick; no w x Jw), driven at A_env / 2^k, its
#                       torque request through recovery_model.Quant3 (the firmware mixer's allocation at the scenario
#                       collective, DShot rounding, marv_plant's ESC map and rotor geometry): kinematics and quantisation
#                       together. PM_lin: the same loop with the axis's angle theta in place of the law (r = -C theta on the
#                       axis, 0 on the others) and no quantiser, the linear design model. Its loop is linear, so its PM does
#                       not depend on the amplitude and one run, at k = 1's amplitude, serves every k. The method of the
#                       stage (c) chirp diagnosis (a1_model.py, 2026-10-02), ported operation for operation.
#   threshold           |PM(f32(att_kp f32(1.1))) - PM(att_kp)| (fine_control_pm_change): attitude.Loop.margin of the axis's
#                       nominal loop on chirp_design_lead's T4 inner loop (attitude_lead.build_loops), the gain scaled as
#                       t3_test.cpp scales the fine control.
#   admitted            |shift_k| < threshold (plateau_admission). A run whose model gives no unique crossover has no shift
#                       and is excluded.
FINE_GAIN_SCALE = 1.1  # the fine negative control "gains x 1.1" (core 7.2; quad spec 4 L5 T3; t3_test.cpp kGainScale)
RECOVERY_MODEL_DIR = ROOT / "tests" / "regression" / "quad" / "L05" / "gz"


def _recovery_model():
    """tests/regression/quad/L05/gz/recovery_model.py (it imports the L5 T3 oracle as recovery_model.oracle)."""
    if str(RECOVERY_MODEL_DIR) not in sys.path:
        sys.path.append(str(RECOVERY_MODEL_DIR))
    import recovery_model  # noqa: PLC0415
    return recovery_model


def quant_inputs(card, params, thrust_n, root=ROOT):
    """The quantiser's inputs (attitude_t3_oracle.Q_KEYS) for a run: the build's mixer parameters (f32), the scenario
    collective thrust_n (N) and the card's plant side, mapped as attitude_t3_oracle.refresh_q_inputs maps them."""
    oracle = _recovery_model().oracle
    cfg = gpc.plant_config(card, root)
    out = {k: params[k] for k in oracle.Q_FW_KEYS if k != "l5_thrust_n"}
    out.update(l5_thrust_n=thrust_n, plant_thrust_coeff=cfg["thrust_coeff"], plant_omega_min=cfg["omega_min_rad_s"],
               plant_omega_max=cfg["omega_max_rad_s"], plant_torque_ratio=cfg["torque_ratio_m"])
    for i, (pos, sign) in enumerate(zip(cfg["rotor_position_frd_m"], cfg["yaw_sign"]), 1):
        out.update({f"plant_rotor{i}_x": pos[0], f"plant_rotor{i}_y": pos[1], f"plant_rotor{i}_yaw_sign": float(sign)})
    return out


def chirp_model_pm(su, qf, axis, amp, w_lo, w_hi, dur_us, n_exec, *, linear=False):
    """run_l4.identify's result (plus "peak |y|", rad) for the design model of section "the plateau admission" driven by the
    chirp (amp rad/s, w_lo, w_hi rad/s, dur_us from the window's first execution) on `axis` for n_exec attitude executions:
    su = attitude_t3_oracle.Setup on the build's parameter table, qf = quant_inputs; `linear` runs PM_lin's model."""
    rm = _recovery_model()
    oracle = rm.oracle
    idx = l5s.AXES.index(axis)
    p, cfg = su.p, su.cfg
    n_ticks = su.divisor * su.ratio
    plants = [oracle.Axis(p[oracle.INERTIA[n]], p["motor_tau"], su.tick_s) for n in oracle.AXES]
    kp = [p[f"rate_kp_{a}"] for a in oracle.AXES]
    ki = [p[f"rate_ki_{a}"] for a in oracle.AXES]
    kd = [p[f"rate_kd_{a}"] for a in oracle.AXES]
    b0, b1, b2, f1, f2 = su.lpf
    qz = None if linear else rm.Quant3(qf)
    chain, y_tick = [None] * 3, [0.0] * 3
    integral, e_prev, u = [0.0] * 3, [0.0] * 3, [0.0] * 3
    y_prev, d_f = [0.0] * 3, [0.0] * 3
    q = q_sp = (1.0, 0.0, 0.0, 0.0)
    ys, ds = [], []
    for j in range(n_exec * n_ticks):
        for i, pl in enumerate(plants):
            x = pl.w
            x1, x2, yy1, yy2 = chain[i] if j > 0 else (x, x, x, x)
            yv = b0 * x + b1 * x1 + b2 * x2 - f1 * yy1 - f2 * yy2
            chain[i] = (x, x1, yv, yy1)
            y_tick[i] = yv
        if j % n_ticks == 0:
            a = j // n_ticks
            dd = l4.chirp_value(su.stamp_us(j), amp, w_lo, w_hi, 0, dur_us)
            if linear:
                yerr = plants[idx].th
                r_hold = [0.0, 0.0, 0.0]
                r_hold[idx] = -cfg.kp * yerr
            else:
                yerr = attitude_error(q, cfg.w)[idx]
                r_hold = rm.law(cfg, q, q_sp)
            ys.append(yerr)
            ds.append(dd)
            r_hold[idx] += dd
            if a == 0:
                u = [0.0] * 3 if qz is None else qz([0.0, 0.0, 0.0])
                y_prev = list(y_tick)
            else:
                dt = su.dt_exec(a)
                req = [0.0] * 3
                for i in range(3):
                    alpha = su.alpha(dt, i)[0]
                    integral[i] += ki[i] * e_prev[i] * dt
                    e = r_hold[i] - y_tick[i]
                    d_raw = -kd[i] * (y_tick[i] - y_prev[i]) / dt
                    d_f[i] = d_f[i] + alpha * (d_raw - d_f[i]) if alpha != 1.0 else d_raw
                    req[i] = kp[i] * e + integral[i] + d_f[i]
                    e_prev[i] = e
                    y_prev[i] = y_tick[i]
                u = req if qz is None else qz(req)
        d3 = [pl.step(u[i]) for i, pl in enumerate(plants)]
        q = oracle.qmul(q, oracle.quat_exp(d3))
        nq = math.sqrt(sum(c * c for c in q))
        q = tuple(c / nq for c in q)
    out = l4.identify(ys, ds, su.t_a, attitude.axis_k(axis, p["att_kp"], p["att_yaw_weight"]), 0.0, (w_lo, w_hi))
    out["peak |y|"] = max(abs(v) for v in ys)
    return out


def fine_control_pm_change(result, axis):
    """(threshold, PM at att_kp, PM at the fine control's gain) (rad) of section "the plateau admission": attitude.Loop.margin
    of the nominal loop of `axis` on chirp_design_lead's T4 inner loop (result = its "result"). Raises PlanError without a
    unique crossover."""
    loop = next(lp for a, name, lp in attitude_lead.build_loops(result["rate"], result["inner"], (axis,))
                if a == axis and name == "nominal")
    pms = []
    for k in (result["k32"], l4.r32(result["k32"] * l4.r32(FINE_GAIN_SCALE))):
        _, pm, why = loop.margin(attitude.axis_k(axis, k, result["w32"]))
        if pm is None:
            raise PlanError([f"{axis} nominal at att_kp {k!r}: {why}"])
        pms.append(pm)
    return abs(pms[1] - pms[0]), pms[0], pms[1]


def plateau_admission(shifts, threshold):
    """The run names admitted to the plateau from `shifts`, name -> shift (rad) or None: |shift| < threshold."""
    return tuple(name for name, s in shifts.items() if s is not None and abs(s) < threshold)


def chirp_admission(card, scenario, plugin_dir=DEFAULT_PLUGIN_DIR, *, params=None, result=None, root=ROOT):
    """Section "the plateau admission" for a roll or pitch chirp scenario: {"axis", "threshold", "pm_design", "pm_fine",
    "pm_linear", "runs" (plateau_name(k) -> {"amp", "pm", "shift"}), "admitted"} (rad, rad/s; pm None without a unique
    crossover). params: the build's parameter table (default: plugin_dir's build, as run_chirp reads it); result:
    chirp_design_lead's "result" (default: computed on params, which refuses a stale build)."""
    if params is None:
        _, defaults = build_parameters(plugin_dir)
        params = l4.read_param_defaults(defaults)
    p = plan(l5s.load(scenario), params, card, root)
    c = p.chirp
    axis = c["axis"]
    if axis not in ROLL_PITCH:
        raise PlanError([f"{scenario}: the plateau admission is roll and pitch's (owner decision 20), not {axis}'s"])
    if result is None:
        result = chirp_design_lead(card, params, p.thrust_n, root)["result"]
    threshold, pm_design, pm_fine = fine_control_pm_change(result, axis)
    su = _recovery_model().oracle.Setup(params)
    qf = quant_inputs(card, params, p.thrust_n, root)
    a_env, w_lo, w_hi = (l4.r32(c[key]) for key in ("amp_rad_s", "w_lo_rad_s", "w_hi_rad_s"))
    amps = {plateau_name(k): l4.r32(a_env * 2.0 ** -k) for k in PLATEAU_K}
    n_exec = p.end + 1
    lin = chirp_model_pm(su, qf, axis, amps[plateau_name(PLATEAU_K[0])], w_lo, w_hi, c["dur_us"], n_exec, linear=True)
    if lin["pm"] is None:
        raise PlanError([f"{axis}: the linear design model has no unique crossover in the band ({lin['reason']})"])
    runs = {}
    for name, amp in amps.items():
        pm = chirp_model_pm(su, qf, axis, amp, w_lo, w_hi, c["dur_us"], n_exec)["pm"]
        runs[name] = {"amp": amp, "pm": pm, "shift": None if pm is None else pm - lin["pm"]}
    return {"axis": axis, "threshold": threshold, "pm_design": pm_design, "pm_fine": pm_fine, "pm_linear": lin["pm"],
            "runs": runs, "admitted": plateau_admission({n: r["shift"] for n, r in runs.items()}, threshold)}


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
