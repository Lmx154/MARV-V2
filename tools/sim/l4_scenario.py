"""Scenario schema of the quad L4 T4 runs (scenarios/quad/L04/*.yaml; quad spec section 4 L4, docs/decisions/0005 "T4").

Separate from the frozen L2 schema (tools/sim/scenario.py), whose entry rules it follows: every number sits in an entry
{value, unit, label, rationale | rule} and nowhere else; a bare number outside an entry, or an entry without a label,
is refused (core contracts section 2).

  label: scenario   a labelled scenario value; `rationale` is required, one line.
  label: derived    `rule` is required, one line: how the value follows from other entries or a cited item.

Required fields (no others are allowed):

  scenario                 the file stem
  site_latitude_rad        geodetic latitude of the NED origin, rad, |value| <= pi/2
  site_height_m            height h0 of the NED origin above the WGS 84 ellipsoid, m
  seed                     integer >= 0, reserved at v0 (marv_plant v0 and the composition draw no random numbers)
  tick_period_num_us       integer > 0, label derived: equal to design/scenario_values.yaml tick_period_num_us
  tick_period_den          integer > 0, label derived: equal to design/scenario_values.yaml tick_period_den
  m_sequence               exactly [2, 1], label derived: the two runs of decision 0005 "T4 envelope widening"
                           (E_i = |y(m=1) - y(m=2)|, the last halving of 0003 item 9)
  initial_state
    position_ned_m         3 numbers, NED, origin at the site
    attitude_q_wxyz        4 numbers, Hamilton, body FRD -> NED, w >= 0, | |q|^2 - 1 | <= 4 eps_float (the ABI bound)
    velocity_ned_m_s       3 numbers
    body_rates_frd_rad_s   3 numbers, all zero: the step starts from rest (the design-model response starts at zero)
  step                     exactly these four entries:
    axis                   one of roll, pitch, yaw (a category, in an entry)
    rate_rad_s             number > 0, label derived: equal to design/scenario_values.yaml rate_max_<axis>; the other two
                           axes are 0
    settle_s               number > 0, s: the step's first rate-loop execution is the first one at or after settle_s
                           that keeps the T3 stamp phase (tools/sim/run_l4.py plan())
    horizon_tau_ref        number > 0, label derived: the T3 horizon rule, the window is 1 + ceil(horizon tau_ref / T)
                           executions (tests/regression/quad/L04/t3/README.md)

The derived entries whose source is the scenario-values register (the tick and the step rate) are checked against it
here; the entries that follow from the live parameters of a build (tau_ref, the rate-loop divisor) are checked by the
runner against the plugin build's parameter table.
"""

from __future__ import annotations

import math
import sys
from fractions import Fraction
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "card"))
import scenario as scn  # noqa: E402
import schema  # noqa: E402

REGISTER = ROOT / "design" / "scenario_values.yaml"
TOP_FIELDS = (
    "scenario", "site_latitude_rad", "site_height_m", "seed", "tick_period_num_us", "tick_period_den", "m_sequence",
    "initial_state", "step",
)
STEP_FIELDS = ("axis", "rate_rad_s", "settle_s", "horizon_tau_ref")
AXES = ("roll", "pitch", "yaw")
M_SEQUENCE = [2, 1]
REGISTER_TICK = ("tick_period_num_us", "tick_period_den")
RATE_MAX = "rate_max_{}"
DERIVED = "derived"


def _register_value(register, key, fails):
    entry = register.get(key) if isinstance(register, dict) else None
    if not isinstance(entry, dict) or "value" not in entry:
        fails.append(f"{key}: the scenario-values register has no entry {key!r}")
        return None
    return entry["value"]


def _need_derived(e, path, fails):
    if e is not None and e.get("label") != DERIVED:
        fails.append(f"{path}: label must be {DERIVED} (the value is read from its rule's source)")


def validate(doc, name, register):
    """The findings for the parsed L4 scenario `doc` of file `name` against the parsed register; empty means valid."""
    fails = []
    if not isinstance(doc, dict):
        return [f"{name}: not a mapping"]

    bare = []
    scn._bare_numbers({k: v for k, v in doc.items() if k != "scenario"}, "", bare)
    for p in bare:
        fails.append(f"{p}: unlabelled number (every number sits in an entry with a label)")
    for k in doc:
        if k not in TOP_FIELDS:
            fails.append(f"{k}: unknown field")
    for k in TOP_FIELDS:
        if k not in doc:
            fails.append(f"{k}: missing field")
    if doc.get("scenario") != Path(name).stem:
        fails.append(f"scenario: {doc.get('scenario')!r} must equal the file stem {Path(name).stem!r}")

    def scalar(parent, key, path, check, why):
        if key not in parent:
            return None, None
        e = scn._entry(parent, key, path, fails)
        if e is None:
            return None, None
        if not check(e["value"]):
            fails.append(f"{path}: value {e['value']!r} {why}")
            return e, None
        return e, e["value"]

    scalar(doc, "site_latitude_rad", "site_latitude_rad", lambda v: scn.is_num(v) and abs(v) <= math.pi / 2,
           "must be a number with |value| <= pi/2")
    scalar(doc, "site_height_m", "site_height_m", scn.is_num, "must be a finite number")
    scalar(doc, "seed", "seed", lambda v: scn.is_int(v) and v >= 0, "must be an integer >= 0")
    for key in REGISTER_TICK:
        e, v = scalar(doc, key, key, lambda x: scn.is_int(x) and x > 0, "must be an integer > 0")
        _need_derived(e, key, fails)
        if v is not None:
            want = _register_value(register, key, fails)
            if want is not None and v != want:
                fails.append(f"{key}: value {v!r} differs from the scenario-values register's {want!r}")

    if "m_sequence" in doc:
        e = scn._entry(doc, "m_sequence", "m_sequence", fails)
        _need_derived(e, "m_sequence", fails)
        if e is not None and e["value"] != M_SEQUENCE:
            fails.append(f"m_sequence: value {e['value']!r} must be {M_SEQUENCE} (decision 0005: m = 1 against m = 2)")

    state = doc.get("initial_state")
    if "initial_state" in doc:
        if not isinstance(state, dict):
            fails.append("initial_state: not a mapping")
        else:
            for k in state:
                if k not in scn.STATE_FIELDS:
                    fails.append(f"initial_state.{k}: unknown field")
            for k in scn.STATE_FIELDS:
                path = f"initial_state.{k}"
                if k not in state:
                    fails.append(f"{path}: missing field")
                    continue
                e = scn._entry(state, k, path, fails)
                v = scn._vector(e, path, scn.STATE_LENGTH[k], fails) if e is not None else None
                if v is None:
                    continue
                if k == "attitude_q_wxyz":
                    if v[0] < 0:
                        fails.append(f"{path}: w = {v[0]!r} < 0; canonical sign is w >= 0")
                    n2 = sum(x * x for x in v)
                    if abs(n2 - 1.0) > scn.QUAT_NORM_TOL:
                        fails.append(f"{path}: | |q|^2 - 1 | = {abs(n2 - 1.0)!r} exceeds {scn.QUAT_NORM_TOL!r}")
                if k == "body_rates_frd_rad_s" and any(x != 0 for x in v):
                    fails.append(f"{path}: must be zero; the step starts from rest")

    step = doc.get("step")
    if "step" in doc:
        if not isinstance(step, dict):
            fails.append("step: not a mapping")
        else:
            for k in step:
                if k not in STEP_FIELDS:
                    fails.append(f"step.{k}: unknown field")
            for k in STEP_FIELDS:
                if k not in step:
                    fails.append(f"step.{k}: missing field")
            _, axis = scalar(step, "axis", "step.axis", lambda v: v in AXES, f"must be one of {list(AXES)}")
            e, rate = scalar(step, "rate_rad_s", "step.rate_rad_s", lambda v: scn.is_num(v) and v > 0,
                             "must be a finite number > 0")
            _need_derived(e, "step.rate_rad_s", fails)
            if rate is not None and axis is not None:
                key = RATE_MAX.format(axis)
                want = _register_value(register, key, fails)
                if want is not None and rate != want:
                    fails.append(f"step.rate_rad_s: value {rate!r} differs from the scenario-values register's {key} "
                                 f"{want!r}")
            scalar(step, "settle_s", "step.settle_s", lambda v: scn.is_num(v) and v > 0, "must be a finite number > 0")
            e, _ = scalar(step, "horizon_tau_ref", "step.horizon_tau_ref", lambda v: scn.is_num(v) and v > 0,
                          "must be a finite number > 0")
            _need_derived(e, "step.horizon_tau_ref", fails)
    return fails


def load(path, register_path=REGISTER):
    """Parse and validate an L4 scenario file; return its document. Raises scenario.ScenarioError with the findings."""
    try:
        doc = schema.load_yaml(path)
        register = schema.load_yaml(register_path)
    except Exception as e:  # OSError or yaml.YAMLError (duplicate keys included)
        raise scn.ScenarioError([f"{path}: {e}"]) from e
    fails = validate(doc, path, register)
    if fails:
        raise scn.ScenarioError([f"{path}: {f}" for f in fails])
    return doc


def values(doc):
    """The plain values of a valid L4 scenario: dict of field -> value (state and step flattened one level)."""
    v = {k: doc[k]["value"] for k in TOP_FIELDS if k not in ("scenario", "initial_state", "step")}
    v["scenario"] = doc["scenario"]
    v["initial_state"] = {k: doc["initial_state"][k]["value"] for k in scn.STATE_FIELDS}
    v["step"] = {k: doc["step"][k]["value"] for k in STEP_FIELDS}
    return v


# ---- the chirp and acro scenarios (quad spec 4 L4 T4: chirp margins meet QF-3; acro scripted stick sequence) --------
#
# Two more scenario kinds share the fields above except `step`, which each replaces by its own section. validate(),
# load() and values() above are the step scenarios' and are unchanged; the chirp and acro kinds have their own.
#
#   chirp (scenarios/quad/L04/chirp_<axis>.yaml): one torque chirp at the plant input, setpoint 0 (tools/sim/run_l4.py
#   plan_chirp derives the band, the amplitude and the stamps from the build):
#     axis               one of roll, pitch, yaw
#     settle_s           number > 0, s: the chirp starts at the first rate-loop execution at or after settle_s
#     duration_start_s   number > 0, s, label scenario: D_0 of the duration convergence (core 7.5)
#     duration_s         number > 0, s, label derived: D_0 2^j with j >= 1 (the converged D; D/2 >= D_0 is the previous
#                        doubling), a whole number of microseconds below 2^31 (the composition's i32 l4_chirp_dur_us)
#     tail_s             number > 0, s: the identification window runs tail_s past the chirp's end
#   acro (scenarios/quad/L04/acro.yaml): a scripted stick sequence at hover collective (run_l4.plan_acro):
#     segments           a list of at least one mapping {start_s, stick}: start_s > 0 and strictly increasing, s;
#                        stick 3 numbers in [-1, 1], the setpoint as a fraction of the build's rate_max_<axis>
#     end_s              number > the last start_s, s: the run's end
# In both kinds initial_state.body_rates_frd_rad_s must be zero: the rate loop's prefilter is seeded from the first
# gyro sample, so a run from rest keeps the chirp's reference exactly zero and the acro script's first step from rest.

CHIRP_TOP_FIELDS = TOP_FIELDS[:-1] + ("chirp",)
CHIRP_FIELDS = ("axis", "settle_s", "duration_start_s", "duration_s", "tail_s")
ACRO_TOP_FIELDS = TOP_FIELDS[:-1] + ("acro",)
ACRO_FIELDS = ("segments", "end_s")
SEGMENT_FIELDS = ("start_s", "stick")
I32_LIMIT_US = 2 ** 31  # the composition's l4_chirp_dur_us and l4_seg<k>_t_us are i32


def _positive(v):
    return scn.is_num(v) and v > 0


def _validate_common(doc, name, register, top_fields, fails):
    """The fields every L4 kind shares, checked as validate() checks them; returns the scalar helper."""
    bare = []
    scn._bare_numbers({k: v for k, v in doc.items() if k != "scenario"}, "", bare)
    for p in bare:
        fails.append(f"{p}: unlabelled number (every number sits in an entry with a label)")
    for k in doc:
        if k not in top_fields:
            fails.append(f"{k}: unknown field")
    for k in top_fields:
        if k not in doc:
            fails.append(f"{k}: missing field")
    if doc.get("scenario") != Path(name).stem:
        fails.append(f"scenario: {doc.get('scenario')!r} must equal the file stem {Path(name).stem!r}")

    def scalar(parent, key, path, check, why):
        if key not in parent:
            return None, None
        e = scn._entry(parent, key, path, fails)
        if e is None:
            return None, None
        if not check(e["value"]):
            fails.append(f"{path}: value {e['value']!r} {why}")
            return e, None
        return e, e["value"]

    scalar(doc, "site_latitude_rad", "site_latitude_rad", lambda v: scn.is_num(v) and abs(v) <= math.pi / 2,
           "must be a number with |value| <= pi/2")
    scalar(doc, "site_height_m", "site_height_m", scn.is_num, "must be a finite number")
    scalar(doc, "seed", "seed", lambda v: scn.is_int(v) and v >= 0, "must be an integer >= 0")
    for key in REGISTER_TICK:
        e, v = scalar(doc, key, key, lambda x: scn.is_int(x) and x > 0, "must be an integer > 0")
        _need_derived(e, key, fails)
        if v is not None:
            want = _register_value(register, key, fails)
            if want is not None and v != want:
                fails.append(f"{key}: value {v!r} differs from the scenario-values register's {want!r}")
    if "m_sequence" in doc:
        e = scn._entry(doc, "m_sequence", "m_sequence", fails)
        _need_derived(e, "m_sequence", fails)
        if e is not None and e["value"] != M_SEQUENCE:
            fails.append(f"m_sequence: value {e['value']!r} must be {M_SEQUENCE} (decision 0005: m = 1 against m = 2)")
    state = doc.get("initial_state")
    if "initial_state" in doc:
        if not isinstance(state, dict):
            fails.append("initial_state: not a mapping")
        else:
            for k in state:
                if k not in scn.STATE_FIELDS:
                    fails.append(f"initial_state.{k}: unknown field")
            for k in scn.STATE_FIELDS:
                path = f"initial_state.{k}"
                if k not in state:
                    fails.append(f"{path}: missing field")
                    continue
                e = scn._entry(state, k, path, fails)
                v = scn._vector(e, path, scn.STATE_LENGTH[k], fails) if e is not None else None
                if v is None:
                    continue
                if k == "attitude_q_wxyz":
                    if v[0] < 0:
                        fails.append(f"{path}: w = {v[0]!r} < 0; canonical sign is w >= 0")
                    n2 = sum(x * x for x in v)
                    if abs(n2 - 1.0) > scn.QUAT_NORM_TOL:
                        fails.append(f"{path}: | |q|^2 - 1 | = {abs(n2 - 1.0)!r} exceeds {scn.QUAT_NORM_TOL!r}")
                if k == "body_rates_frd_rad_s" and any(x != 0 for x in v):
                    fails.append(f"{path}: must be zero; the run starts from rest")
    return scalar


def _section(doc, key, fields, fails):
    sec = doc.get(key)
    if key not in doc:
        return None
    if not isinstance(sec, dict):
        fails.append(f"{key}: not a mapping")
        return None
    for k in sec:
        if k not in fields:
            fails.append(f"{key}.{k}: unknown field")
    for k in fields:
        if k not in sec:
            fails.append(f"{key}.{k}: missing field")
    return sec


def validate_chirp(doc, name, register):
    """The findings for a parsed L4 chirp scenario (the section comment above); empty means valid."""
    fails = []
    if not isinstance(doc, dict):
        return [f"{name}: not a mapping"]
    scalar = _validate_common(doc, name, register, CHIRP_TOP_FIELDS, fails)
    sec = _section(doc, "chirp", CHIRP_FIELDS, fails)
    if sec is None:
        return fails
    scalar(sec, "axis", "chirp.axis", lambda v: v in AXES, f"must be one of {list(AXES)}")
    scalar(sec, "settle_s", "chirp.settle_s", _positive, "must be a finite number > 0")
    scalar(sec, "tail_s", "chirp.tail_s", _positive, "must be a finite number > 0")
    _, d0 = scalar(sec, "duration_start_s", "chirp.duration_start_s", _positive, "must be a finite number > 0")
    e, d = scalar(sec, "duration_s", "chirp.duration_s", _positive, "must be a finite number > 0")
    _need_derived(e, "chirp.duration_s", fails)
    if d0 is not None and d is not None:
        ratio = Fraction(d) / Fraction(d0)
        if ratio.denominator != 1 or ratio < 2 or ratio.numerator & (ratio.numerator - 1):
            fails.append(f"chirp.duration_s: {d!r} is not duration_start_s {d0!r} times 2^j with j >= 1")
    if d is not None:
        us = Fraction(d) * scn.MICROSECONDS_PER_SECOND
        if us.denominator != 1 or us >= I32_LIMIT_US:
            fails.append(f"chirp.duration_s: {d!r} s is not a whole number of microseconds below 2^31")
    return fails


def validate_acro(doc, name, register):
    """The findings for a parsed L4 acro scenario (the section comment above); empty means valid."""
    fails = []
    if not isinstance(doc, dict):
        return [f"{name}: not a mapping"]
    scalar = _validate_common(doc, name, register, ACRO_TOP_FIELDS, fails)
    sec = _section(doc, "acro", ACRO_FIELDS, fails)
    if sec is None:
        return fails
    segs = sec.get("segments")
    last = None
    if "segments" in sec:
        if not isinstance(segs, list) or not segs:
            fails.append("acro.segments: must be a non-empty list")
            segs = []
        for i, seg in enumerate(segs):
            path = f"acro.segments[{i}]"
            if not isinstance(seg, dict):
                fails.append(f"{path}: not a mapping")
                continue
            for k in seg:
                if k not in SEGMENT_FIELDS:
                    fails.append(f"{path}.{k}: unknown field")
            for k in SEGMENT_FIELDS:
                if k not in seg:
                    fails.append(f"{path}.{k}: missing field")
            _, t = scalar(seg, "start_s", f"{path}.start_s", _positive, "must be a finite number > 0")
            if t is not None:
                if last is not None and not t > last:
                    fails.append(f"{path}.start_s: {t!r} is not after the previous segment's {last!r}")
                last = t
            if "stick" in seg:
                e = scn._entry(seg, "stick", f"{path}.stick", fails)
                v = scn._vector(e, f"{path}.stick", len(AXES), fails) if e is not None else None
                if v is not None and any(abs(x) > 1 for x in v):
                    fails.append(f"{path}.stick: {v!r} has an entry outside [-1, 1]")
    _, end = scalar(sec, "end_s", "acro.end_s", _positive, "must be a finite number > 0")
    if end is not None and last is not None and not end > last:
        fails.append(f"acro.end_s: {end!r} is not after the last segment's start {last!r}")
    return fails


def _load_kind(path, register_path, check):
    try:
        doc = schema.load_yaml(path)
        register = schema.load_yaml(register_path)
    except Exception as e:  # OSError or yaml.YAMLError (duplicate keys included)
        raise scn.ScenarioError([f"{path}: {e}"]) from e
    fails = check(doc, path, register)
    if fails:
        raise scn.ScenarioError([f"{path}: {f}" for f in fails])
    return doc


def load_chirp(path, register_path=REGISTER):
    """Parse and validate an L4 chirp scenario; return its document. Raises scenario.ScenarioError."""
    return _load_kind(path, register_path, validate_chirp)


def load_acro(path, register_path=REGISTER):
    """Parse and validate an L4 acro scenario; return its document. Raises scenario.ScenarioError."""
    return _load_kind(path, register_path, validate_acro)


def _common_values(doc, top_fields, section):
    v = {k: doc[k]["value"] for k in top_fields if k not in ("scenario", "initial_state", section)}
    v["scenario"] = doc["scenario"]
    v["initial_state"] = {k: doc["initial_state"][k]["value"] for k in scn.STATE_FIELDS}
    return v


def chirp_values(doc):
    """The plain values of a valid chirp scenario (state and chirp flattened one level)."""
    v = _common_values(doc, CHIRP_TOP_FIELDS, "chirp")
    v["chirp"] = {k: doc["chirp"][k]["value"] for k in CHIRP_FIELDS}
    return v


def acro_values(doc):
    """The plain values of a valid acro scenario: segments as a list of {start_s, stick}, and end_s."""
    v = _common_values(doc, ACRO_TOP_FIELDS, "acro")
    v["acro"] = {"segments": [{k: s[k]["value"] for k in SEGMENT_FIELDS} for s in doc["acro"]["segments"]],
                 "end_s": doc["acro"]["end_s"]["value"]}
    return v
