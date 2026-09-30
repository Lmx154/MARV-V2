"""Scenario schema of the quad L5 T4 runs (scenarios/quad/L05/*.yaml; quad spec section 4 L5, docs/decisions/0006 F).

Follows the L4 schema (tools/sim/l4_scenario.py) and the frozen L2 entry rules (tools/sim/scenario.py): every number sits
in an entry {value, unit, label, rationale | rule} and nowhere else; a bare number outside an entry, or an entry without
a label, is refused (core contracts section 2). Entries are validated by the L2 helpers.

  label: scenario   a labelled scenario value; `rationale` is required, one line.
  label: derived    `rule` is required, one line: how the value follows from other entries or a cited item.

Required fields (no others are allowed):

  scenario                 the file stem
  site_latitude_rad        geodetic latitude of the NED origin, rad, |value| <= pi/2
  site_height_m            height of the NED origin above the WGS 84 ellipsoid, m
  seed                     integer >= 0, reserved at v0
  tick_period_num_us       integer > 0, label derived: equal to design/scenario_values.yaml tick_period_num_us
  tick_period_den          integer > 0, label derived: equal to design/scenario_values.yaml tick_period_den
  m_sequence               exactly [2, 1], label derived (decision 0005 "T4 envelope widening": E = |y(m=1) - y(m=2)|)
  initial_state            the four L2 state fields; attitude_q_wxyz is any canonical unit quaternion (w >= 0,
                           | |q|^2 - 1 | <= 4 eps_float), so a non-level start is allowed; body rates are any numbers
  initial_state            optionally also rotor_speed_rad_s (decision 0007): 4 numbers >= 0, logical motor order
                           (marv_plant_config.initial_omega_rad_s), or the text "hover" with label derived: the runner
                           resolves it to the card's hover rotor speed per motor, sqrt(T_i / k), T_i the firmware mixer
                           allocation M[i, thrust] m g(phi, h0) at zero torque (tools/sim/run_l5.py hover_rotor_speeds).
                           Absent = the rotors start at rest, as before.
  thrust                   value "hover", label derived: the collective request is the float32 of m g(phi, h0)
                           (tools/sim/run_l4.py hover_thrust)
  script
    settle_s               number > 0, s: the script's origin is the first attitude execution at or after settle_s that
                           keeps the stamp phase (tools/sim/run_l5.py plan)
    segments               a list of 0..8 mappings {start_attitude_execution, stick}: an integer >= 1, strictly
                           increasing, counted from the origin (execution 0, where the sticks are zero: the T3 oracle's
                           seed execution a = 0); stick 3 numbers in [-1, 1] (s_roll, s_pitch, s_yaw, shaped sticks)
    end_attitude_execution integer > the last start: the run's last execution, counted from the origin
  and optionally (each used by the scenario kinds that need it)
    chirp                  {axis (roll|pitch|yaw), amp_rad_s > 0, w_lo_rad_s > 0, w_hi_rad_s > w_lo, start_attitude_execution
                           >= 0, duration_s > 0}
    disturbance            {yaw_nm (finite), start_attitude_execution >= 0}

The tick entries are checked against the scenario-values register here; what follows from the live parameters of a build
(the attitude divisor, the tick against the build's) is checked by the runner (tools/sim/run_l5.py).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "card"))
import l4_scenario as l4s  # noqa: E402
import scenario as scn  # noqa: E402
import schema  # noqa: E402

REGISTER = l4s.REGISTER
TOP_FIELDS = ("scenario", "site_latitude_rad", "site_height_m", "seed", "tick_period_num_us", "tick_period_den",
              "m_sequence", "initial_state", "thrust", "script")
OPTIONAL_SCRIPT = ("chirp", "disturbance")
SCRIPT_FIELDS = ("settle_s", "segments", "end_attitude_execution")
SEGMENT_FIELDS = ("start_attitude_execution", "stick")
CHIRP_FIELDS = ("axis", "amp_rad_s", "w_lo_rad_s", "w_hi_rad_s", "start_attitude_execution", "duration_s")
DISTURBANCE_FIELDS = ("yaw_nm", "start_attitude_execution")
AXES = l4s.AXES
M_SEQUENCE = l4s.M_SEQUENCE
SEGMENT_CAPACITY = 8  # the l5_attitude_scripted register's l5_seg1..8 entries
THRUST_HOVER = "hover"
ROTOR_SPEED_HOVER = "hover"


def _scalar(parent, key, path, check, why, fails):
    if key not in parent:
        return None, None
    e = scn._entry(parent, key, path, fails)
    if e is None:
        return None, None
    if not check(e["value"]):
        fails.append(f"{path}: value {e['value']!r} {why}")
        return e, None
    return e, e["value"]


def _fields(parent, path, fields, fails, optional=()):
    for k in parent:
        if k not in fields and k not in optional:
            fails.append(f"{path}.{k}: unknown field")
    for k in fields:
        if k not in parent:
            fails.append(f"{path}.{k}: missing field")


def _whole(v, low):
    return scn.is_int(v) and v >= low


def validate(doc, name, register):
    """The findings for the parsed L5 scenario `doc` of file `name` against the parsed register; empty means valid."""
    fails = []
    if not isinstance(doc, dict):
        return [f"{name}: not a mapping"]
    bare = []
    scn._bare_numbers({k: v for k, v in doc.items() if k != "scenario"}, "", bare)
    fails += [f"{p}: unlabelled number (every number sits in an entry with a label)" for p in bare]
    for k in doc:
        if k not in TOP_FIELDS:
            fails.append(f"{k}: unknown field")
    for k in TOP_FIELDS:
        if k not in doc:
            fails.append(f"{k}: missing field")
    if doc.get("scenario") != Path(name).stem:
        fails.append(f"scenario: {doc.get('scenario')!r} must equal the file stem {Path(name).stem!r}")

    _scalar(doc, "site_latitude_rad", "site_latitude_rad", lambda v: scn.is_num(v) and abs(v) <= math.pi / 2,
            "must be a number with |value| <= pi/2", fails)
    _scalar(doc, "site_height_m", "site_height_m", scn.is_num, "must be a finite number", fails)
    _scalar(doc, "seed", "seed", lambda v: _whole(v, 0), "must be an integer >= 0", fails)
    for key in l4s.REGISTER_TICK:
        e, v = _scalar(doc, key, key, lambda x: _whole(x, 1), "must be an integer > 0", fails)
        l4s._need_derived(e, key, fails)
        if v is not None:
            want = l4s._register_value(register, key, fails)
            if want is not None and v != want:
                fails.append(f"{key}: value {v!r} differs from the scenario-values register's {want!r}")
    if "m_sequence" in doc:
        e = scn._entry(doc, "m_sequence", "m_sequence", fails)
        l4s._need_derived(e, "m_sequence", fails)
        if e is not None and e["value"] != M_SEQUENCE:
            fails.append(f"m_sequence: value {e['value']!r} must be {M_SEQUENCE} (decision 0005: m = 1 against m = 2)")
    if "thrust" in doc:
        e = scn._entry(doc, "thrust", "thrust", fails)
        l4s._need_derived(e, "thrust", fails)
        if e is not None and e["value"] != THRUST_HOVER:
            fails.append(f"thrust: value {e['value']!r} must be {THRUST_HOVER!r}")

    state = doc.get("initial_state")
    if "initial_state" in doc:
        if not isinstance(state, dict):
            fails.append("initial_state: not a mapping")
        else:
            _fields(state, "initial_state", scn.STATE_FIELDS, fails, scn.OPTIONAL_STATE)
            for k in scn.STATE_FIELDS:
                if k not in state:
                    continue
                path = f"initial_state.{k}"
                e = scn._entry(state, k, path, fails)
                v = scn._vector(e, path, scn.STATE_LENGTH[k], fails) if e is not None else None
                if v is not None and k == "attitude_q_wxyz":
                    if v[0] < 0:
                        fails.append(f"{path}: w = {v[0]!r} < 0; canonical sign is w >= 0")
                    n2 = sum(x * x for x in v)
                    if abs(n2 - 1.0) > scn.QUAT_NORM_TOL:
                        fails.append(f"{path}: | |q|^2 - 1 | = {abs(n2 - 1.0)!r} exceeds {scn.QUAT_NORM_TOL!r}")
            if "rotor_speed_rad_s" in state:
                path = "initial_state.rotor_speed_rad_s"
                e = scn._entry(state, "rotor_speed_rad_s", path, fails)
                if e is not None and e["value"] == ROTOR_SPEED_HOVER:
                    l4s._need_derived(e, "rotor_speed_rad_s", fails)
                elif e is not None:
                    v = scn._vector(e, path, scn.OPTIONAL_STATE_LENGTH["rotor_speed_rad_s"], fails)
                    if v is not None and any(x < 0 for x in v):
                        fails.append(f"{path}: {v!r} has a negative entry")

    script = doc.get("script")
    if "script" in doc:
        if not isinstance(script, dict):
            fails.append("script: not a mapping")
        else:
            _validate_script(script, fails)
    return fails


def _validate_script(script, fails):
    _fields(script, "script", SCRIPT_FIELDS, fails, OPTIONAL_SCRIPT)
    _scalar(script, "settle_s", "script.settle_s", l4s._positive, "must be a finite number > 0", fails)
    last = None
    if "segments" in script:
        segs = script["segments"]
        if not isinstance(segs, list) or len(segs) > SEGMENT_CAPACITY:
            fails.append(f"script.segments: must be a list of at most {SEGMENT_CAPACITY} segments")
            segs = []
        for i, seg in enumerate(segs):
            path = f"script.segments[{i}]"
            if not isinstance(seg, dict):
                fails.append(f"{path}: not a mapping")
                continue
            _fields(seg, path, SEGMENT_FIELDS, fails)
            _, a = _scalar(seg, "start_attitude_execution", f"{path}.start_attitude_execution",
                           lambda v: _whole(v, 1), "must be an integer >= 1", fails)
            if a is not None:
                if last is not None and not a > last:
                    fails.append(f"{path}.start_attitude_execution: {a!r} is not after the previous segment's {last!r}")
                last = a
            if "stick" in seg:
                e = scn._entry(seg, "stick", f"{path}.stick", fails)
                v = scn._vector(e, f"{path}.stick", len(AXES), fails) if e is not None else None
                if v is not None and any(abs(x) > 1 for x in v):
                    fails.append(f"{path}.stick: {v!r} has an entry outside [-1, 1]")
    _, end = _scalar(script, "end_attitude_execution", "script.end_attitude_execution", lambda v: _whole(v, 1),
                     "must be an integer >= 1", fails)
    if end is not None and last is not None and not end > last:
        fails.append(f"script.end_attitude_execution: {end!r} is not after the last segment's start {last!r}")
    if "chirp" in script:
        c = script["chirp"]
        if not isinstance(c, dict):
            fails.append("script.chirp: not a mapping")
        else:
            _fields(c, "script.chirp", CHIRP_FIELDS, fails)
            _scalar(c, "axis", "script.chirp.axis", lambda v: v in AXES, f"must be one of {list(AXES)}", fails)
            _scalar(c, "amp_rad_s", "script.chirp.amp_rad_s", l4s._positive, "must be a finite number > 0", fails)
            _, lo = _scalar(c, "w_lo_rad_s", "script.chirp.w_lo_rad_s", l4s._positive, "must be a finite number > 0", fails)
            _, hi = _scalar(c, "w_hi_rad_s", "script.chirp.w_hi_rad_s", l4s._positive, "must be a finite number > 0", fails)
            if lo is not None and hi is not None and not hi > lo:
                fails.append(f"script.chirp.w_hi_rad_s: {hi!r} is not above w_lo_rad_s {lo!r}")
            _scalar(c, "start_attitude_execution", "script.chirp.start_attitude_execution", lambda v: _whole(v, 0),
                    "must be an integer >= 0", fails)
            _scalar(c, "duration_s", "script.chirp.duration_s", l4s._positive, "must be a finite number > 0", fails)
    if "disturbance" in script:
        d = script["disturbance"]
        if not isinstance(d, dict):
            fails.append("script.disturbance: not a mapping")
        else:
            _fields(d, "script.disturbance", DISTURBANCE_FIELDS, fails)
            _scalar(d, "yaw_nm", "script.disturbance.yaw_nm", scn.is_num, "must be a finite number", fails)
            _scalar(d, "start_attitude_execution", "script.disturbance.start_attitude_execution",
                    lambda v: _whole(v, 0), "must be an integer >= 0", fails)


def load(path, register_path=REGISTER):
    """Parse and validate an L5 scenario file; return its document. Raises scenario.ScenarioError with the findings."""
    try:
        doc = schema.load_yaml(path)
        register = schema.load_yaml(register_path)
    except Exception as e:  # OSError or yaml.YAMLError (duplicate keys included)
        raise scn.ScenarioError([f"{path}: {e}"]) from e
    fails = validate(doc, path, register)
    if fails:
        raise scn.ScenarioError([f"{path}: {f}" for f in fails])
    return doc


def _plain(section, fields):
    return {k: section[k]["value"] for k in fields}


def values(doc):
    """The plain values of a valid L5 scenario: field -> value, the script flattened one level, segments as a list of
    {start_attitude_execution, stick}."""
    v = {k: doc[k]["value"] for k in TOP_FIELDS if k not in ("scenario", "initial_state", "script")}
    v["scenario"] = doc["scenario"]
    v["initial_state"] = {k: doc["initial_state"][k]["value"] for k in scn.STATE_FIELDS}
    for k in scn.OPTIONAL_STATE:
        if k in doc["initial_state"]:
            v["initial_state"][k] = doc["initial_state"][k]["value"]
    s = doc["script"]
    out = {"settle_s": s["settle_s"]["value"], "end_attitude_execution": s["end_attitude_execution"]["value"],
           "segments": [_plain(seg, SEGMENT_FIELDS) for seg in s["segments"]]}
    if "chirp" in s:
        out["chirp"] = _plain(s["chirp"], CHIRP_FIELDS)
    if "disturbance" in s:
        out["disturbance"] = _plain(s["disturbance"], DISTURBANCE_FIELDS)
    v["script"] = out
    return v
