"""Scenario schema of the quad L11 T4 runs (scenarios/quad/L11/*.yaml; quad spec section 4 L11, docs/decisions/0020 section 5 "T4").

The L5 schema (tools/sim/l5_scenario.py) with the script of the vertical loop: every number sits in an entry {value, unit,
label, rationale | rule} and nowhere else (the L2 entry rules, tools/sim/scenario.py; core contracts section 2). The fields
the two layers share are validated by the L5 module itself on `l5_view(doc)`, the document with a placeholder L5 script, so
the two schemas cannot drift apart; what differs is the script and one initial_state rule:

  initial_state.rotor_speed_rad_s  required, the text "hover" with label derived (L5's rule, tools/sim/run_l5.py
                           hover_rotor_speeds): the T3 oracle starts at the hover trim (vertical_t3_oracle.py), so the run
                           starts there, at execution 0, and has no settle phase
  script
    segments               a list of 0..8 mappings {start_vertical_execution, z_ref_m}: an integer >= 1, strictly
                           increasing, counted from the run start (vertical execution 0, the oracle's seed execution n = 0,
                           where z_ref = 0); z_ref_m a finite number, m, NED (z down); before the first segment z_ref = 0
    end_vertical_execution integer > the last start: the run's last vertical execution
  and optionally
    thrust_disturbance     {thrust_n, start_vertical_execution >= 1}: thrust_n a finite number, N, or the text
                           "mass_conflict" with label derived (the card's mass minus its open conflict value, times
                           g(phi, h0): the oracle's thrust_offset_n, tools/sim/run_l11.py mass_conflict_thrust_n)
    torque_pulse           {axis (roll|pitch|yaw), torque_nm (finite), duration_s > 0, start_vertical_execution >= 1}

A vertical execution is every rate_loop_divisor * vert_loop_ratio ticks (tick 0 included); its stamp is floor(tick num / den)
us, the oracle's (vertical_t3_oracle.py script() and Cascade). The tick entries are checked against the scenario-values
register here; what follows from the live parameters of a build is checked by the runner.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "card"))
import l5_scenario as l5s  # noqa: E402
import scenario as scn  # noqa: E402
import schema  # noqa: E402

REGISTER = l5s.REGISTER
TOP_FIELDS = l5s.TOP_FIELDS
SCRIPT_FIELDS = ("segments", "end_vertical_execution")
OPTIONAL_SCRIPT = ("thrust_disturbance", "torque_pulse")
SEGMENT_FIELDS = ("start_vertical_execution", "z_ref_m")
THRUST_DISTURBANCE_FIELDS = ("thrust_n", "start_vertical_execution")
TORQUE_PULSE_FIELDS = ("axis", "torque_nm", "duration_s", "start_vertical_execution")
AXES = l5s.AXES
SEGMENT_CAPACITY = l5s.SEGMENT_CAPACITY  # the l11_vertical_scripted register's l11_seg1..8 entries
MASS_CONFLICT = "mass_conflict"
ROTOR_SPEED_HOVER = l5s.ROTOR_SPEED_HOVER
_PLACEHOLDER = {k: {"value": v, "unit": u, "label": "scenario", "rationale": "placeholder of l5_view"}
                for k, v, u in (("settle_s", 1.0, "s"), ("end_attitude_execution", 1, "1"))}


def l5_view(doc):
    """`doc` with the L5 schema's script replaced by a placeholder: the document that the L5 validator, values() and world-input
    writer (tools/sim/run_l5.py l2_scenario_doc), which read only the fields the two layers share, accept."""
    return {**doc, "script": {**_PLACEHOLDER, "segments": []}}


def validate(doc, name, register):
    """The findings for the parsed L11 scenario `doc` of file `name` against the parsed register; empty means valid."""
    if not isinstance(doc, dict):
        return [f"{name}: not a mapping"]
    fails = [f for f in l5s.validate(l5_view(doc), name, register) if not f.startswith("script")]
    bare = []
    scn._bare_numbers({"script": doc.get("script")}, "", bare)
    fails += [f"{p}: unlabelled number (every number sits in an entry with a label)" for p in bare]
    state = doc.get("initial_state")
    if isinstance(state, dict):
        if "rotor_speed_rad_s" not in state:
            fails.append("initial_state.rotor_speed_rad_s: missing field")
        elif isinstance(state["rotor_speed_rad_s"], dict) and state["rotor_speed_rad_s"].get("value") != ROTOR_SPEED_HOVER:
            fails.append(f"initial_state.rotor_speed_rad_s: value must be {ROTOR_SPEED_HOVER!r} (the oracle starts at the trim)")
    script = doc.get("script")
    if "script" not in doc:
        fails.append("script: missing field")
    elif not isinstance(script, dict):
        fails.append("script: not a mapping")
    else:
        _validate_script(script, fails)
    return fails


def _validate_script(script, fails):
    l5s._fields(script, "script", SCRIPT_FIELDS, fails, OPTIONAL_SCRIPT)
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
            l5s._fields(seg, path, SEGMENT_FIELDS, fails)
            _, a = l5s._scalar(seg, "start_vertical_execution", f"{path}.start_vertical_execution",
                               lambda v: l5s._whole(v, 1), "must be an integer >= 1", fails)
            if a is not None:
                if last is not None and not a > last:
                    fails.append(f"{path}.start_vertical_execution: {a!r} is not after the previous segment's {last!r}")
                last = a
            l5s._scalar(seg, "z_ref_m", f"{path}.z_ref_m", scn.is_num, "must be a finite number", fails)
    _, end = l5s._scalar(script, "end_vertical_execution", "script.end_vertical_execution", lambda v: l5s._whole(v, 1),
                         "must be an integer >= 1", fails)
    if end is not None and last is not None and not end > last:
        fails.append(f"script.end_vertical_execution: {end!r} is not after the last segment's start {last!r}")
    if "thrust_disturbance" in script:
        d = script["thrust_disturbance"]
        if not isinstance(d, dict):
            fails.append("script.thrust_disturbance: not a mapping")
        else:
            l5s._fields(d, "script.thrust_disturbance", THRUST_DISTURBANCE_FIELDS, fails)
            e, v = l5s._scalar(d, "thrust_n", "script.thrust_disturbance.thrust_n",
                               lambda x: x == MASS_CONFLICT or scn.is_num(x), f"must be a finite number or {MASS_CONFLICT!r}",
                               fails)
            if v == MASS_CONFLICT and e.get("label") != "derived":
                fails.append("script.thrust_disturbance.thrust_n: label must be derived (the value is read from its rule's source)")
            l5s._scalar(d, "start_vertical_execution", "script.thrust_disturbance.start_vertical_execution",
                        lambda x: l5s._whole(x, 1), "must be an integer >= 1", fails)
    if "torque_pulse" in script:
        d = script["torque_pulse"]
        if not isinstance(d, dict):
            fails.append("script.torque_pulse: not a mapping")
        else:
            l5s._fields(d, "script.torque_pulse", TORQUE_PULSE_FIELDS, fails)
            l5s._scalar(d, "axis", "script.torque_pulse.axis", lambda v: v in AXES, f"must be one of {list(AXES)}", fails)
            l5s._scalar(d, "torque_nm", "script.torque_pulse.torque_nm", scn.is_num, "must be a finite number", fails)
            l5s._scalar(d, "duration_s", "script.torque_pulse.duration_s", lambda v: scn.is_num(v) and v > 0,
                        "must be a finite number > 0", fails)
            l5s._scalar(d, "start_vertical_execution", "script.torque_pulse.start_vertical_execution",
                        lambda x: l5s._whole(x, 1), "must be an integer >= 1", fails)


def load(path, register_path=REGISTER):
    """Parse and validate an L11 scenario file; return its document. Raises scenario.ScenarioError with the findings."""
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
    """The plain values of a valid L11 scenario: field -> value (l5s.values' shape for the shared fields), the script
    flattened one level, segments as a list of {start_vertical_execution, z_ref_m}."""
    v = l5s.values(l5_view(doc))
    s = doc["script"]
    out = {"end_vertical_execution": s["end_vertical_execution"]["value"],
           "segments": [{k: seg[k]["value"] for k in SEGMENT_FIELDS} for seg in s["segments"]]}
    for key, fields in (("thrust_disturbance", THRUST_DISTURBANCE_FIELDS), ("torque_pulse", TORQUE_PULSE_FIELDS)):
        if key in s:
            out[key] = {k: s[key][k]["value"] for k in fields}
    v["script"] = out
    return v
