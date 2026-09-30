"""Scenario schema for the quad L2 scenarios (scenarios/quad/L02/*.yaml), docs/decisions/0003 items 5, 7, 9, 10.

A scenario is a YAML mapping. Every number in it sits in an entry {value, unit, label, rationale | rule} and nowhere
else; a bare number outside an entry, or an entry without a label, is refused (core contracts section 2).

  label: scenario   a labelled scenario value; `rationale` is required, one line.
  label: derived    `rule` is required, one line: how the value follows from other entries or a cited item.

Required fields (no others are allowed):

  scenario                 the file stem
  site_latitude_rad        geodetic latitude of the NED origin, rad, |value| <= pi/2
  site_height_m            height h0 of the NED origin above the WGS 84 ellipsoid, m
  seed                     integer >= 0, reserved at v0 (marv_plant v0 draws no random numbers)
  tick_period_num_us       integer > 0
  tick_period_den          integer > 0; the tick period is num_us / den microseconds, exactly
  m_sequence               distinct integers >= 1, the ticks per host step of the SIM-7 refinement runs (item 9)
  duration_ticks           integer >= 1, a multiple of every m in m_sequence; T = duration_ticks * tick period
  initial_state
    position_ned_m         3 numbers, NED, origin at the site
    attitude_q_wxyz        4 numbers, Hamilton, body FRD -> NED, w >= 0, | |q|^2 - 1 | <= 4 eps_float (the ABI bound)
    velocity_ned_m_s       3 numbers
    body_rates_frd_rad_s   3 numbers; nonzero only in rotation.yaml, then off the separatrix (see below)
  separatrix_margin_min    required iff body_rates_frd_rad_s is nonzero, else refused
  command                  exactly one of
    dshot                  4 integers, each 0 (stop) or kDshotThrottleMin..kDshotThrottleMax of constants.hpp
    hover                  a non-empty subset of [lo, hi]: DShot D_lo = floor(D*) and D_hi = D_lo + 1 (tools/sim/hover.py)

Separatrix check (torque-free rigid body, principal moments I_min < I_mid < I_max from the card's inertia_diag).
With omega0 in the principal frame, L^2 = sum (I_i w_i)^2 and 2E = sum I_i w_i^2. The separatrix is L^2 = 2E I_mid.
The distance to it, normalised to (0, 1]:
  L^2 > 2E I_mid:   mu = (L^2 - 2E I_mid) / (2E (I_max - I_mid))   (1 is a pure spin about the maximum axis)
  L^2 < 2E I_mid:   mu = (2E I_mid - L^2) / (2E (I_mid - I_min))   (1 is a pure spin about the minimum axis)
  L^2 = 2E I_mid:   mu = 0.
The scenario is refused when mu < separatrix_margin_min, or the card inertia has I_mid equal to I_min or I_max, or
the card is not given.
"""

from __future__ import annotations

import math
import sys
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import prim_constants  # noqa: E402
import schema  # noqa: E402

LABELS = ("scenario", "derived")
ENTRY_KEYS = ("value", "unit", "label", "rationale", "rule")
TOP_FIELDS = (
    "scenario", "site_latitude_rad", "site_height_m", "seed", "tick_period_num_us", "tick_period_den", "m_sequence",
    "duration_ticks", "initial_state", "command",
)
OPTIONAL_TOP = ("separatrix_margin_min",)
STATE_FIELDS = ("position_ned_m", "attitude_q_wxyz", "velocity_ned_m_s", "body_rates_frd_rad_s")
STATE_LENGTH = {"position_ned_m": 3, "attitude_q_wxyz": 4, "velocity_ned_m_s": 3, "body_rates_frd_rad_s": 3}
HOVER_MEMBERS = ("lo", "hi")
MOTORS = 4
# 4 eps_float, the bound on | |q|^2 - 1 | in sim/plant/include/marv_plant.h (q_wxyz); eps_float = 2^-23 is the IEEE 754
# binary32 machine epsilon.
QUAT_NORM_TOL = 4 * 2.0 ** -23
MICROSECONDS_PER_SECOND = 10 ** 6  # SI: micro = 10^-6


class ScenarioError(Exception):
    """Refusal; `lines` are the findings, one per line."""

    def __init__(self, lines):
        super().__init__("; ".join(lines))
        self.lines = list(lines)


def is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def is_num(v):
    return (isinstance(v, (int, float)) and not isinstance(v, bool)) and math.isfinite(v)


def _bare_numbers(node, path, out):
    """Every number not inside an entry's `value` is unlabelled."""
    if isinstance(node, dict):
        if "value" in node:
            for k, v in node.items():
                if k != "value":
                    _bare_numbers(v, f"{path}.{k}", out)
            return
        for k, v in node.items():
            _bare_numbers(v, f"{path}.{k}" if path else str(k), out)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            _bare_numbers(v, f"{path}[{i}]", out)
    elif isinstance(node, (int, float)):
        out.append(path)


def _entry(doc, key, path, fails):
    """The entry at doc[key] after the structural checks, or None."""
    e = doc.get(key)
    if not isinstance(e, dict):
        fails.append(f"{path}: missing or not an entry mapping")
        return None
    for k in e:
        if k not in ENTRY_KEYS:
            fails.append(f"{path}: unknown entry field {k!r}")
    for k in ("value", "unit", "label"):
        if k not in e:
            fails.append(f"{path}: missing field {k!r}")
    if "label" in e:
        if e["label"] not in LABELS:
            fails.append(f"{path}: label {e['label']!r} is not one of {LABELS}")
        else:
            need = "rationale" if e["label"] == "scenario" else "rule"
            text = e.get(need)
            if not isinstance(text, str) or not text.strip():
                fails.append(f"{path}: label {e['label']} needs a non-empty {need}")
            elif "\n" in text.strip():
                fails.append(f"{path}: {need} must be one line")
    if "unit" in e and (not isinstance(e["unit"], str) or not e["unit"]):
        fails.append(f"{path}: unit must be non-empty text")
    return e if "value" in e else None


def _vector(e, path, n, fails):
    v = e["value"]
    if not isinstance(v, list) or len(v) != n or not all(is_num(x) for x in v):
        fails.append(f"{path}: value must be a list of {n} finite numbers")
        return None
    return v


def legal_dshot(v, dmin, dmax):
    return is_int(v) and (v == 0 or dmin <= v <= dmax)


def separatrix_mu(inertia_diag, omega):
    """Normalised distance to the separatrix (docstring above); raises ScenarioError if it is undefined."""
    order = sorted(range(3), key=lambda i: inertia_diag[i])
    i_min, i_mid, i_max = (inertia_diag[i] for i in order)
    if not (i_min < i_mid < i_max):
        raise ScenarioError(["card inertia_diag: principal moments are not distinct; no separatrix"])
    l2 = sum((inertia_diag[i] * omega[i]) ** 2 for i in range(3))
    two_e = sum(inertia_diag[i] * omega[i] ** 2 for i in range(3))
    d = l2 - two_e * i_mid
    if d > 0:
        return d / (two_e * (i_max - i_mid))
    return -d / (two_e * (i_mid - i_min))


def validate(doc, name, inertia_diag=None):
    """Return the list of findings for the parsed scenario `doc` of file `name`; empty means valid."""
    fails = []
    if not isinstance(doc, dict):
        return [f"{name}: not a mapping"]
    consts = prim_constants.load()
    dmin, dmax = consts["kDshotThrottleMin"], consts["kDshotThrottleMax"]

    bare = []
    _bare_numbers({k: v for k, v in doc.items() if k != "scenario"}, "", bare)
    for p in bare:
        fails.append(f"{p}: unlabelled number (every number sits in an entry with a label)")

    allowed = set(TOP_FIELDS) | set(OPTIONAL_TOP)
    for k in doc:
        if k not in allowed:
            fails.append(f"{k}: unknown field")
    for k in TOP_FIELDS:
        if k not in doc:
            fails.append(f"{k}: missing field")
    if doc.get("scenario") != Path(name).stem:
        fails.append(f"scenario: {doc.get('scenario')!r} must equal the file stem {Path(name).stem!r}")

    def scalar(key, check, why):
        if key not in doc:
            return None
        e = _entry(doc, key, key, fails)
        if e is not None and not check(e["value"]):
            fails.append(f"{key}: value {e['value']!r} {why}")
        return e["value"] if e is not None else None

    scalar("site_latitude_rad", lambda v: is_num(v) and abs(v) <= math.pi / 2, "must be a number with |value| <= pi/2")
    scalar("site_height_m", is_num, "must be a finite number")
    scalar("seed", lambda v: is_int(v) and v >= 0, "must be an integer >= 0")
    scalar("tick_period_num_us", lambda v: is_int(v) and v > 0, "must be an integer > 0")
    scalar("tick_period_den", lambda v: is_int(v) and v > 0, "must be an integer > 0")
    ticks = scalar("duration_ticks", lambda v: is_int(v) and v >= 1, "must be an integer >= 1")

    ms = None
    if "m_sequence" in doc:
        e = _entry(doc, "m_sequence", "m_sequence", fails)
        if e is not None:
            v = e["value"]
            if (not isinstance(v, list) or not v or not all(is_int(x) and x >= 1 for x in v)
                    or len(set(v)) != len(v)):
                fails.append(f"m_sequence: value {v!r} must be a non-empty list of distinct integers >= 1")
            else:
                ms = v
    if ms is not None and is_int(ticks):
        for m in ms:
            if ticks % m:
                fails.append(f"duration_ticks: {ticks} is not a multiple of m = {m}")

    state = doc.get("initial_state")
    rates = None
    if "initial_state" in doc:
        if not isinstance(state, dict):
            fails.append("initial_state: not a mapping")
        else:
            for k in state:
                if k not in STATE_FIELDS:
                    fails.append(f"initial_state.{k}: unknown field")
            for k in STATE_FIELDS:
                if k not in state:
                    fails.append(f"initial_state.{k}: missing field")
                    continue
                e = _entry(state, k, f"initial_state.{k}", fails)
                if e is None:
                    continue
                v = _vector(e, f"initial_state.{k}", STATE_LENGTH[k], fails)
                if v is None:
                    continue
                if k == "attitude_q_wxyz":
                    if v[0] < 0:
                        fails.append(f"initial_state.{k}: w = {v[0]!r} < 0; canonical sign is w >= 0")
                    n2 = sum(x * x for x in v)
                    if abs(n2 - 1.0) > QUAT_NORM_TOL:
                        fails.append(f"initial_state.{k}: | |q|^2 - 1 | = {abs(n2 - 1.0)!r} exceeds {QUAT_NORM_TOL!r}")
                if k == "body_rates_frd_rad_s":
                    rates = v

    nonzero_rates = rates is not None and any(x != 0 for x in rates)
    if "separatrix_margin_min" in doc:
        e = _entry(doc, "separatrix_margin_min", "separatrix_margin_min", fails)
        if e is not None and not (is_num(e["value"]) and 0 < e["value"] <= 1):
            fails.append(f"separatrix_margin_min: value {e['value']!r} must be a number in (0, 1]")
        if rates is not None and not nonzero_rates:
            fails.append("separatrix_margin_min: given, but the body rates are zero")
    elif nonzero_rates:
        fails.append("separatrix_margin_min: missing; nonzero body rates need the separatrix check")
    if nonzero_rates and "separatrix_margin_min" in doc and isinstance(doc["separatrix_margin_min"], dict):
        margin = doc["separatrix_margin_min"].get("value")
        if inertia_diag is None:
            fails.append("initial_state.body_rates_frd_rad_s: nonzero, and no card given to check the separatrix")
        elif is_num(margin):
            try:
                mu = separatrix_mu(inertia_diag, rates)
            except ScenarioError as err:
                fails.extend(err.lines)
            else:
                if mu < margin:
                    fails.append(f"initial_state.body_rates_frd_rad_s: distance to the separatrix mu = {mu!r} is "
                                 f"below separatrix_margin_min = {margin!r}")

    cmd = doc.get("command")
    if "command" in doc:
        if not isinstance(cmd, dict):
            fails.append("command: not a mapping")
        else:
            kinds = [k for k in cmd if k in ("dshot", "hover")]
            for k in cmd:
                if k not in ("dshot", "hover"):
                    fails.append(f"command.{k}: unknown field")
            if len(kinds) != 1:
                fails.append("command: exactly one of dshot, hover is required")
            elif kinds[0] == "dshot":
                e = _entry(cmd, "dshot", "command.dshot", fails)
                if e is not None:
                    v = e["value"]
                    if not isinstance(v, list) or len(v) != MOTORS:
                        fails.append(f"command.dshot: value must be a list of {MOTORS} integers")
                    else:
                        for i, d in enumerate(v):
                            if not legal_dshot(d, dmin, dmax):
                                fails.append(f"command.dshot: motor {i + 1} value {d!r} is illegal; it must be an "
                                             f"integer, 0 or {dmin}..{dmax} (1..{dmin - 1} are special commands)")
            else:
                e = _entry(cmd, "hover", "command.hover", fails)
                if e is not None:
                    v = e["value"]
                    if (not isinstance(v, list) or not v or len(set(v)) != len(v)
                            or not all(x in HOVER_MEMBERS for x in v)):
                        fails.append(f"command.hover: value {v!r} must be a non-empty subset of {list(HOVER_MEMBERS)}")
    return fails


def load(path, card_path=None):
    """Parse and validate a scenario file; return its document (a dict). Raises ScenarioError with the findings.
    `card_path` is needed only to check nonzero body rates against the card's inertia."""
    try:
        doc = schema.load_yaml(path)
    except Exception as e:  # OSError or yaml.YAMLError (duplicate keys included)
        raise ScenarioError([f"{path}: {e}"]) from e
    inertia = None
    if card_path is not None:
        card = schema.load_yaml(card_path)
        inertia = [float(x) for x in card["inertia_diag"]["value"]]
    fails = validate(doc, path, inertia)
    if fails:
        raise ScenarioError([f"{path}: {f}" for f in fails])
    return doc


def values(doc):
    """The plain values of a valid scenario: dict of field -> value (state and command flattened one level)."""
    plain = [k for k in TOP_FIELDS + OPTIONAL_TOP if k not in ("scenario", "initial_state", "command") and k in doc]
    v = {k: doc[k]["value"] for k in plain}
    v["scenario"] = doc["scenario"]
    v["initial_state"] = {k: doc["initial_state"][k]["value"] for k in STATE_FIELDS}
    cmd = doc["command"]
    v["command"] = {k: cmd[k]["value"] for k in cmd}
    return v


def tick_period_s(doc):
    """The tick period in seconds as an exact Fraction: num_us / (den * 10^6)."""
    return Fraction(doc["tick_period_num_us"]["value"], doc["tick_period_den"]["value"] * MICROSECONDS_PER_SECOND)


def iterations(doc, m):
    """Host steps of a run at m ticks per host step: duration_ticks / m (an integer for every m in m_sequence)."""
    return doc["duration_ticks"]["value"] // m
