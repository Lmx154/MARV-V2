"""Entry schema and sigma policy for vehicle cards, sensor profiles and the design-budget register.

This module is the only implementation of the entry schema (core contracts 2.1, 2.2) and of the sigma policy. The
card, profile and budget linters in lint.py all call check_entry(); none restates a rule.

An entry is a YAML mapping. Canonical fields:

  value       number, list of numbers, category string (categorical entries only), or UNKNOWN
  unit        SI unit text; "1" if dimensionless. Required for every non-categorical entry
  method      published | measured | identified | datasheet | derived(<rule>) | design-budget | scenario
  source      non-empty text (cards and profiles; not budget entries, which carry a rationale)
  sigma       number, list of number | UNKNOWN, UNKNOWN, choice, or exact
  sigma_rule  required whenever any sigma component is numeric: "stated by <source>" or the derivation
  status      list, subset of {UNVERIFIED, INFERRED}
  conflict    list of {value, unit, source, state: open|refuted, refuted_by (required iff refuted)}
  check       text: what will settle or confirm the entry
  lock        {by, on: YYYY-MM-DD, via: manual}; only on published, measured, datasheet, identified, derived
  shape       frd3 | diag3 | range | motors | set; required when value is a list
  note        free text

  Shapes: frd3 and diag3 have 3 components; range has 2 with min <= max; motors has one per motor (4);
  set is a list keyed by a setting (for example a noise density at <= 8 / 16 / 32 g) with any length >= 1, the key
  stated in the note.

Sigma policy (the owner's decision of 2026-09-29):

  published, measured, identified, datasheet   sigma components are numbers > 0, or UNKNOWN. sigma = 0 is rejected.
                                               If a source gives no uncertainty, derive one by a stated rule
                                               (sigma_rule) or write UNKNOWN; never invent one.
  derived(<rule>)                              numbers >= 0 (0 means exact), or UNKNOWN.
  exact                                        an integer count. Allowed only when value is an int (not a float)
                                               and method is datasheet, published, measured or derived. Never on
                                               identified, design-budget or scenario entries, never on a float.
  design-budget, scenario                      sigma is `choice`: these are choices, not measurements. Numbers and
                                               UNKNOWN are rejected.
  value UNKNOWN                                sigma must be UNKNOWN (or `choice` for design-budget and scenario).
  An open conflict requires status UNVERIFIED.

Categorical entries (a value from an allowed set, for example spin) carry no unit, sigma, sigma_rule or shape;
doubt about one goes in status.

Model entries (a named model in place of a number, for example rotors.esc_map) carry `model` (from an allowed set)
instead of value, unit and shape, and follow the sigma policy of their method. What a model means is defined by the
consumer of the card; the card holds no numbers for it (linear_in_omega: DShot throttle range, from the firmware's
protocol constants, maps linearly onto rotors.speed_range, and 0 is stop).

Spin convention: `ccw` = counter-clockwise viewed from above (looking along +z_FRD, that is, down at the vehicle
from above); `cw` = clockwise viewed from above.

Budget entries (design/budget.yaml): value (number or UNKNOWN), unit, method: design-budget, sigma: choice,
rationale (non-empty, required even when value is UNKNOWN: state what will set it), used_by (non-empty list of
requirement names); optional status, check, note.
"""

from __future__ import annotations

import datetime
import math
import re

import yaml

UNKNOWN = "UNKNOWN"
CHOICE = "choice"
MEASURED_METHODS = ("published", "measured", "identified", "datasheet")
LOCKABLE_METHODS = MEASURED_METHODS + ("derived",)
CHOICE_METHODS = ("design-budget", "scenario")
ALL_METHODS = MEASURED_METHODS + ("derived(<rule>)",) + CHOICE_METHODS
STATUS_TAGS = ("UNVERIFIED", "INFERRED")
SHAPES = ("frd3", "diag3", "range", "motors", "set")
FIXED_SHAPE_LENGTH = {"frd3": 3, "diag3": 3}
MOTOR_COUNT = 4
DERIVED = re.compile(r"derived\((.*)\)", re.DOTALL)
LOCK_FIELDS = ("by", "on", "via")
CONFLICT_FIELDS = ("value", "unit", "source", "state", "refuted_by")

CARD_FIELDS = (
    "value", "unit", "method", "source", "sigma", "sigma_rule", "status", "conflict", "check", "lock", "shape", "note",
)
EXACT = "exact"
MODEL_FIELDS = (
    "model", "method", "source", "sigma", "sigma_rule", "status", "conflict", "check", "lock", "note",
)
CATEGORICAL_FIELDS = ("value", "method", "source", "status", "conflict", "check", "lock", "note")
BUDGET_FIELDS = (
    "value", "unit", "method", "sigma", "rationale", "used_by", "status", "check", "note",
)


class _Loader(yaml.SafeLoader):
    """Safe YAML loader that refuses duplicate keys, reads 1e-6 (no dot) as a float and knows only true/false as
    booleans (so the field name `on` in a lock stays a string)."""

    yaml_implicit_resolvers = {
        first: [(tag, rx) for tag, rx in resolvers if tag != "tag:yaml.org,2002:bool"]
        for first, resolvers in yaml.SafeLoader.yaml_implicit_resolvers.items()
    }

    def construct_mapping(self, node, deep=False):
        seen = set()
        for key_node, _ in node.value:
            key = self.construct_object(key_node, deep=True)
            if key in seen:
                raise yaml.constructor.ConstructorError(
                    None, None, f"duplicate key {key!r}", key_node.start_mark
                )
            seen.add(key)
        return super().construct_mapping(node, deep)


_Loader.add_implicit_resolver(
    "tag:yaml.org,2002:bool", re.compile(r"^(?:true|True|TRUE|false|False|FALSE)$"), list("tTfF")
)
_Loader.add_implicit_resolver(
    "tag:yaml.org,2002:float",
    re.compile(r"^[-+]?[0-9]+(?:\.[0-9]*)?[eE][-+]?[0-9]+$"),
    list("-+0123456789"),
)


def load_yaml(path):
    """Parse a YAML file with duplicate keys refused. Raises OSError or yaml.YAMLError."""
    with open(path, encoding="utf-8") as f:
        return yaml.load(f, Loader=_Loader)  # noqa: S506 (SafeLoader subclass)


class Findings:
    """Collected (file, entry path, reason) findings."""

    def __init__(self):
        self.items = []

    def add(self, file, path, reason):
        self.items.append((str(file), path, reason))

    def __bool__(self):
        return bool(self.items)

    def lines(self):
        return [f"{f}: {p}: {r}" for f, p, r in self.items]


def is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def is_text(v):
    return isinstance(v, str) and bool(v.strip())


def parse_method(method):
    """Return (kind, error). kind is one of MEASURED_METHODS, CHOICE_METHODS or "derived"."""
    if not isinstance(method, str):
        return None, "method: must be text"
    if method in MEASURED_METHODS or method in CHOICE_METHODS:
        return method, None
    m = DERIVED.fullmatch(method.strip())
    if m:
        if not m.group(1).strip():
            return None, "method: derived(<rule>) needs a non-empty rule"
        return "derived", None
    return None, f"method: {method!r} is not one of {', '.join(ALL_METHODS)}"


def _check_status(entry, path, out, file):
    status = entry.get("status")
    if status is None:
        return []
    if not isinstance(status, list) or not all(isinstance(s, str) for s in status):
        out.add(file, path, f"status: must be a list drawn from {', '.join(STATUS_TAGS)}")
        return []
    bad = [s for s in status if s not in STATUS_TAGS]
    if bad:
        out.add(file, path, f"status: {bad} not in {', '.join(STATUS_TAGS)}")
    return status


def _check_value(entry, path, out, file, categories, motors):
    """Validate value and shape. Returns (is_unknown, list_length_or_None)."""
    value = entry["value"]
    shape = entry.get("shape")
    if shape is not None and shape not in SHAPES:
        out.add(file, path, f"shape: {shape!r} is not one of {', '.join(SHAPES)}")
        shape = None
    if categories is not None:
        if value not in categories:
            out.add(file, path, f"value: {value!r} is not one of {', '.join(categories)}")
        return False, None
    if value == UNKNOWN:
        return True, None
    if isinstance(value, list):
        if not value or not all(is_number(v) for v in value):
            out.add(file, path, "value: a list must be non-empty and hold finite numbers only")
            return False, None
        if shape is None:
            out.add(file, path, f"shape: required when value is a list (one of {', '.join(SHAPES)})")
            return False, len(value)
        want = motors if shape == "motors" else 2 if shape == "range" else FIXED_SHAPE_LENGTH.get(shape)
        if want is not None and len(value) != want:
            out.add(file, path, f"shape: {shape} needs {want} components, value has {len(value)}")
        elif shape == "range" and value[0] > value[1]:
            out.add(file, path, f"shape: range needs min <= max, value is [{value[0]}, {value[1]}]")
        return False, len(value)
    if is_number(value):
        if shape is not None:
            out.add(file, path, "shape: only allowed when value is a list")
        return False, None
    out.add(file, path, f"value: must be a finite number, a list of numbers or {UNKNOWN}, got {value!r}")
    return False, None


def _is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def _check_sigma(entry, path, out, file, kind, value_unknown, list_len):
    if "sigma" not in entry:
        out.add(file, path, "sigma: missing (a number, UNKNOWN, choice or exact; see the sigma policy)")
        return
    sigma = entry["sigma"]
    if sigma == EXACT and kind in CHOICE_METHODS:
        out.add(file, path, f"sigma: a {kind} entry is a choice and has no uncertainty; write sigma: {CHOICE}, "
                            f"not {sigma!r}")
        return
    if sigma == EXACT:
        if kind == "identified":
            out.add(file, path, "sigma: exact is only for datasheet, published, measured or derived entries, "
                                "not identified")
        elif "value" not in entry or not _is_int(entry["value"]):
            out.add(file, path, f"sigma: exact is only for an integer count; value {entry.get('value')!r} is not "
                                "an int")
        if "sigma_rule" in entry:
            out.add(file, path, "sigma_rule: only allowed when a sigma component is numeric")
        return
    parts = sigma if isinstance(sigma, list) else [sigma]
    if isinstance(sigma, list) and not sigma:
        out.add(file, path, "sigma: an empty list")
        return
    numeric = []
    for p in parts:
        if p == UNKNOWN or p == CHOICE:
            continue
        if not is_number(p):
            out.add(file, path, f"sigma: {p!r} is not a finite number, UNKNOWN, choice or exact")
            return
        numeric.append(p)
    if isinstance(sigma, list) and any(p == CHOICE for p in parts):
        out.add(file, path, "sigma: choice is a whole-entry marker, not a list component")
        return
    if isinstance(sigma, list) and list_len is not None and len(sigma) != list_len:
        out.add(file, path, f"sigma: list of {len(sigma)} does not match the value's {list_len} components")
    if not isinstance(sigma, list) and list_len is not None and is_number(sigma):
        out.add(file, path, f"sigma: a vector value needs one sigma per component (a list of {list_len}), or "
                            f"{UNKNOWN}; a single number is ambiguous")
    if kind in CHOICE_METHODS:
        if sigma != CHOICE:
            out.add(file, path, f"sigma: a {kind} entry is a choice and has no uncertainty; write sigma: {CHOICE}, "
                                f"not {sigma!r}")
        return
    if sigma == CHOICE:
        out.add(file, path, f"sigma: choice is only for design-budget and scenario entries, not {kind}")
        return
    if value_unknown and numeric:
        out.add(file, path, f"sigma: value is {UNKNOWN}, so sigma must be {UNKNOWN}")
    for n in numeric:
        if n < 0:
            out.add(file, path, f"sigma: {n} is negative")
        elif n == 0 and kind in MEASURED_METHODS:
            out.add(file, path, f"sigma: 0 is rejected for method {kind}; derive a sigma by a stated rule "
                                f"(sigma_rule) or write sigma: {UNKNOWN}")
    rule = entry.get("sigma_rule")
    if numeric and not is_text(rule):
        out.add(file, path, 'sigma_rule: required when sigma is numeric ("stated by <source>" or the derivation)')
    if not numeric and rule is not None:
        out.add(file, path, "sigma_rule: only allowed when a sigma component is numeric")


def _check_conflict(entry, path, out, file, status, categorical):
    conflict = entry.get("conflict")
    if conflict is None:
        return
    if not isinstance(conflict, list) or not conflict:
        out.add(file, path, "conflict: must be a non-empty list of {value, unit, source, state}")
        return
    any_open = False
    for i, c in enumerate(conflict):
        cp = f"{path}.conflict[{i}]"
        if not isinstance(c, dict):
            out.add(file, cp, "must be a mapping {value, unit, source, state}")
            continue
        for k in c:
            if k not in CONFLICT_FIELDS:
                out.add(file, cp, f"unknown field {k!r}")
        need = ("value", "source", "state") if categorical else ("value", "unit", "source", "state")
        for k in need:
            if k not in c:
                out.add(file, cp, f"{k}: missing")
        if categorical and "unit" in c:
            out.add(file, cp, "unit: not allowed on a categorical entry's conflict")
        if "source" in c and not is_text(c["source"]):
            out.add(file, cp, "source: must be non-empty text")
        if "unit" in c and not is_text(c["unit"]):
            out.add(file, cp, "unit: must be non-empty text")
        state = c.get("state")
        if "state" in c and state not in ("open", "refuted"):
            out.add(file, cp, f"state: {state!r} is not open or refuted")
        if state == "open":
            any_open = True
        if state == "refuted" and not is_text(c.get("refuted_by")):
            out.add(file, cp, "refuted_by: required when state is refuted (the log that refuted it)")
        if state != "refuted" and "refuted_by" in c:
            out.add(file, cp, "refuted_by: only allowed when state is refuted")
    if any_open and "UNVERIFIED" not in status:
        out.add(file, path, "conflict: an open conflict requires status [UNVERIFIED]")


def _check_lock(entry, path, out, file, kind):
    lock = entry.get("lock")
    if lock is None:
        return
    if kind not in LOCKABLE_METHODS:
        out.add(file, path, f"lock: not allowed on a {kind} entry")
        return
    if not isinstance(lock, dict):
        out.add(file, path, "lock: must be {by, on: YYYY-MM-DD, via: manual}")
        return
    for k in lock:
        if k not in LOCK_FIELDS:
            out.add(file, path, f"lock: unknown field {k!r}")
    if not is_text(lock.get("by")):
        out.add(file, path, "lock.by: missing or empty")
    if lock.get("via") != "manual":
        out.add(file, path, f"lock.via: must be manual, got {lock.get('via')!r}")
    on = lock.get("on")
    if isinstance(on, datetime.datetime):
        on = None
    if isinstance(on, str) and re.fullmatch(r"\d{4}-\d{2}-\d{2}", on):
        try:
            on = datetime.date.fromisoformat(on)
        except ValueError:
            on = None
    if not isinstance(on, datetime.date):
        out.add(file, path, f"lock.on: must be a valid date YYYY-MM-DD, got {lock.get('on')!r}")


def check_entry(entry, path, out, file, *, categories=None, models=None, motors=MOTOR_COUNT, budget=False):
    """Validate one entry. Findings are added to `out`.

    categories  allowed value strings for a categorical entry (no unit, sigma, sigma_rule or shape)
    models      allowed model names for a model entry (`model` in place of value, unit and shape; sigma policy applies)
    budget      validate as a design-budget register entry
    """
    if not isinstance(entry, dict):
        out.add(file, path, "must be a mapping of entry fields")
        return
    categorical = categories is not None
    model_entry = models is not None
    allowed = (BUDGET_FIELDS if budget else CATEGORICAL_FIELDS if categorical
               else MODEL_FIELDS if model_entry else CARD_FIELDS)
    for k in entry:
        if k not in allowed:
            out.add(file, path, f"unknown field {k!r}")

    lead = "model" if model_entry else "value"
    required = [lead, "method"]
    if budget:
        required += ["unit", "sigma", "rationale", "used_by"]
    elif model_entry:
        required += ["source", "sigma"]
    elif categorical:
        required += ["source"]
    else:
        required += ["unit", "source"]
    for k in required:
        if k not in entry:
            out.add(file, path, f"{k}: missing")
    if lead not in entry or "method" not in entry:
        return

    kind, err = parse_method(entry["method"])
    if err:
        out.add(file, path, err)
    if budget and kind != "design-budget" and err is None:
        out.add(file, path, f"method: a register entry is design-budget, not {entry['method']!r}")
    if "unit" in entry and not is_text(entry["unit"]):
        out.add(file, path, 'unit: must be non-empty text ("1" if dimensionless)')
    if "source" in entry and not is_text(entry["source"]):
        out.add(file, path, "source: must be non-empty text")
    if "check" in entry and not is_text(entry["check"]):
        out.add(file, path, "check: must be non-empty text")
    if "note" in entry and not isinstance(entry["note"], str):
        out.add(file, path, "note: must be text")

    if model_entry:
        value_unknown, list_len = False, None
        if entry["model"] not in models:
            out.add(file, path, f"model: {entry['model']!r} is not one of {', '.join(models)}")
    else:
        value_unknown, list_len = _check_value(entry, path, out, file, categories, motors)
    status = _check_status(entry, path, out, file)
    if not categorical and kind is not None:
        _check_sigma(entry, path, out, file, kind, value_unknown, list_len)
    if budget:
        if not is_text(entry.get("rationale")):
            out.add(file, path, "rationale: missing or empty (state the rule or, when the value is UNKNOWN, what "
                                "will set it)")
        used_by = entry.get("used_by")
        if "used_by" in entry and not (
            isinstance(used_by, list) and used_by and all(is_text(u) for u in used_by)
        ):
            out.add(file, path, "used_by: must be a non-empty list of requirement names")
    else:
        _check_conflict(entry, path, out, file, status, categorical or model_entry)
        if kind is not None:
            _check_lock(entry, path, out, file, kind)
