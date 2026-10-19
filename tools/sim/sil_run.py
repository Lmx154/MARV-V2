"""The definition of an interactive SIL run, shared by the run (tools/sim/run_quad_fc.py --interactive) and the ground-station
helper (tools/gs/runs.py, ground-segment unit GX-3): the card, the firmware parameter overrides and the GUI flag, with the checks
both make before anything starts. Pure Python (no gz, no build): the helper imports it.

An override is a firmware parameter value the world hands to the SIL through <sil_override> (core section 4); in the SIL it
replaces the compiled value, locked or not, and the record's source becomes "sil-override" (fw/sil/src/param_override.cpp).
A run with at least one override carries the label LABEL in its run definition and in the helper's run manifest, and a
configuration that has a record with that source is refused by the validator (tools/validate/validate_config.py, invariant O-1:
a SIL override never reaches a board).

The checks. A name must be a parameter of the image's parameter manifest (build/host-gz-quadfc/generated/marv_params_fc3v2/.../
params_manifest.json: names and types; the manifest declares no other range). A value must lie in the domain its type declares
(tools/gen/params_gen.py: a finite number that fits binary32 for f32, an integer in the int32 range for i32); the firmware's own
checks of a value's meaning (composition::validate at boot) stay the SIL's: a value they refuse makes the run exit, which the
helper reports. A name may appear once (the plugin refuses a duplicate).
"""

from __future__ import annotations

import json
import math
import re
import struct
from pathlib import Path

LABEL = "sil-override"  # fw/sil/src/param_override.cpp: the source text of an overridden record
GZ_BUILD = "host-gz-quadfc"  # the build tree whose plugin and parameter set the interactive run uses
MANIFEST_REL = Path("generated") / "marv_params_fc3v2" / "marv" / "params" / "params_manifest.json"
INT32_MIN, INT32_MAX = -(1 << 31), (1 << 31) - 1  # the i32 domain (params_gen.py _parse_value)
TYPES = ("f32", "i32")
INTEGER_RE = re.compile(r"^[+-]?[0-9]+$")
SCHEMA = 1


def manifest_path(build_dir) -> Path:
    return Path(build_dir) / MANIFEST_REL


def load_param_types(path) -> dict[str, str]:
    """name -> "f32" | "i32" from a params_manifest.json; ValueError when it cannot be read."""
    try:
        params = json.loads(Path(path).read_text(encoding="utf-8"))["params"]
        types = {name: entry["type"] for name, entry in params.items()}
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as e:
        raise ValueError(f"no parameter manifest at {path} ({e}): build {GZ_BUILD} first") from None
    if not types or any(t not in TYPES for t in types.values()):
        raise ValueError(f"the parameter manifest {path} has no parameters or a type other than {', '.join(TYPES)}")
    return types


def _number(name: str, value, ptype: str):
    """The value of an override as the number of its type: ValueError names the parameter and the rule it breaks."""
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError(f"override {name}: the value must be a number")
    if isinstance(value, str):
        text = value.strip()
        if ptype == "i32":
            if not INTEGER_RE.match(text):
                raise ValueError(f"override {name}: {value!r} is not an integer (the parameter is i32)")
            value = int(text)
        else:
            try:
                value = float(text)
            except ValueError:
                raise ValueError(f"override {name}: {value!r} is not a number") from None
    if ptype == "i32":
        if isinstance(value, float):
            if not math.isfinite(value) or not value.is_integer():
                raise ValueError(f"override {name}: {value!r} is not an integer (the parameter is i32)")
            value = int(value)
        if not INT32_MIN <= value <= INT32_MAX:
            raise ValueError(f"override {name}: {value} is outside the i32 range {INT32_MIN} .. {INT32_MAX}")
        return value
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"override {name}: {value!r} is not a finite number")
    try:
        struct.pack("<f", value)
    except OverflowError:
        raise ValueError(f"override {name}: {value!r} does not fit binary32 (f32)") from None
    return value


def check_overrides(items, types: dict[str, str]) -> list[dict]:
    """The validated overrides, [{"name", "type", "value"}] in the order given. `items` is a list of (name, value) pairs, a value
    a number or its text. ValueError for an unknown parameter, a repeated one or a value outside its type's domain."""
    out, seen = [], set()
    for name, value in items:
        if not isinstance(name, str) or name not in types:
            raise ValueError(f"override {name!r}: not a parameter of the image's parameter set")
        if name in seen:
            raise ValueError(f"override {name}: given twice")
        seen.add(name)
        out.append({"name": name, "type": types[name], "value": _number(name, value, types[name])})
    return out


def split_cli(text: str) -> tuple[str, str]:
    """`name=value` -> (name, value text); ValueError without the `=`."""
    name, eq, value = text.partition("=")
    if not eq or not name or not value:
        raise ValueError(f"override {text!r}: expected name=value")
    return name, value


def value_text(override: dict) -> str:
    """The text of a value in a world and on a command line: repr of the float (the shortest text that reads back the same
    binary64) or the decimal integer."""
    return str(override["value"]) if override["type"] == "i32" else repr(float(override["value"]))


def cli_arg(override: dict) -> str:
    return f"{override['name']}={value_text(override)}"


def run_definition(card: str, overrides: list[dict], gui: bool) -> dict:
    """The run definition (written beside the world as run_definition.json, and in the helper's manifest as options): the card, the
    overrides and the GUI flag; label LABEL iff there is an override."""
    return {"schema": SCHEMA, "card": card, "gui": bool(gui), "overrides": list(overrides),
            "label": LABEL if overrides else None}
