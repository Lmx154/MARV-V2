#!/usr/bin/env python3
"""Parameter bootstrap generator (core contracts 4, gate G2).

Reads YAML parameter sources and writes, into --out-dir:

  param_ids.hpp         ParamId (dense enum), kParamCount, kParamSchemaHash, ParamTraits specialisations,
                        declarations of the generated tables
  param_defaults.cpp    the const ParamRecord defaults table and the names table
  params_manifest.json  name -> id, type, unit, plus the schema hash, for the harness
  params_provenance.json  per source entry: name, shape, component names, sigma kind of each component, lock

Source file: a YAML mapping, parameter name -> entry. One entry:

  fixture_mass_kg: {type: f32, value: 0.75, unit: kg, method: scenario, source: "where it comes from", sigma: 0.01}

  type      f32 | i32
  value     finite number (f32) or integer in int32 range (i32)
  unit      SI symbol; "1" if dimensionless
  method    published | measured | identified | datasheet | derived(<rule>) | design-budget | scenario
  source    non-empty text; for derived(<rule>) the record's source is "<rule>; <source>"
  sigma     1-sigma uncertainty in the same unit, or a token. The record's SigmaKind (fw/params param_types.hpp):
              number > 0   Known
              0            Exact (an integer count or a definition); refused for published, measured, identified,
                           datasheet, whose values carry an uncertainty
              exact        Exact, for an i32 entry (an integer count) only; refused for design-budget, scenario
              UNKNOWN      Unknown (not yet established; never read as 0); refused for design-budget, scenario
              choice       Choice (a design choice or scenario value); refused unless design-budget or scenario
            The emitted sigma is +0.0f for every kind but Known.
  lock      optional {by: <non-empty text>, on: <YYYY-MM-DD, a valid date>, via: manual}; all three are required
            and via must be manual. Emits locked = true (every component of a vector entry) and is recorded in
            params_provenance.json.
  shape     optional; makes the entry a vector of scalar records named <name><suffix>:
              frd3 -> _x _y _z    diag3 -> _xx _yy _zz    range -> _min _max    motors -> _m1 .. _mN (N = length)
            value is then a list of that length. sigma is a list of the same length (each a number or UNKNOWN) or
            the single token UNKNOWN / choice / exact for all components; a single number is refused. Every component
            shares type, unit, method and source. Expansion happens before the duplicate-name check and the schema
            hash, which therefore see the scalar records.
  conflict, status, check   accepted and ignored (core 2.1 card fields)

Origin is not an entry field: it is default-from-card for --card files and default-from-register for --register
files. Parameter ids follow the order the sources are given (--card / --register in command-line order, entries in
file order). The name is used verbatim as the ParamId enumerator, so it must be a plain C++ identifier.

Schema hash: SHA-256 over the ordered (name, type, unit) records, each encoded as UTF-8 "name\\x1ftype\\x1funit\\x1e";
the hash is the first 8 bytes of the digest read as a big-endian uint64. Values, sigma, method and source are not
part of it, so changing a value never changes the hash while renaming, retyping or re-uniting a parameter does.

Any entry that breaks a rule is refused: the message names the file, entry and field, nothing is written, and the
exit status is 1.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import struct
import sys
from dataclasses import dataclass
from datetime import date
from pathlib import Path

import yaml

MAX_PARAMS = 0xFFFF
INT32_MIN = -(2**31)
INT32_MAX = 2**31 - 1

TYPES = {"f32": ("F32", "float"), "i32": ("I32", "std::int32_t")}
METHODS = {
    "published": "Published",
    "measured": "Measured",
    "identified": "Identified",
    "datasheet": "Datasheet",
    "design-budget": "DesignBudget",
    "scenario": "Scenario",
}
DERIVED = re.compile(r"derived\((.*)\)", re.DOTALL)
ORIGINS = {"card": "DefaultFromCard", "register": "DefaultFromRegister"}
REQUIRED = ("type", "value", "unit", "method", "source", "sigma")
OPTIONAL = ("lock", "shape")
SIGMA_TOKENS = {"UNKNOWN": "Unknown", "choice": "Choice", "exact": "Exact"}
LIST_SIGMA_TOKENS = {"UNKNOWN": "Unknown"}
UNCERTAIN_METHODS = frozenset({"published", "measured", "identified", "datasheet"})
CHOICE_METHODS = frozenset({"design-budget", "scenario"})
FIXED_SHAPES = {"frd3": ("_x", "_y", "_z"), "diag3": ("_xx", "_yy", "_zz"), "range": ("_min", "_max")}
SHAPES = (*FIXED_SHAPES, "motors")
LOCK_FIELDS = ("by", "on", "via")
ISO_DATE = re.compile(r"[0-9]{4}-[0-9]{2}-[0-9]{2}")
IGNORED = ("conflict", "status", "check")
IDENTIFIER = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
CXX_KEYWORDS = frozenset(
    """alignas alignof and and_eq asm auto bitand bitor bool break case catch char char8_t char16_t char32_t class
    compl concept const consteval constexpr constinit const_cast continue co_await co_return co_yield decltype default
    delete do double dynamic_cast else enum explicit export extern false float for friend goto if inline int long
    mutable namespace new noexcept not not_eq nullptr operator or or_eq private protected public register
    reinterpret_cast requires return short signed sizeof static static_assert static_cast struct switch template this
    thread_local throw true try typedef typeid typename union unsigned using virtual void volatile wchar_t while xor
    xor_eq""".split()
)


class Refusal(Exception):
    pass


class _Loader(yaml.SafeLoader):
    """Safe YAML loader that refuses duplicate keys and reads 1e-6 (no dot) as a float."""

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




def _construct_timestamp(loader, node):
    """A date the calendar rejects (2026-13-45) stays text, so a lock refuses it by name."""
    try:
        return yaml.SafeLoader.construct_yaml_timestamp(loader, node)
    except ValueError:
        return loader.construct_scalar(node)


_Loader.add_constructor("tag:yaml.org,2002:timestamp", _construct_timestamp)
_Loader.add_implicit_resolver(
    "tag:yaml.org,2002:float",
    re.compile(r"^[-+]?[0-9]+(?:\.[0-9]*)?[eE][-+]?[0-9]+$"),
    list("-+0123456789"),
)


@dataclass(frozen=True)
class Param:
    name: str
    type: str
    value: float | int
    unit: str
    method: str
    source: str
    sigma: float
    origin: str
    sigma_kind: str = "Known"
    locked: bool = False


@dataclass(frozen=True)
class Entry:
    """One source entry as written, for params_provenance.json."""

    name: str
    source_file: str
    shape: str | None
    components: tuple[str, ...]
    sigma_kinds: tuple[str, ...]
    lock: dict | None


def to_f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def _is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _finite_f32(v, what: str) -> float:
    if not _is_number(v):
        raise Refusal(f"{what} must be a number, got {v!r} (write 1.0e-6 or 1e-6, not a string)")
    try:
        f = to_f32(float(v))
    except OverflowError:
        raise Refusal(f"{what} {v!r} does not fit a float") from None
    if not math.isfinite(f):
        raise Refusal(f"{what} {v!r} is not finite")
    if f == 0.0 and float(v) != 0.0:
        raise Refusal(f"{what} {v!r} underflows to zero as a float")
    return f


def _check_name(name) -> None:
    if not isinstance(name, str) or not IDENTIFIER.fullmatch(name) or "__" in name or name in CXX_KEYWORDS:
        raise Refusal(f"field 'name': {name!r} is not a valid enumerator (letter first, letters/digits/_ only, "
                      "no '__', not a C++ keyword)")


def _parse_value(ptype: str, value, what: str):
    if ptype == "f32":
        return _finite_f32(value, what)
    if isinstance(value, bool) or not isinstance(value, int):
        raise Refusal(f"{what}: i32 needs an integer, got {value!r}")
    if not INT32_MIN <= value <= INT32_MAX:
        raise Refusal(f"{what}: {value!r} is outside the int32 range")
    return value


def _check_kind(kind: str, method_key: str, what: str) -> None:
    if kind == "Exact" and method_key in UNCERTAIN_METHODS:
        raise Refusal(f"{what}: 0 (exact) is not allowed for method {method_key}; a {method_key} value has an "
                      "uncertainty, give it or write UNKNOWN")
    if kind == "Choice" and method_key not in CHOICE_METHODS:
        raise Refusal(f"{what}: choice is only for method design-budget or scenario, not {method_key}")
    if kind == "Unknown" and method_key in CHOICE_METHODS:
        raise Refusal(f"{what}: UNKNOWN is not allowed for method {method_key}; a design choice has no unknown "
                      "uncertainty (give a number, 0, or choice)")


def _parse_sigma(sigma, ptype: str, method_key: str, what: str, tokens: dict[str, str]) -> tuple[str, float]:
    if isinstance(sigma, str) and sigma in tokens:
        kind = tokens[sigma]
        if sigma == "exact":
            if ptype != "i32":
                raise Refusal(f"{what}: exact is only for an i32 entry (an integer count), not {ptype}; "
                              "write 0 for a definition")
            if method_key in CHOICE_METHODS:
                raise Refusal(f"{what}: exact is not allowed for method {method_key}; write 0 or choice")
        else:
            _check_kind(kind, method_key, what)
        return kind, 0.0
    if not _is_number(sigma):
        allowed = " or ".join(sorted(tokens))
        raise Refusal(f"{what}: must be a number or {allowed}, got {sigma!r}")
    if not math.isfinite(sigma):
        raise Refusal(f"{what}: {sigma!r} is not finite")
    if sigma < 0:
        raise Refusal(f"{what}: {sigma!r} is negative")
    if sigma == 0:
        _check_kind("Exact", method_key, what)
        return "Exact", 0.0
    return "Known", _finite_f32(sigma, what)


def _parse_lock(lock) -> dict:
    if not isinstance(lock, dict):
        raise Refusal(f"field 'lock': must be a mapping {{by, on, via}}, got {lock!r}")
    if True in lock:  # YAML 1.1 reads an unquoted `on` key as the boolean true
        if "on" in lock:
            raise Refusal("field 'lock': 'on' given twice")
        lock = {("on" if key is True else key): v for key, v in lock.items()}
    for key in lock:
        if key not in LOCK_FIELDS:
            raise Refusal(f"field 'lock': unknown key {key!r}, expected by, on, via")
    for key in LOCK_FIELDS:
        if key not in lock:
            raise Refusal(f"field 'lock': '{key}' is missing (by, on and via are all required)")
    by, on, via = lock["by"], lock["on"], lock["via"]
    if not isinstance(by, str) or not by.strip():
        raise Refusal(f"field 'lock': 'by' must be a non-empty string, got {by!r}")
    if type(on) is date:
        on_text = on.isoformat()
    elif isinstance(on, str) and ISO_DATE.fullmatch(on):
        try:
            on_text = date.fromisoformat(on).isoformat()
        except ValueError:
            raise Refusal(f"field 'lock': 'on' {on!r} is not a valid date") from None
    else:
        raise Refusal(f"field 'lock': 'on' must be a valid date written YYYY-MM-DD, got {on!r}")
    if via != "manual":
        raise Refusal(f"field 'lock': 'via' must be manual, got {via!r}")
    return {"by": by, "on": on_text, "via": via}


def _components(name: str, shape, value) -> tuple[tuple[str, ...], list]:
    """Return the component names of a vector entry and its value list."""
    if not isinstance(shape, str) or shape not in SHAPES:
        raise Refusal(f"field 'shape': {shape!r} is not one of {sorted(SHAPES)}")
    if not isinstance(value, list):
        raise Refusal(f"field 'value': shape {shape} needs a list, got {value!r}")
    if shape == "motors":
        if not value:
            raise Refusal("field 'value': shape motors needs at least one motor")
        suffixes = tuple(f"_m{i}" for i in range(1, len(value) + 1))
    else:
        suffixes = FIXED_SHAPES[shape]
        if len(value) != len(suffixes):
            raise Refusal(f"field 'value': shape {shape} needs {len(suffixes)} values ({', '.join(suffixes)}), "
                          f"got {len(value)}")
    return tuple(name + suffix for suffix in suffixes), value


def parse_entry(name: str, entry, origin: str) -> tuple[list[Param], Entry]:
    """Validate one entry and expand it to scalar records; raises Refusal(message naming the field)."""
    _check_name(name)
    if not isinstance(entry, dict):
        raise Refusal("entry must be a mapping with type, value, unit, method, source, sigma")
    for field in REQUIRED:
        if field not in entry:
            raise Refusal(f"field '{field}': missing")
    for field in entry:
        if field not in REQUIRED and field not in OPTIONAL and field not in IGNORED:
            raise Refusal(f"field '{field}': unknown field")

    ptype = entry["type"]
    if ptype not in TYPES:
        raise Refusal(f"field 'type': {ptype!r} is not one of {sorted(TYPES)}")

    unit = entry["unit"]
    if not isinstance(unit, str) or not unit.strip():
        raise Refusal(f"field 'unit': must be a non-empty string (\"1\" if dimensionless), got {unit!r}")

    method = entry["method"]
    if not isinstance(method, str):
        raise Refusal(f"field 'method': must be a string, got {method!r}")
    m = DERIVED.fullmatch(method.strip())
    if m:
        rule = m.group(1).strip()
        if not rule:
            raise Refusal("field 'method': derived(...) needs the rule that produces the number")
    elif method == "derived":
        raise Refusal("field 'method': derived needs its rule, write derived(<rule>)")
    elif method not in METHODS:
        raise Refusal(f"field 'method': {method!r} is not one of {sorted(METHODS)} or derived(<rule>)")
    method_key = "derived" if m else method

    source = entry["source"]
    if not isinstance(source, str) or not source.strip():
        raise Refusal("field 'source': must be a non-empty string")

    lock = _parse_lock(entry["lock"]) if "lock" in entry else None

    sigma = entry["sigma"]
    value = entry["value"]
    shape = entry["shape"] if "shape" in entry else None
    if "shape" in entry:
        names, values = _components(name, shape, value)
        for comp in names:
            _check_name(comp)
        if isinstance(sigma, list):
            if len(sigma) != len(names):
                raise Refusal(f"field 'sigma': needs {len(names)} entries to match the value list, got {len(sigma)}")
            sigmas = [_parse_sigma(x, ptype, method_key, f"field 'sigma'[{i}]", LIST_SIGMA_TOKENS)
                      for i, x in enumerate(sigma)]
        elif isinstance(sigma, str) and sigma in SIGMA_TOKENS:
            sigmas = [_parse_sigma(sigma, ptype, method_key, "field 'sigma'", SIGMA_TOKENS)] * len(names)
        else:
            raise Refusal("field 'sigma': a vector needs a list (a number or UNKNOWN per component) or the single "
                          f"token UNKNOWN, choice or exact for all, got {sigma!r}")
        stored = [_parse_value(ptype, v, f"field 'value'[{i}]") for i, v in enumerate(values)]
    else:
        names = (name,)
        sigmas = [_parse_sigma(sigma, ptype, method_key, "field 'sigma'", SIGMA_TOKENS)]
        stored = [_parse_value(ptype, value, "field 'value'")]

    full_method = "derived(" + rule + ")" if m else method
    full_source = f"{rule}; {source}" if m else source
    params = [Param(comp, ptype, v, unit, full_method, full_source, sig, origin, kind, lock is not None)
              for comp, v, (kind, sig) in zip(names, stored, sigmas, strict=True)]
    return params, Entry(name, "", shape, names, tuple(k for k, _ in sigmas), lock)


def load_sources(sources: list[tuple[str, Path]]) -> tuple[list[Param], list[Entry]]:
    params: list[Param] = []
    entries: list[Entry] = []
    where: dict[str, str] = {}
    errors: list[str] = []
    for kind, path in sources:
        label = path.name
        try:
            with open(path, encoding="utf-8") as f:
                doc = yaml.load(f, Loader=_Loader)  # noqa: S506 (SafeLoader subclass)
        except OSError as e:
            errors.append(f"{path}: cannot read: {e.strerror}")
            continue
        except yaml.YAMLError as e:
            errors.append(f"{path}: {e}")
            continue
        if not isinstance(doc, dict):
            errors.append(f"{path}: top level must be a mapping of parameter name -> entry")
            continue
        for name, entry in doc.items():
            try:
                expanded, described = parse_entry(name, entry, ORIGINS[kind])
            except Refusal as e:
                errors.append(f"{label}: entry '{name}': {e}")
                continue
            clashes = [p.name for p in expanded if p.name in where]
            if clashes:
                for c in clashes:
                    errors.append(f"{label}: entry '{name}': field 'name': '{c}' duplicate, already defined in "
                                  f"{where[c]}")
                continue
            for p in expanded:
                where[p.name] = label
            params += expanded
            entries.append(Entry(described.name, label, described.shape, described.components,
                                 described.sigma_kinds, described.lock))
    if not errors and not params:
        errors.append("no parameters: the sources define nothing")
    if not errors and len(params) > MAX_PARAMS:
        errors.append(f"{len(params)} parameters exceed the uint16 id space ({MAX_PARAMS})")
    if errors:
        raise Refusal("\n".join(errors))
    return params, entries


def schema_hash(params: list[Param]) -> int:
    h = hashlib.sha256()
    for p in params:
        h.update(f"{p.name}\x1f{p.type}\x1f{p.unit}\x1e".encode())
    return int.from_bytes(h.digest()[:8], "big")


def float_literal(x: float) -> str:
    text = f"{x:.9g}"
    if not any(c in text for c in ".e"):
        text += ".0"
    return text + "f"


def int_literal(v: int) -> str:
    return "-2147483647 - 1" if v == INT32_MIN else str(v)


def c_string(s: str) -> str:
    out = []
    for b in s.encode("utf-8"):
        c = chr(b)
        if 0x20 <= b < 0x7F and c not in '"\\?':
            out.append(c)
        else:
            out.append(f"\\{b:03o}")
    return '"' + "".join(out) + '"'


BANNER = "// Generated by tools/gen/params_gen.py from the parameter sources. Do not edit."


def render_header(params: list[Param], digest: int) -> str:
    lines = [
        BANNER,
        "// kParamSchemaHash: first 8 bytes (big-endian) of SHA-256 over the ordered (name, type, unit) records,",
        '// each encoded as UTF-8 "name\\x1ftype\\x1funit\\x1e" (type is f32 or i32). Values are excluded.',
        "#pragma once",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "#include <marv/params/param_types.hpp>",
        "",
        "namespace marv {",
        "",
        "enum class ParamId : std::uint16_t {",
    ]
    lines += [f"  {p.name} = {i}," for i, p in enumerate(params)]
    lines += [
        "};",
        "",
        f"inline constexpr std::size_t kParamCount = {len(params)};",
        f"inline constexpr std::uint64_t kParamSchemaHash = 0x{digest:016x}ULL;",
        "",
        "template <ParamId>",
        "struct ParamTraits;",
        "",
    ]
    for p in params:
        enum_name, cxx = TYPES[p.type]
        lines += [
            "template <>",
            f"struct ParamTraits<ParamId::{p.name}> {{",
            f"  using value_type = {cxx};",
            f"  static constexpr ParamType type = ParamType::{enum_name};",
            "};",
            "",
        ]
    lines += [
        "namespace generated {",
        "extern const ParamRecord kParamDefaults[kParamCount];",
        "extern const char* const kParamNames[kParamCount];",
        "}  // namespace generated",
        "",
        "}  // namespace marv",
        "",
    ]
    return "\n".join(lines)


def render_defaults(params: list[Param]) -> str:
    lines = [
        BANNER,
        "#include <marv/params/param_ids.hpp>",
        "",
        "namespace marv::generated {",
        "",
        "const ParamRecord kParamDefaults[kParamCount] = {",
    ]
    for p in params:
        enum_name = TYPES[p.type][0]
        if p.type == "f32":
            value = f"{{ParamType::{enum_name}, {float_literal(p.value)}, 0}}"
        else:
            value = f"{{ParamType::{enum_name}, 0.0f, {int_literal(p.value)}}}"
        method = "Derived" if p.method.startswith("derived(") else METHODS[p.method]
        lines += [
            f"    // {p.name}",
            f"    {{{value}, {float_literal(p.sigma)}, ParamOrigin::{p.origin}, ParamMethod::{method}, {'true' if p.locked else 'false'},",
            f"     SigmaKind::{p.sigma_kind},",
            f"     {c_string(p.unit)}, {c_string(p.source)}}},",
        ]
    lines += ["};", "", "const char* const kParamNames[kParamCount] = {"]
    lines += [f"    {c_string(p.name)}," for p in params]
    lines += ["};", "", "}  // namespace marv::generated", ""]
    return "\n".join(lines)


def render_provenance(entries: list[Entry], digest: int) -> str:
    doc = {
        "schema_hash": f"0x{digest:016x}",
        "entries": [
            {
                "name": e.name,
                "source_file": e.source_file,
                "shape": e.shape,
                "components": list(e.components),
                "sigma_kind": list(e.sigma_kinds),
                "lock": e.lock,
            }
            for e in entries
        ],
    }
    return json.dumps(doc, indent=2, ensure_ascii=False) + "\n"


def render_manifest(params: list[Param], digest: int) -> str:
    doc = {
        "schema_hash": f"0x{digest:016x}",
        "schema_hash_algorithm": "first 8 bytes big-endian of SHA-256 over ordered (name, type, unit)",
        "count": len(params),
        "params": {p.name: {"id": i, "type": p.type, "unit": p.unit} for i, p in enumerate(params)},
    }
    return json.dumps(doc, indent=2, ensure_ascii=False) + "\n"


class _SourceAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        getattr(namespace, "sources").append((self.const, Path(values)))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.set_defaults(sources=[])
    ap.add_argument("--card", action=_SourceAction, const="card", metavar="YAML",
                    help="vehicle-card source (origin default-from-card); repeatable")
    ap.add_argument("--register", action=_SourceAction, const="register", metavar="YAML",
                    help="design-budget register source (origin default-from-register); repeatable")
    ap.add_argument("--out-dir", required=True, type=Path,
                    help="directory receiving param_ids.hpp, param_defaults.cpp, params_manifest.json, "
                         "params_provenance.json")
    args = ap.parse_args(argv)
    if not args.sources:
        ap.error("at least one --card or --register source is required")

    try:
        params, entries = load_sources(args.sources)
    except Refusal as e:
        print(f"params_gen: refused:\n{e}", file=sys.stderr)
        return 1

    digest = schema_hash(params)
    outputs = {
        "param_ids.hpp": render_header(params, digest),
        "param_defaults.cpp": render_defaults(params),
        "params_manifest.json": render_manifest(params, digest),
        "params_provenance.json": render_provenance(entries, digest),
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for fname, text in outputs.items():
        (args.out_dir / fname).write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
