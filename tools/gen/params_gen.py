#!/usr/bin/env python3
"""Parameter bootstrap generator (core contracts 4, gate G2).

Reads YAML parameter sources and writes, into --out-dir:

  param_ids.hpp         ParamId (dense enum), kParamCount, kParamSchemaHash, ParamTraits specialisations,
                        declarations of the generated tables
  param_defaults.cpp    the const ParamRecord defaults table and the names table
  params_manifest.json  name -> id, type, unit, plus the schema hash, for the harness

Source file: a YAML mapping, parameter name -> entry. One entry:

  fixture_mass_kg: {type: f32, value: 0.75, unit: kg, method: scenario, source: "where it comes from", sigma: 0.01}

  type      f32 | i32
  value     finite number (f32) or integer in int32 range (i32)
  unit      SI symbol; "1" if dimensionless
  method    published | measured | identified | datasheet | derived(<rule>) | design-budget | scenario
  source    non-empty text; for derived(<rule>) the record's source is "<rule>; <source>"
  sigma     1-sigma uncertainty in the same unit; finite, >= 0 (0 for an integer count or an exact definition)
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


def parse_entry(name: str, entry, origin: str) -> Param:
    """Validate one entry; raises Refusal(message naming the field)."""
    if not isinstance(name, str) or not IDENTIFIER.fullmatch(name) or "__" in name or name in CXX_KEYWORDS:
        raise Refusal(f"field 'name': {name!r} is not a valid enumerator (letter first, letters/digits/_ only, "
                      "no '__', not a C++ keyword)")
    if not isinstance(entry, dict):
        raise Refusal("entry must be a mapping with type, value, unit, method, source, sigma")
    for field in REQUIRED:
        if field not in entry:
            raise Refusal(f"field '{field}': missing")
    for field in entry:
        if field not in REQUIRED and field not in IGNORED:
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

    source = entry["source"]
    if not isinstance(source, str) or not source.strip():
        raise Refusal("field 'source': must be a non-empty string")

    sigma = entry["sigma"]
    if not _is_number(sigma):
        raise Refusal(f"field 'sigma': must be a number, got {sigma!r}")
    if not math.isfinite(sigma):
        raise Refusal(f"field 'sigma': {sigma!r} is not finite")
    if sigma < 0:
        raise Refusal(f"field 'sigma': {sigma!r} is negative")
    sigma_f32 = _finite_f32(sigma, "field 'sigma'")

    value = entry["value"]
    if ptype == "f32":
        stored = _finite_f32(value, "field 'value'")
    else:
        if isinstance(value, bool) or not isinstance(value, int):
            raise Refusal(f"field 'value': i32 needs an integer, got {value!r}")
        if not INT32_MIN <= value <= INT32_MAX:
            raise Refusal(f"field 'value': {value!r} is outside the int32 range")
        stored = value

    return Param(name, ptype, stored, unit, "derived(" + rule + ")" if m else method,
                 f"{rule}; {source}" if m else source, sigma_f32, origin)


def load_sources(sources: list[tuple[str, Path]]) -> list[Param]:
    params: list[Param] = []
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
            if name in where:
                errors.append(f"{label}: entry '{name}': field 'name': duplicate, already defined in {where[name]}")
                continue
            try:
                param = parse_entry(name, entry, ORIGINS[kind])
            except Refusal as e:
                errors.append(f"{label}: entry '{name}': {e}")
                continue
            where[name] = label
            params.append(param)
    if not errors and not params:
        errors.append("no parameters: the sources define nothing")
    if not errors and len(params) > MAX_PARAMS:
        errors.append(f"{len(params)} parameters exceed the uint16 id space ({MAX_PARAMS})")
    if errors:
        raise Refusal("\n".join(errors))
    return params


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
            f"    {{{value}, {float_literal(p.sigma)}, ParamOrigin::{p.origin}, ParamMethod::{method}, false,",
            f"     {c_string(p.unit)}, {c_string(p.source)}}},",
        ]
    lines += ["};", "", "const char* const kParamNames[kParamCount] = {"]
    lines += [f"    {c_string(p.name)}," for p in params]
    lines += ["};", "", "}  // namespace marv::generated", ""]
    return "\n".join(lines)


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
                    help="directory receiving param_ids.hpp, param_defaults.cpp, params_manifest.json")
    args = ap.parse_args(argv)
    if not args.sources:
        ap.error("at least one --card or --register source is required")

    try:
        params = load_sources(args.sources)
    except Refusal as e:
        print(f"params_gen: refused:\n{e}", file=sys.stderr)
        return 1

    digest = schema_hash(params)
    outputs = {
        "param_ids.hpp": render_header(params, digest),
        "param_defaults.cpp": render_defaults(params),
        "params_manifest.json": render_manifest(params, digest),
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for fname, text in outputs.items():
        (args.out_dir / fname).write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
