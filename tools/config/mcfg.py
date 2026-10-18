#!/usr/bin/env python3
"""The MARV binary config file, `.mcfg` (docs/decisions/0053 item 3; ground segment GD-3). This docstring is the single
source of the format.

The board stores its parameters as `.mcfg` (on the SD card and in the internal-flash A/B slots). The firmware never
parses text. The JSON that `dump` writes is the host's view of the same data. All integers are little-endian.

File = header (128 bytes) + payload. Payload = section directory + section bodies.

Header (offset: field)
    0   magic            8 bytes, "MARVMCFG"
    8   format_version   u16, 1. A reader refuses a version it does not know.
    10  header_size      u16, 128
    12  sequence         u32, the A/B slot sequence: of two valid slots the larger sequence is the newer
    16  section_count    u16, 1 to 3
    18  reserved         u16, 0
    20  payload_size     u32, the bytes after the header; the file is exactly header_size + payload_size bytes
    24  image_sha256     32 bytes, the binding to the firmware image (SHA-256 of the image); all zero = not bound
    56  report_id        32 bytes, the binding to the validation report (the report's SHA-256, or its id zero-padded
                         on the right); all zero = no report
    88  payload_sha256   32 bytes, SHA-256 of the payload. A reader recomputes it and refuses a mismatch.
    120 reserved         8 bytes, 0

Payload
    section directory, section_count entries of 24 bytes each:
        kind u8 (0 common, 1 quad, 2 rocket), 3 zero bytes, schema_hash u64, record_count u32, offset u32 (from
        the start of the payload), length u32 (body bytes, a multiple of 4)
    The kinds are strictly ascending, so a kind appears once. The bodies follow the directory in the same order with no
    gap: the first offset is section_count * 24, each next offset is the previous offset + length, and the last
    offset + length is payload_size. A file holds the sections its writer holds: an image reads the sections it knows
    and carries the others unchanged.

Section body = record_count records of 20 bytes, then the string table, then zero bytes to a multiple of 4.
    Records are in parameter-id order: record i is ParamId i, so a record carries no id and no name. The names, types
    and units come from the section's schema (schema_hash: the generated kParamSchemaHash of the parameter set the
    section was written from; tools/gen/params_gen.py), which an image checks against its own before reading.
    Record, 20 bytes:
        value_bits   u32  the IEEE-754 binary32 bits of an f32 value; the two's-complement bits of an i32 value
        sigma_bits   u32  the binary32 bits of the 1-sigma (+0.0 unless sigma_kind is Known)
        source_off   u32  byte offset of the record's source text in the string table
        type         u8   0 f32, 1 i32 (param_types.hpp ParamType)
        sigma_kind   u8   0 Known, 1 Exact, 2 Unknown, 3 Choice (SigmaKind)
        method       u8   0 published, 1 measured, 2 identified, 3 datasheet, 4 derived, 5 design-budget, 6 scenario
                          (ParamMethod)
        origin       u8   0 DefaultFromCard, 1 DefaultFromRegister, 2 Manual, 3 Identified (ParamOrigin)
        flags        u8   bit 0 locked (the parameter keeps its compiled value); other bits 0
        reserved     3 bytes, 0
    String table: UTF-8 texts, each terminated by one zero byte, each distinct text stored once, in the order of
    first use. A source_off points at the first byte of a text.

Config hash (core IF-1: the hash of the sections an image reads)
    SHA-256 over, for each section the image reads in ascending kind, the 20 bytes (kind u8, 3 zero bytes, schema_hash
    u64, record_count u32, length u32) followed by the section body. Values, sigma, provenance and sources are all in
    it, the header (sequence, binding, payload hash) and the directory offsets are not. A section the image does not
    read does not change it.

Canonical form: a writer produces exactly one byte string for given sections (the layout above, zero padding, strings
in first-use order), so encode(decode(f)) == f for a file a canonical writer produced, and a section it does not
rewrite is carried byte for byte.

Command line (uv run python tools/config/mcfg.py <command> --help)
    encode   build a file from a params_manifest.json (the factory file: the compiled defaults of one parameter set
             as one section, default kind quad), or from a JSON view; --carry-from copies the sections of another
             file that this one does not write, byte for byte; --max-bytes N refuses (exit 1, nothing written) a file
             longer than N bytes (the build passes half the config slot, design/budget.yaml config_slot_bytes)
    decode   check a file (every rule above) and write its lossless JSON view (bits, no names)
    dump     the JSON view with names, units and link ids from --names <params_manifest.json>; with
             --compare-table <table> compares every record, bit for bit, with the human-readable table
             tools/gen/params_gen.py --out-table writes, and exits 1 on any difference
    hash     the config hash over the sections named by --sections (default: all present)
    migrate  a schema change: records of the old parameter set carry over to the new one by name (same type and unit),
             the rest takes the new defaults, and the report lists what carried, what was added, dropped or
             changed. A parameter locked in the new manifest takes its compiled default. The result has sequence + 1
             and no binding, and the sections the new set does not write are carried byte for byte.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
import sys
from dataclasses import dataclass, replace
from pathlib import Path

MAGIC = b"MARVMCFG"
FORMAT_VERSION = 1
HEADER_SIZE = 128
HEADER = struct.Struct("<8sHHIHHI32s32s32s8x")
DIRECTORY_ENTRY = struct.Struct("<B3xQIII")
RECORD = struct.Struct("<IIIBBBBB3x")
HASH_ENTRY = struct.Struct("<B3xQII")
DIGEST = 32
SECTION_KINDS = ("common", "quad", "rocket")
TYPES = ("f32", "i32")
SIGMA_KINDS = ("Known", "Exact", "Unknown", "Choice")
METHODS = ("published", "measured", "identified", "datasheet", "derived", "design-budget", "scenario")
ORIGINS = ("DefaultFromCard", "DefaultFromRegister", "Manual", "Identified")
FLAG_LOCKED = 1
MAX_SECTIONS = len(SECTION_KINDS)


class McfgError(Exception):
    pass


@dataclass(frozen=True)
class Record:
    type: str
    value_bits: int
    sigma_bits: int
    sigma_kind: str
    method: str
    origin: str
    locked: bool
    source: str


@dataclass(frozen=True)
class Section:
    kind: str
    schema_hash: int
    record_count: int
    body: bytes


@dataclass(frozen=True)
class Config:
    sequence: int
    image_sha256: bytes
    report_id: bytes
    sections: tuple[Section, ...]

    def section(self, kind: str) -> Section | None:
        return next((s for s in self.sections if s.kind == kind), None)


def f32_of(bits: int) -> float:
    return struct.unpack("<f", struct.pack("<I", bits))[0]


def i32_of(bits: int) -> int:
    return bits - (1 << 32) if bits & 0x80000000 else bits


def sigma_ok(kind: str, bits: int) -> bool:
    if kind == "Known":
        f = f32_of(bits)
        return f > 0.0 and f <= 3.4028234663852886e38
    return bits == 0


def build_section(kind: str, schema_hash: int, records: list[Record]) -> Section:
    """The canonical section of the records (in id order)."""
    if kind not in SECTION_KINDS:
        raise McfgError(f"section kind {kind!r} is not one of {SECTION_KINDS}")
    strings = bytearray()
    offsets: dict[str, int] = {}
    out = bytearray()
    for i, r in enumerate(records):
        for field, allowed, value in (("type", TYPES, r.type), ("sigma_kind", SIGMA_KINDS, r.sigma_kind),
                                      ("method", METHODS, r.method), ("origin", ORIGINS, r.origin)):
            if value not in allowed:
                raise McfgError(f"record {i}: {field} {value!r} is not one of {allowed}")
        for field, bits in (("value_bits", r.value_bits), ("sigma_bits", r.sigma_bits)):
            if not 0 <= bits <= 0xFFFFFFFF:
                raise McfgError(f"record {i}: {field} {bits!r} is not 32 bits")
        if not sigma_ok(r.sigma_kind, r.sigma_bits):
            raise McfgError(f"record {i}: sigma bits 0x{r.sigma_bits:08x} do not match sigma kind {r.sigma_kind} "
                            "(Known needs a finite sigma > 0, every other kind +0.0)")
        if not r.source or "\0" in r.source:
            raise McfgError(f"record {i}: source must be non-empty text without a zero byte")
        if r.source not in offsets:
            offsets[r.source] = len(strings)
            strings += r.source.encode("utf-8") + b"\0"
        out += RECORD.pack(r.value_bits, r.sigma_bits, offsets[r.source], TYPES.index(r.type),
                           SIGMA_KINDS.index(r.sigma_kind), METHODS.index(r.method), ORIGINS.index(r.origin),
                           FLAG_LOCKED if r.locked else 0)
    out += strings
    out += b"\0" * (-len(out) % 4)
    return Section(kind, schema_hash, len(records), bytes(out))


def section_records(section: Section) -> list[Record]:
    """The records of a section; raises McfgError for any byte the format does not allow."""
    body = section.body
    table = section.record_count * RECORD.size
    if len(body) < table or len(body) % 4:
        raise McfgError(f"section {section.kind}: body of {len(body)} bytes cannot hold {section.record_count} records")
    strings = body[table:]
    if strings and strings[-1] != 0:
        raise McfgError(f"section {section.kind}: the string table does not end with a zero byte")
    records = []
    for i in range(section.record_count):
        v, s, off, t, k, m, o, flags = RECORD.unpack_from(body, i * RECORD.size)
        if body[i * RECORD.size + 17 : (i + 1) * RECORD.size] != b"\0\0\0":
            raise McfgError(f"section {section.kind} record {i}: reserved bytes are not zero")
        if t >= len(TYPES) or k >= len(SIGMA_KINDS) or m >= len(METHODS) or o >= len(ORIGINS):
            raise McfgError(f"section {section.kind} record {i}: an enumeration value is out of range "
                            f"(type {t}, sigma_kind {k}, method {m}, origin {o})")
        if flags & ~FLAG_LOCKED:
            raise McfgError(f"section {section.kind} record {i}: unknown flag bits 0x{flags:02x}")
        if not sigma_ok(SIGMA_KINDS[k], s):
            raise McfgError(f"section {section.kind} record {i}: sigma bits 0x{s:08x} do not match sigma kind "
                            f"{SIGMA_KINDS[k]}")
        if off >= len(strings):
            raise McfgError(f"section {section.kind} record {i}: source offset {off} is outside the string table")
        end = strings.index(b"\0", off)
        try:
            source = strings[off:end].decode("utf-8")
        except UnicodeDecodeError:
            raise McfgError(f"section {section.kind} record {i}: the source text is not UTF-8") from None
        if not source:
            raise McfgError(f"section {section.kind} record {i}: the source text is empty")
        records.append(Record(TYPES[t], v, s, SIGMA_KINDS[k], METHODS[m], ORIGINS[o], bool(flags & FLAG_LOCKED), source))
    return records


def encode(cfg: Config) -> bytes:
    if not 1 <= len(cfg.sections) <= MAX_SECTIONS:
        raise McfgError(f"a file holds 1 to {MAX_SECTIONS} sections, got {len(cfg.sections)}")
    kinds = [SECTION_KINDS.index(s.kind) for s in cfg.sections]
    if kinds != sorted(set(kinds)):
        raise McfgError("sections must be in ascending kind order, each kind once")
    if not 0 <= cfg.sequence <= 0xFFFFFFFF:
        raise McfgError(f"sequence {cfg.sequence!r} is not a u32")
    if len(cfg.image_sha256) != DIGEST or len(cfg.report_id) != DIGEST:
        raise McfgError("image_sha256 and report_id are 32 bytes each")
    directory = bytearray()
    bodies = bytearray()
    offset = len(cfg.sections) * DIRECTORY_ENTRY.size
    for s, kind in zip(cfg.sections, kinds, strict=True):
        if len(s.body) % 4:
            raise McfgError(f"section {s.kind}: body length {len(s.body)} is not a multiple of 4")
        directory += DIRECTORY_ENTRY.pack(kind, s.schema_hash, s.record_count, offset, len(s.body))
        bodies += s.body
        offset += len(s.body)
    payload = bytes(directory + bodies)
    header = HEADER.pack(MAGIC, FORMAT_VERSION, HEADER_SIZE, cfg.sequence, len(cfg.sections), 0, len(payload),
                         cfg.image_sha256, cfg.report_id, hashlib.sha256(payload).digest())
    return header + payload


def decode(data: bytes) -> Config:
    """Parse and verify a file: every rule of the module docstring; raises McfgError naming the first broken rule."""
    if len(data) < HEADER_SIZE:
        raise McfgError(f"file of {len(data)} bytes is shorter than the {HEADER_SIZE}-byte header")
    magic, version, hsize, sequence, count, reserved, psize, image, report, psha = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise McfgError(f"bad magic {magic!r}")
    if version != FORMAT_VERSION:
        raise McfgError(f"format version {version} is not known (this reader knows {FORMAT_VERSION})")
    if hsize != HEADER_SIZE or reserved != 0 or data[120:128] != b"\0" * 8:
        raise McfgError("header_size or a reserved field is not as the format says")
    if len(data) != HEADER_SIZE + psize:
        raise McfgError(f"file is {len(data)} bytes, the header says {HEADER_SIZE + psize}")
    payload = data[HEADER_SIZE:]
    if hashlib.sha256(payload).digest() != psha:
        raise McfgError("payload SHA-256 mismatch: the file is corrupt")
    if not 1 <= count <= MAX_SECTIONS or count * DIRECTORY_ENTRY.size > psize:
        raise McfgError(f"section_count {count} is not valid for a payload of {psize} bytes")
    sections = []
    expected = count * DIRECTORY_ENTRY.size
    last_kind = -1
    for i in range(count):
        kind, schema, records, offset, length = DIRECTORY_ENTRY.unpack_from(payload, i * DIRECTORY_ENTRY.size)
        if payload[i * DIRECTORY_ENTRY.size + 1 : i * DIRECTORY_ENTRY.size + 4] != b"\0\0\0":
            raise McfgError(f"directory entry {i}: reserved bytes are not zero")
        if kind >= len(SECTION_KINDS) or kind <= last_kind:
            raise McfgError(f"directory entry {i}: kind {kind} is invalid or not ascending")
        last_kind = kind
        if offset != expected or length % 4 or offset + length > psize:
            raise McfgError(f"directory entry {i}: offset {offset} length {length} are not contiguous")
        expected = offset + length
        sections.append(Section(SECTION_KINDS[kind], schema, records, bytes(payload[offset : offset + length])))
    if expected != psize:
        raise McfgError(f"{psize - expected} payload bytes follow the last section")
    cfg = Config(sequence, image, report, tuple(sections))
    for s in cfg.sections:
        section_records(s)
    return cfg


def config_hash(cfg: Config, kinds: tuple[str, ...] | None = None) -> bytes:
    """The config hash over the sections of the named kinds (default: every section present)."""
    h = hashlib.sha256()
    for s in cfg.sections:
        if kinds is None or s.kind in kinds:
            h.update(HASH_ENTRY.pack(SECTION_KINDS.index(s.kind), s.schema_hash, s.record_count, len(s.body)))
            h.update(s.body)
    return h.digest()


def _need(entry: dict, key: str, name: str):
    if key not in entry:
        raise McfgError(f"manifest entry {name}: field {key!r} is missing (regenerate it with tools/gen/params_gen.py)")
    return entry[key]


def load_manifest(path: Path) -> dict:
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise McfgError(f"{path}: cannot read the manifest: {e}") from None


def manifest_schema_hash(manifest: dict) -> int:
    return int(manifest["schema_hash"], 16)


def manifest_records(manifest: dict) -> list[Record]:
    """The compiled default of every parameter of a params_manifest.json, in id order."""
    records = []
    for i, (name, e) in enumerate(manifest["params"].items()):
        if _need(e, "id", name) != i:
            raise McfgError(f"manifest entry {name}: id {e['id']} is not its position {i}")
        records.append(Record(_need(e, "type", name), int(_need(e, "default_bits", name), 16),
                              int(_need(e, "sigma_bits", name), 16), _need(e, "sigma_kind", name),
                              _need(e, "method", name), _need(e, "origin", name), bool(_need(e, "lock", name)),
                              _need(e, "source", name)))
    if manifest["count"] != len(records):
        raise McfgError("manifest count does not match its params")
    return records


def factory_config(manifest: dict, kind: str = "quad", sequence: int = 0, image_sha256: bytes = bytes(DIGEST),
                   report_id: bytes = bytes(DIGEST)) -> Config:
    section = build_section(kind, manifest_schema_hash(manifest), manifest_records(manifest))
    return Config(sequence, image_sha256, report_id, (section,))


def carry_sections(cfg: Config, other: Config) -> Config:
    """cfg plus the sections of `other` whose kind cfg lacks, unchanged byte for byte."""
    have = {s.kind for s in cfg.sections}
    merged = list(cfg.sections) + [s for s in other.sections if s.kind not in have]
    merged.sort(key=lambda s: SECTION_KINDS.index(s.kind))
    return replace(cfg, sections=tuple(merged))


def migrate(old: Config, kind: str, old_manifest: dict, new_manifest: dict) -> tuple[Config, dict]:
    """Move the `kind` section from the old parameter set to the new one by name; returns the new config and a report."""
    section = old.section(kind)
    if section is None:
        raise McfgError(f"the file has no {kind} section")
    if section.schema_hash != manifest_schema_hash(old_manifest):
        raise McfgError(f"the {kind} section's schema hash 0x{section.schema_hash:016x} is not the old manifest's "
                        f"{old_manifest['schema_hash']}")
    old_records = section_records(section)
    old_names = list(old_manifest["params"])
    if len(old_names) != len(old_records):
        raise McfgError("the old manifest and the section have different record counts")
    old_by_name = {n: (old_manifest["params"][n], r) for n, r in zip(old_names, old_records, strict=True)}
    defaults = manifest_records(new_manifest)
    report: dict = {"carried": [], "added": [], "dropped": [], "changed": [], "locked_reset": []}
    out = []
    for (name, e), default in zip(new_manifest["params"].items(), defaults, strict=True):
        match = old_by_name.get(name)
        if match is None:
            report["added"].append(name)
            out.append(default)
        elif (match[0]["type"], match[0]["unit"]) != (e["type"], e["unit"]):
            report["changed"].append({"name": name, "old": [match[0]["type"], match[0]["unit"]],
                                      "new": [e["type"], e["unit"]]})
            out.append(default)
        elif default.locked and match[1] != default:
            report["locked_reset"].append(name)
            out.append(default)
        else:
            report["carried"].append(name)
            out.append(match[1])
    report["dropped"] = [n for n in old_names if n not in new_manifest["params"]]
    new_section = build_section(kind, manifest_schema_hash(new_manifest), out)
    cfg = Config((old.sequence + 1) & 0xFFFFFFFF, bytes(DIGEST), bytes(DIGEST), (new_section,))
    return carry_sections(cfg, old), report


def _record_json(i: int, r: Record, names: list | None) -> dict:
    value = f32_of(r.value_bits) if r.type == "f32" else i32_of(r.value_bits)
    doc = {"id": i}
    if names is not None:
        name, e = names[i]
        doc.update(name=name, unit=e["unit"], link_id=e["link_id"])
    doc.update(type=r.type, value=value, value_bits=f"0x{r.value_bits:08x}", sigma=f32_of(r.sigma_bits),
               sigma_bits=f"0x{r.sigma_bits:08x}", sigma_kind=r.sigma_kind, method=r.method, origin=r.origin,
               locked=r.locked, source=r.source)
    return doc


def view(cfg: Config, names: dict | None = None) -> dict:
    """The JSON view. `names` is {schema_hash: params_manifest} for the schemas whose names are known."""
    sections = []
    for s in cfg.sections:
        manifest = (names or {}).get(s.schema_hash)
        records = section_records(s)
        named = list(manifest["params"].items()) if manifest else None
        sections.append({
            "kind": s.kind,
            "schema_hash": f"0x{s.schema_hash:016x}",
            "named": manifest is not None,
            "record_count": s.record_count,
            "records": [_record_json(i, r, named) for i, r in enumerate(records)],
        })
    return {
        "format_version": FORMAT_VERSION,
        "sequence": cfg.sequence,
        "image_sha256": cfg.image_sha256.hex(),
        "report_id": cfg.report_id.hex(),
        "payload_sha256_ok": True,
        "config_hash": config_hash(cfg).hex(),
        "sections": sections,
    }


def config_from_view(doc: dict) -> Config:
    """The inverse of view(): bits, enumerations and sources are read; names and decoded values are ignored."""
    sections = []
    for s in doc["sections"]:
        records = [Record(r["type"], int(r["value_bits"], 16), int(r["sigma_bits"], 16), r["sigma_kind"], r["method"],
                          r["origin"], bool(r["locked"]), r["source"]) for r in s["records"]]
        sections.append(build_section(s["kind"], int(s["schema_hash"], 16), records))
    return Config(int(doc["sequence"]), bytes.fromhex(doc["image_sha256"]), bytes.fromhex(doc["report_id"]),
                  tuple(sections))


def parse_table(text: str) -> tuple[int, list[dict]]:
    """The rows of tools/gen/params_gen.py's --out-table file and its schema hash."""
    schema = None
    columns = None
    rows = []
    for line in text.splitlines():
        if line.startswith("# schema_hash "):
            schema = int(line.split()[2], 16)
        elif line.startswith("#") or not line:
            continue
        elif columns is None:
            columns = line.split("\t")
        else:
            rows.append(dict(zip(columns, line.split("\t"), strict=True)))
    if schema is None or columns is None:
        raise McfgError("the table has no schema_hash line or no column header")
    return schema, rows


def compare_table(cfg: Config, table_text: str) -> list[str]:
    """Differences between the config's section of the table's schema and the table, record by record, bit for bit."""
    schema, rows = parse_table(table_text)
    section = next((s for s in cfg.sections if s.schema_hash == schema), None)
    if section is None:
        return [f"no section has the table's schema hash 0x{schema:016x}"]
    records = section_records(section)
    if len(records) != len(rows):
        return [f"{len(records)} records in the file, {len(rows)} rows in the table"]
    diffs = []
    for i, (r, row) in enumerate(zip(records, rows, strict=True)):
        if row["type"] == "f32":
            value_bits = struct.unpack("<I", struct.pack("<f", float.fromhex(row["value_exact"])))[0]
            if struct.unpack("<I", struct.pack("<f", float(row["value"])))[0] != value_bits:
                diffs.append(f"record {i} {row['name']}: the table's decimal value {row['value']} and its "
                             f"hexadecimal value {row['value_exact']} are different floats")
        else:
            value_bits = int(row["value"]) & 0xFFFFFFFF
        want = Record(row["type"], value_bits, struct.unpack("<I", struct.pack("<f", float(row["sigma"])))[0],
                      row["sigma_kind"], row["method"],
                      row["origin"], row["locked"] == "true", json.loads(row["source"]))
        if want != r:
            diffs.append(f"record {i} {row['name']}: file {r} != table {want}")
    return diffs


def _digest_arg(text: str) -> bytes:
    try:
        b = bytes.fromhex(text)
    except ValueError:
        raise McfgError(f"{text!r} is not hexadecimal") from None
    if len(b) > DIGEST:
        raise McfgError(f"{text!r} is longer than {DIGEST} bytes")
    return b.ljust(DIGEST, b"\0")


def _read(path: Path) -> Config:
    try:
        return decode(Path(path).read_bytes())
    except OSError as e:
        raise McfgError(f"{path}: cannot read: {e.strerror}") from None


def _write(path: Path, data: bytes) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(data)


def _emit(doc, out: Path | None) -> None:
    text = json.dumps(doc, indent=2, ensure_ascii=False) + "\n"
    if out is None:
        sys.stdout.write(text)
    else:
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        Path(out).write_text(text, encoding="utf-8", newline="\n")


def _kinds(text: str | None) -> tuple[str, ...] | None:
    if text is None:
        return None
    kinds = tuple(k for k in text.split(",") if k)
    bad = [k for k in kinds if k not in SECTION_KINDS]
    if bad:
        raise McfgError(f"unknown section kind {bad[0]!r}, expected {SECTION_KINDS}")
    return kinds


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="MARV .mcfg tool (format: the module docstring)")
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("encode", help="build a .mcfg from a params_manifest.json or a JSON view")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--manifest", type=Path)
    src.add_argument("--json", type=Path, help="a JSON view written by decode or dump")
    p.add_argument("--section", choices=SECTION_KINDS, default="quad", help="kind of the section (with --manifest)")
    p.add_argument("--sequence", type=int, default=0)
    p.add_argument("--image-sha256", default="", help="hex; default unbound (zero)")
    p.add_argument("--report-id", default="", help="hex; default none (zero)")
    p.add_argument("--carry-from", type=Path, help="a .mcfg whose other sections are copied byte for byte")
    p.add_argument("--max-bytes", type=int, help="refuse a file longer than this (nothing is written)")
    p.add_argument("--out", type=Path, required=True)

    p = sub.add_parser("decode", help="verify a .mcfg and write its lossless JSON view")
    p.add_argument("file", type=Path)
    p.add_argument("--out", type=Path)

    p = sub.add_parser("dump", help="the JSON view with names; --compare-table checks it against a generated table")
    p.add_argument("file", type=Path)
    p.add_argument("--names", type=Path, action="append", default=[], help="params_manifest.json; repeatable")
    p.add_argument("--compare-table", type=Path, help="a params_gen.py --out-table file; exit 1 on any difference")
    p.add_argument("--out", type=Path)

    p = sub.add_parser("hash", help="the config hash over the named sections")
    p.add_argument("file", type=Path)
    p.add_argument("--sections", help="comma-separated kinds; default all present")

    p = sub.add_parser("migrate", help="carry a section across a schema change by parameter name")
    p.add_argument("file", type=Path)
    p.add_argument("--old-manifest", type=Path, required=True)
    p.add_argument("--manifest", type=Path, required=True, help="the new params_manifest.json")
    p.add_argument("--section", choices=SECTION_KINDS, default="quad")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--report", type=Path, help="write the migration report (JSON) here instead of stdout")

    args = ap.parse_args(argv)
    try:
        if args.command == "encode":
            if args.manifest is not None:
                cfg = factory_config(load_manifest(args.manifest), args.section, args.sequence,
                                     _digest_arg(args.image_sha256), _digest_arg(args.report_id))
            else:
                cfg = config_from_view(json.loads(args.json.read_text(encoding="utf-8")))
                cfg = replace(cfg, sequence=args.sequence if args.sequence else cfg.sequence)
            if args.carry_from is not None:
                cfg = carry_sections(cfg, _read(args.carry_from))
            data = encode(cfg)
            if args.max_bytes is not None and len(data) > args.max_bytes:
                raise McfgError(f"{args.out}: the config is {len(data)} bytes, more than --max-bytes {args.max_bytes} (half "
                                f"the config slot, design/budget.yaml config_slot_bytes): recompute that entry")
            _write(args.out, data)
        elif args.command == "decode":
            _emit(view(_read(args.file)), args.out)
        elif args.command == "dump":
            cfg = _read(args.file)
            names = {}
            for m in args.names:
                manifest = load_manifest(m)
                names[manifest_schema_hash(manifest)] = manifest
            if args.compare_table is not None:
                diffs = compare_table(cfg, args.compare_table.read_text(encoding="utf-8"))
                if diffs:
                    print("\n".join(diffs), file=sys.stderr)
                    return 1
                schema, rows = parse_table(args.compare_table.read_text(encoding="utf-8"))
                print(f"{args.file}: {len(rows)} records match {args.compare_table} bit for bit "
                      f"(schema hash 0x{schema:016x})")
            else:
                _emit(view(cfg, names), args.out)
        elif args.command == "hash":
            print(config_hash(_read(args.file), _kinds(args.sections)).hex())
        elif args.command == "migrate":
            cfg, report = migrate(_read(args.file), args.section, load_manifest(args.old_manifest),
                                  load_manifest(args.manifest))
            _write(args.out, encode(cfg))
            _emit(report, args.report)
    except (McfgError, OSError, KeyError, ValueError) as e:
        print(f"mcfg: {type(e).__name__}: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
