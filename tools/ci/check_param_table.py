#!/usr/bin/env python3
"""Post-link check: a linked image (the m33 ELF or a SIL library) holds exactly one parameter table, the intended set's.

Every parameter set defines the same symbols (marv::generated::kParamDefaults, kParamNames; fw/params/CMakeLists.txt), so
the linker takes whichever set's table the archive order offers first. This check reads the linked ELF itself and fails
unless:
  - exactly one defined kParamDefaults and one kParamNames symbol (of any set) are in it;
  - their sizes are the set's record count times sizeof(ParamRecord) and times a pointer;
  - every record's name (kParamNames), type, unit and value bits equal the set's params_manifest.json entry of that id;
  - the schema hash recomputed from the linked names, types and units equals the manifest's schema_hash.

ParamRecord's layout (fw/params/include/marv/params/param_types.hpp) is derived below from its member list by the natural
alignment rule of both ABIs in use (AAPCS for the m33, System V x86-64 for the host); the pointer size is the ELF class's.
Pointers in a position-independent library are read through its R_X86_64_RELATIVE relocations.

Usage: check_param_table.py --elf <file> --manifest <params_manifest.json> [--delete-on-failure]
Exit status: 0 the table is the set's; 1 it is not (lines tagged PARAM-TABLE; with --delete-on-failure the ELF is removed so
no image that failed is left to flash); 2 usage or a file that cannot be read.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import sys
from pathlib import Path

# ELF constants (System V gABI, "ELF Header" and "Sections"; x86-64 psABI, "Relocation Types").
ELF_MAGIC = b"\x7fELF"
EI_CLASS = 4
EI_DATA = 5
ELFCLASS32 = 1
ELFCLASS64 = 2
ELFDATA2LSB = 1
SHT_SYMTAB = 2
SHT_RELA = 4
SHT_NOBITS = 8
SHT_REL = 9
SHT_RELR = 19
SHF_ALLOC = 2
SHN_UNDEF = 0
R_X86_64_RELATIVE = 8
RELA_TYPE_MASK = 0xFFFFFFFF

LAYOUT = {
    ELFCLASS32: {"ehdr": "<16sHHIIIIIHHHHHH", "shdr": "<IIIIIIIIII", "sym": "<IIIBBH", "ptr": "<I", "rela": "<IIi"},
    ELFCLASS64: {"ehdr": "<16sHHIQQQIHHHHHH", "shdr": "<IIQQQQIIQQ", "sym": "<IBBHQQ", "ptr": "<Q", "rela": "<QQq"},
}

# The table symbols of any set: marv::generated::[<namespace>::]kParamDefaults and kParamNames (mangled).
DEFAULTS_RE = re.compile(r"^_ZN4marv9generated(?:\d+\w+)?14kParamDefaultsE$")
NAMES_RE = re.compile(r"^_ZN4marv9generated(?:\d+\w+)?11kParamNamesE$")

# ParamRecord as declared: ParamValue {ParamType (uint8), float, int32}, float sigma, four one-byte enumerations or bool,
# then the unit and source pointers. (name, size, alignment); "ptr" is the ELF class's pointer.
RECORD_MEMBERS = (
    ("type", 1, 1),
    ("f32", 4, 4),
    ("i32", 4, 4),
    ("sigma", 4, 4),
    ("origin", 1, 1),
    ("method", 1, 1),
    ("locked", 1, 1),
    ("sigma_kind", 1, 1),
    ("unit", "ptr", "ptr"),
    ("source", "ptr", "ptr"),
)
PARAM_TYPES = ("f32", "i32")  # ParamType::F32 = 0, I32


class TableError(Exception):
    pass


def record_layout(ptr_size: int) -> tuple[dict[str, int], int]:
    """Member offsets and sizeof(ParamRecord) by the natural alignment rule."""
    offsets = {}
    at = 0
    widest = 1
    for name, size, align in RECORD_MEMBERS:
        size = ptr_size if size == "ptr" else size
        align = ptr_size if align == "ptr" else align
        at = -(-at // align) * align
        offsets[name] = at
        at += size
        widest = max(widest, align)
    return offsets, -(-at // widest) * widest


class Elf:
    def __init__(self, path: Path):
        self.data = path.read_bytes()
        if self.data[: len(ELF_MAGIC)] != ELF_MAGIC or self.data[EI_DATA] != ELFDATA2LSB:
            raise TableError(f"{path}: not a little-endian ELF file")
        self.cls = self.data[EI_CLASS]
        if self.cls not in LAYOUT:
            raise TableError(f"{path}: unknown ELF class {self.cls}")
        lay = LAYOUT[self.cls]
        ehdr = struct.unpack_from(lay["ehdr"], self.data, 0)
        shoff, shentsize, shnum = ehdr[6], ehdr[11], ehdr[12]
        self.sections = [struct.unpack_from(lay["shdr"], self.data, shoff + i * shentsize) for i in range(shnum)]
        self.ptr_size = struct.calcsize(lay["ptr"])
        self.symbols = self._symbols(lay)
        self.relative = self._relative(lay)

    # Section header fields: name, type, flags, addr, offset, size, link, info, addralign, entsize.
    def _symbols(self, lay: dict) -> list[tuple[str, int, int, int]]:
        out = []
        entsize = struct.calcsize(lay["sym"])
        for sh in self.sections:
            if sh[1] != SHT_SYMTAB:
                continue
            strtab = self.sections[sh[6]]
            for at in range(sh[4], sh[4] + sh[5], entsize):
                fields = struct.unpack_from(lay["sym"], self.data, at)
                if self.cls == ELFCLASS32:
                    name, value, size, shndx = fields[0], fields[1], fields[2], fields[5]
                else:
                    name, shndx, value, size = fields[0], fields[3], fields[4], fields[5]
                end = self.data.index(b"\0", strtab[4] + name)
                out.append((self.data[strtab[4] + name : end].decode("ascii", "replace"), value, size, shndx))
        return out

    def _relative(self, lay: dict) -> dict[int, int]:
        rel = {}
        entsize = struct.calcsize(lay["rela"])
        for sh in self.sections:
            if not sh[2] & SHF_ALLOC or sh[1] not in (SHT_REL, SHT_RELA, SHT_RELR):
                continue
            if sh[1] != SHT_RELA or self.cls != ELFCLASS64:
                raise TableError("the image has dynamic relocations other than ELF64 RELA, which this check does not read")
            for at in range(sh[4], sh[4] + sh[5], entsize):
                offset, info, addend = struct.unpack_from(lay["rela"], self.data, at)
                rel[offset] = addend if info & RELA_TYPE_MASK == R_X86_64_RELATIVE else None
        return rel

    def defined(self, pattern: re.Pattern) -> list[tuple[str, int, int]]:
        return [(n, v, s) for n, v, s, shndx in self.symbols if shndx != SHN_UNDEF and pattern.match(n)]

    def read(self, addr: int, size: int) -> bytes:
        for sh in self.sections:
            if sh[2] & SHF_ALLOC and sh[1] != SHT_NOBITS and sh[3] <= addr and addr + size <= sh[3] + sh[5]:
                at = sh[4] + addr - sh[3]
                return self.data[at : at + size]
        raise TableError(f"address 0x{addr:x} (+{size}) is in no loaded section with contents")

    def pointer(self, addr: int) -> int:
        if addr in self.relative:
            if self.relative[addr] is None:
                raise TableError(f"the pointer at 0x{addr:x} has a relocation other than RELATIVE")
            return self.relative[addr]
        return struct.unpack(LAYOUT[self.cls]["ptr"], self.read(addr, self.ptr_size))[0]

    def string(self, addr: int) -> str:
        for sh in self.sections:
            if sh[2] & SHF_ALLOC and sh[1] != SHT_NOBITS and sh[3] <= addr < sh[3] + sh[5]:
                at = sh[4] + addr - sh[3]
                end = self.data.index(b"\0", at)
                return self.data[at:end].decode("utf-8")
        raise TableError(f"string address 0x{addr:x} is in no loaded section with contents")


def one(elf: Elf, pattern: re.Pattern, what: str) -> tuple[str, int, int]:
    found = elf.defined(pattern)
    if len(found) != 1:
        names = ", ".join(n for n, _, _ in found) or "none"
        raise TableError(f"expected exactly one {what} symbol, found {len(found)} ({names})")
    return found[0]


def check(elf: Elf, manifest: dict) -> list[str]:
    params = sorted(manifest["params"].items(), key=lambda kv: kv[1]["id"])
    count = len(params)
    if count != manifest["count"] or [p["id"] for _, p in params] != list(range(count)):
        raise TableError("the manifest's ids are not 0 .. count - 1")
    offsets, record_size = record_layout(elf.ptr_size)
    _, defaults, defaults_size = one(elf, DEFAULTS_RE, "kParamDefaults")
    _, names, names_size = one(elf, NAMES_RE, "kParamNames")
    findings = []
    if defaults_size != count * record_size:
        findings.append(f"kParamDefaults is {defaults_size} bytes; the set has {count} records of {record_size} bytes "
                        f"= {count * record_size}")
    if names_size != count * elf.ptr_size:
        findings.append(f"kParamNames is {names_size} bytes; the set has {count} names of {elf.ptr_size} bytes")
    if findings:
        return findings
    schema = hashlib.sha256()
    for i, (name, p) in enumerate(params):
        at = defaults + i * record_size
        linked_name = elf.string(elf.pointer(names + i * elf.ptr_size))
        type_code = elf.read(at + offsets["type"], 1)[0]
        linked_type = PARAM_TYPES[type_code] if type_code < len(PARAM_TYPES) else f"<{type_code}>"
        value_at = offsets["f32"] if linked_type == "f32" else offsets["i32"]
        linked_bits = struct.unpack("<I", elf.read(at + value_at, struct.calcsize("<I")))[0]
        linked_unit = elf.string(elf.pointer(at + offsets["unit"]))
        want = (name, p["type"], p["unit"], int(p["default_bits"], 0))
        got = (linked_name, linked_type, linked_unit, linked_bits)
        if got != want:
            findings.append(f"record {i}: linked (name, type, unit, value bits) {got[:3] + (hex(got[3]),)}; the set has "
                            f"{want[:3] + (hex(want[3]),)}")
        schema.update(f"{linked_name}\x1f{linked_type}\x1f{linked_unit}\x1e".encode())
    # kParamSchemaHash: the first 8 bytes, big-endian, of that SHA-256 (tools/gen/params_gen.py, schema hash rule).
    linked_schema = int.from_bytes(schema.digest()[: struct.calcsize("<Q")], "big")
    if linked_schema != int(manifest["schema_hash"], 0):
        findings.append(f"the linked table's schema hash is 0x{linked_schema:016x}; the set's is {manifest['schema_hash']}")
    return findings


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--elf", required=True, type=Path)
    ap.add_argument("--manifest", required=True, type=Path)
    ap.add_argument("--delete-on-failure", action="store_true")
    args = ap.parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        elf = Elf(args.elf)
    except (OSError, ValueError, TableError) as e:
        print(f"PARAM-TABLE-ERROR: {e}", file=sys.stderr)
        return 2
    try:
        findings = check(elf, manifest)
    except (TableError, ValueError, KeyError, IndexError, struct.error) as e:
        findings = [str(e)]
    if findings:
        for f in findings:
            print(f"PARAM-TABLE: {args.elf}: {f} ({args.manifest})", file=sys.stderr)
        if args.delete_on_failure:
            args.elf.unlink(missing_ok=True)
            print(f"PARAM-TABLE: {args.elf} removed", file=sys.stderr)
        return 1
    print(f"check_param_table: {args.elf.name}: {manifest['count']} records, schema {manifest['schema_hash']}, names, "
          f"types, units and values equal {args.manifest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
