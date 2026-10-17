# SPDX-License-Identifier: BSD-3-Clause
"""Read a MAVLink dialect XML (with its `<include>` chain) and derive what a codec needs: the field order on the wire, the
payload sizes and the CRC_EXTRA byte.

Own implementation from the published serialisation rules (https://mavlink.io/en/guide/serialization.html): fields are
sorted by the size of their element type, largest first, keeping the XML order among equal sizes; extension fields (after
`<extensions/>`) are not reordered and follow the others; CRC_EXTRA is the X.25 CRC over the message name, then for each
non-extension field in wire order its type, its name and, for an array, its length. tests check the result against the
table of the C headers that pymavlink's mavgen generates from the same XML.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

ELEMENT_BYTES = {
    "char": 1,
    "uint8_t": 1,
    "int8_t": 1,
    "uint8_t_mavlink_version": 1,
    "uint16_t": 2,
    "int16_t": 2,
    "uint32_t": 4,
    "int32_t": 4,
    "float": 4,
    "uint64_t": 8,
    "int64_t": 8,
    "double": 8,
}
TYPE_RE = re.compile(r"^([a-z0-9_]+?)(?:\[(\d+)\])?$")
X25_INIT = 0xFFFF
X25_POLY = 0x8408


@dataclass
class Field:
    name: str
    ctype: str
    array_len: int
    enum: str | None
    extension: bool
    offset: int = 0

    @property
    def element_bytes(self) -> int:
        return ELEMENT_BYTES[self.ctype]

    @property
    def wire_bytes(self) -> int:
        return self.element_bytes * (self.array_len or 1)

    @property
    def is_array(self) -> bool:
        return self.array_len > 0

    @property
    def crc_type(self) -> str:
        return "uint8_t" if self.ctype == "uint8_t_mavlink_version" else self.ctype


@dataclass
class Message:
    id: int
    name: str
    fields: list[Field]  # XML order
    wire: list[Field] = field(default_factory=list)  # wire order, offsets set
    crc_extra: int = 0
    min_len: int = 0  # bytes without the extension fields
    max_len: int = 0  # bytes with them


@dataclass
class Enum:
    name: str
    entries: list[tuple[str, int]]


@dataclass
class Dialect:
    messages: dict[str, Message] = field(default_factory=dict)
    enums: dict[str, Enum] = field(default_factory=dict)
    own_enums: list[str] = field(default_factory=list)  # enums defined in the primary file, in file order
    mavlink_version: int = 0  # the <version> of the file that defines HEARTBEAT


def x25_accumulate(crc: int, data: bytes) -> int:
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ X25_POLY if crc & 1 else crc >> 1
    return crc


def crc_extra_of(msg: Message) -> int:
    crc = x25_accumulate(X25_INIT, (msg.name + " ").encode())
    for f in msg.wire:
        if f.extension:
            continue
        crc = x25_accumulate(crc, (f.crc_type + " ").encode())
        crc = x25_accumulate(crc, (f.name + " ").encode())
        if f.is_array:
            crc = x25_accumulate(crc, bytes([f.array_len]))
    return (crc & 0xFF) ^ (crc >> 8)


def finish_message(msg: Message) -> None:
    base = [f for f in msg.fields if not f.extension]
    ext = [f for f in msg.fields if f.extension]
    msg.wire = sorted(base, key=lambda f: -f.element_bytes) + ext
    offset = 0
    for f in msg.wire:
        f.offset = offset
        offset += f.wire_bytes
    msg.max_len = offset
    msg.min_len = sum(f.wire_bytes for f in base)
    msg.crc_extra = crc_extra_of(msg)


def parse_field(node: ET.Element, extension: bool) -> Field:
    match = TYPE_RE.match(node.attrib["type"])
    if not match or match.group(1) not in ELEMENT_BYTES:
        raise ValueError(f"unsupported field type {node.attrib['type']!r} in {node.attrib.get('name')}")
    return Field(
        name=node.attrib["name"],
        ctype=match.group(1),
        array_len=int(match.group(2) or 0),
        enum=node.attrib.get("enum"),
        extension=extension,
    )


def parse_file(path: Path, include_dirs: list[Path], out: Dialect, seen: set[Path], primary: bool) -> None:
    path = path.resolve()
    if path in seen:
        return
    seen.add(path)
    root = ET.parse(path).getroot()
    for node in root.findall("include"):
        name = (node.text or "").strip()
        for directory in [path.parent, *include_dirs]:
            if (directory / name).exists():
                parse_file(directory / name, include_dirs, out, seen, False)
                break
        else:
            raise FileNotFoundError(f"{path}: include {name!r} not found")
    for enum in root.iterfind("enums/enum"):
        entries = []
        for entry in enum.findall("entry"):
            try:
                entries.append((entry.attrib["name"], int(entry.attrib["value"], 0)))
            except ValueError:
                entries.append((entry.attrib["name"], -1))  # an expression value; only the primary file's enums are emitted
        if enum.attrib["name"] in out.enums:  # common.xml re-opens enums of minimal.xml
            out.enums[enum.attrib["name"]].entries.extend(entries)
        else:
            out.enums[enum.attrib["name"]] = Enum(enum.attrib["name"], entries)
        if primary:
            out.own_enums.append(enum.attrib["name"])
    for node in root.iterfind("messages/message"):
        fields = []
        extension = False
        for child in node:
            if child.tag == "extensions":
                extension = True
            elif child.tag == "field":
                fields.append(parse_field(child, extension))
        msg = Message(id=int(node.attrib["id"]), name=node.attrib["name"], fields=fields)
        finish_message(msg)
        if msg.name in out.messages:
            raise ValueError(f"{path}: message {msg.name} defined twice")
        out.messages[msg.name] = msg
        if msg.name == "HEARTBEAT":
            out.mavlink_version = int(root.findtext("version", "0"))


def parse_dialect(path: Path, include_dirs: list[Path] | None = None) -> Dialect:
    out = Dialect()
    parse_file(Path(path), include_dirs or [], out, set(), True)
    ids: dict[int, str] = {}
    for msg in out.messages.values():
        if msg.id in ids:
            raise ValueError(f"message id {msg.id} is used by {ids[msg.id]} and {msg.name}")
        ids[msg.id] = msg.name
    return out


def read_scope(path: Path) -> list[str]:
    names = []
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            names.append(line)
    return names


def camel(name: str) -> str:
    return "".join(part.capitalize() for part in name.split("_"))


def enum_entry_camel(enum_name: str, entry_name: str) -> str:
    prefix = enum_name + "_"
    return camel(entry_name[len(prefix):] if entry_name.startswith(prefix) else entry_name)
