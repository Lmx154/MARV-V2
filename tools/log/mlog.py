#!/usr/bin/env python3
"""Decoder of the MARV onboard flight log (.mlog), format version 1 (fw/log/include/marv/log/format.hpp is the layout's source).

A file is a sequence of 512-byte self-validating sectors: magic "MLOG", session id u64, sector sequence u32, offset of the first
record that begins in the sector u16, format version u16, 488 payload bytes, CRC-32 u32 over everything before it. Records are
back to back in the payload stream and may span sectors. The decoder

  * validates every sector (magic, version, CRC, first-record offset, session, sequence) and, in the default strict mode,
    STOPS at the first invalid one and reports it (what follows a power cut is never trusted);
  * rejects stale sectors: a sector whose CRC is good but whose session id is not the file's (left over from an earlier file in
    the same region);
  * in resync mode skips invalid sectors and sectors of other sessions, reports a hole when the sequence jumps, drops the record
    cut by it and continues at the next record boundary the sector header names;
  * decodes the header records (hashes, boot report, tick, record kinds and fields with units, the parameter table) and then
    every data record by the field list the HEADER gives, so a reader needs nothing else; the 32-bit stamps (*_lo_us) are
    unwrapped to 64-bit microseconds (column t_us) against the Status/End/Info records' full stamps.

Output: Decoded.records (a list of dicts), Decoded.header (a dict), Decoded.problems; as CSV (one file per kind) or JSON from the CLI.
Standard library only.
"""

from __future__ import annotations

import argparse
import csv
import json
import struct
import sys
import zlib
from dataclasses import dataclass, field
from pathlib import Path

SECTOR = 512
MAGIC = b"MLOG"
FORMAT_VERSION = 1
SEC_HEADER = 20  # magic 4, session 8, sequence 4, first-record offset 2, version 2
SEC_CRC = 4
PAYLOAD = SECTOR - SEC_HEADER - SEC_CRC
NO_RECORD = PAYLOAD
LEAD = 4  # kind u8, sub u8, length u16

KIND_NAMES = {
    0: "pad", 1: "imu", 2: "rate_tick", 3: "manual_control", 4: "gap", 5: "status", 6: "end", 7: "telemetry",
    8: "header_info", 9: "header_kind", 10: "header_param", 11: "header_field", 12: "header_telemetry",
}
K_PAD, K_IMU, K_RATE, K_MANUAL, K_GAP, K_STATUS, K_END, K_TELEMETRY = 0, 1, 2, 3, 4, 5, 6, 7
K_INFO, K_HKIND, K_HPARAM, K_HFIELD, K_HTELEM = 8, 9, 10, 11, 12
# Records of the ring have a fixed size and no length field; every other kind carries its length in its lead.
RING_SIZES = {K_IMU: 32, K_RATE: 16, K_MANUAL: 112, K_GAP: 32}
FIELD_TYPES = {0: ("B", 1), 1: ("H", 2), 2: ("I", 4), 3: ("Q", 8), 4: ("f", 4), 5: (None, 0)}
FIELD_TYPE_NAMES = {0: "u8", 1: "u16", 2: "u32", 3: "u64", 4: "f32", 5: "bytes"}
STATE_NAMES = {0: "Absent", 1: "Ready", 2: "Logging", 3: "PostRoll", 4: "Full", 5: "Fault"}
END_REASONS = {0: "disarm", 1: "full", 2: "fault"}
WRAP = 1 << 32


@dataclass
class Problem:
    sector: int  # index of the sector in the file
    reason: str  # magic | version | crc | first_offset | session | sequence | hole | record | partial_sector | unknown_kind
    detail: str = ""

    def __str__(self) -> str:
        return f"sector {self.sector}: {self.reason}" + (f" ({self.detail})" if self.detail else "")


@dataclass
class Decoded:
    header: dict = field(default_factory=dict)
    records: list[dict] = field(default_factory=list)
    problems: list[Problem] = field(default_factory=list)
    session: int | None = None
    sectors_ok: int = 0
    sectors_total: int = 0
    stopped_at: int | None = None  # strict mode: the sector it stopped at
    ended: bool = False  # an End record was read
    ends_inside_record: bool = False  # the sectors run out in the middle of a record (a cut, not a corruption)

    @property
    def clean(self) -> bool:
        return not self.problems and self.ended


def sector_check(buf: bytes) -> tuple[str | None, dict]:
    """Validate one 512-byte sector alone. Returns (reason or None, the sector's fields)."""
    if buf[:4] != MAGIC:
        return "magic", {}
    session, seq, first, version = struct.unpack_from("<QIHH", buf, 4)
    info = {"session": session, "seq": seq, "first": first, "version": version}
    if zlib.crc32(buf[: SECTOR - SEC_CRC]) != struct.unpack_from("<I", buf, SECTOR - SEC_CRC)[0]:
        return "crc", info
    if version != FORMAT_VERSION:
        return "version", info
    if first > NO_RECORD:
        return "first_offset", info
    return None, info


class _Stream:
    """Reassembles records from sector payloads and decodes them."""

    def __init__(self, out: Decoded) -> None:
        self.out = out
        self.pending = bytearray()
        self.fields: dict[int, list[dict]] = {}  # kind -> [{name, unit, offset, size, type}]
        self.last_t: int | None = None
        self.sector = 0

    # -- records -----------------------------------------------------------------------------------------------
    @staticmethod
    def record_length(buf: bytes) -> int | None:
        """The length of the record starting at buf[0], or None if buf is too short to say; -1 for an unknown kind."""
        kind = buf[0]
        if kind in RING_SIZES:
            return RING_SIZES[kind]
        if kind in KIND_NAMES and kind != K_PAD:
            if len(buf) < LEAD:
                return None
            n = struct.unpack_from("<H", buf, 2)[0]
            return n if n >= LEAD else -1
        return -1

    def unwrap(self, lo: int) -> int:
        if self.last_t is None:
            return lo
        delta = (lo - self.last_t) % WRAP
        if delta >= WRAP // 2:
            delta -= WRAP
        t = self.last_t + delta
        self.last_t = t
        return t

    def emit(self, rec: bytes) -> None:
        kind = rec[0]
        out = self.out
        if kind == K_INFO:
            self._info(rec)
        elif kind == K_HKIND:
            n, = struct.unpack_from("<H", rec, LEAD)
            self.out.header.setdefault("kinds", {})[rec[1]] = {"name": _cstr(rec, LEAD + 2)[0], "record_bytes": n}
        elif kind == K_HFIELD:
            off, size, ftype = struct.unpack_from("<HBB", rec, LEAD)
            name, p = _cstr(rec, LEAD + 4)
            unit, _ = _cstr(rec, p)
            self.fields.setdefault(rec[1], []).append(
                {"name": name, "unit": unit, "offset": off, "size": size, "type": ftype})
        elif kind == K_HTELEM:
            version, count = struct.unpack_from("<II", rec, LEAD)
            out.header["telemetry"] = {"version": version, "field_count": count}
        elif kind == K_HPARAM:
            pid, flags, bits = struct.unpack_from("<HHI", rec, LEAD)
            name, p = _cstr(rec, LEAD + 8)
            unit, _ = _cstr(rec, p)
            value = struct.unpack("<f", struct.pack("<I", bits))[0] if rec[1] == 0 else struct.unpack("<i", struct.pack("<I", bits))[0]
            out.header.setdefault("params", []).append(
                {"id": pid, "name": name, "type": "f32" if rec[1] == 0 else "i32", "value": value, "unit": unit,
                 "locked": bool(flags & 1)})
        else:
            self._data(kind, rec)

    def _info(self, rec: bytes) -> None:
        version, sector_bytes, start = struct.unpack_from("<HHQ", rec, LEAD)
        o = LEAD + 12
        image, config = rec[o: o + 32], rec[o + 32: o + 64]
        schema, tick_num, tick_den, odr, boot_seq = struct.unpack_from("<QIIfI", rec, o + 64)
        o2 = o + 64 + 8 + 4 + 4 + 4 + 4
        names = ("config_source", "slot", "config_status", "fault", "request", "request_fault")
        self.out.header["info"] = {
            "format_version": version, "sector_bytes": sector_bytes, "session_start_us": start,
            "image_hash": image.hex(), "config_hash": config.hex(), "schema_hash": schema,
            "tick_num_us": tick_num, "tick_den": tick_den, "odr_hz": odr,
            "boot": dict(zip(names, rec[o2: o2 + 6]), sequence=boot_seq)}
        self.last_t = start

    def _data(self, kind: int, rec: bytes) -> None:
        name = KIND_NAMES.get(kind, f"kind{kind}")
        row: dict = {"kind": name, "sector": self.sector}
        spec = self.fields.get(kind)
        if spec is None:
            row["raw"] = rec.hex()
        else:
            for f in spec:
                fmt, size = FIELD_TYPES.get(f["type"], (None, 0))
                if fmt is None:
                    row[f["name"]] = rec[f["offset"]: f["offset"] + f["size"]].hex()
                else:
                    row[f["name"]] = struct.unpack_from("<" + fmt, rec, f["offset"])[0]
        if kind == K_STATUS:
            row["state"] = STATE_NAMES.get(row.get("state"), row.get("state"))
        if kind == K_END:
            row["reason"] = END_REASONS.get(row.get("reason"), row.get("reason"))
            self.out.ended = True
        if kind in (K_STATUS, K_END):
            self.last_t = row["t_us"]  # a full stamp re-anchors the unwrapping
        elif kind != K_MANUAL:
            for key in row:
                if key.endswith("_lo_us"):
                    row["t_us"] = self.unwrap(row[key])
                    break
        self.out.records.append(row)

    # -- sector payloads --------------------------------------------------------------------------------------
    def lose_pending(self, why: str) -> None:
        if self.pending:
            self.out.problems.append(Problem(self.sector, "record", why))
        self.pending.clear()

    def feed(self, sector: int, payload: bytes, first: int) -> None:
        self.sector = sector
        limit = len(payload) if first == NO_RECORD else first  # the payload before `limit` continues an earlier record
        pos = 0
        had_pending = bool(self.pending)
        while self.pending and pos < limit:
            n = self.record_length(bytes(self.pending))
            if n == -1:
                self.lose_pending("unknown kind in a continued record")
                break
            want = (LEAD - len(self.pending)) if n is None else n - len(self.pending)
            take = min(want, limit - pos)
            self.pending += payload[pos: pos + take]
            pos += take
            n = self.record_length(bytes(self.pending))
            if n == -1:
                self.lose_pending("unknown kind in a continued record")
            elif n is not None and len(self.pending) == n:
                self.emit(bytes(self.pending))
                self.pending.clear()
        if self.pending:
            if first != NO_RECORD:
                self.lose_pending("record cut: the next record begins before it ends")
            else:
                return
        elif first == NO_RECORD:
            if had_pending and any(payload[pos:]):
                self.out.problems.append(Problem(sector, "record", "bytes after the end of a record, with none beginning"))
            return
        elif had_pending and pos < first and any(payload[pos:first]):
            self.out.problems.append(Problem(sector, "record", "bytes between a record's end and the first record offset"))
        pos = first
        while pos < len(payload):
            if payload[pos] == K_PAD:
                break
            rest = payload[pos:]
            n = self.record_length(rest)
            if n == -1:
                self.out.problems.append(Problem(sector, "unknown_kind", f"kind {payload[pos]} at payload offset {pos}"))
                break
            if n is None or n > len(rest):
                self.pending = bytearray(rest)
                break
            self.emit(bytes(rest[:n]))
            pos += n


def _cstr(buf: bytes, pos: int) -> tuple[str, int]:
    end = buf.index(0, pos)
    return buf[pos:end].decode("utf-8", "replace"), end + 1


def decode(data: bytes, *, resync: bool = False) -> Decoded:
    """Decode a whole file. Strict (default): stops at the first invalid sector. resync: skips invalid sectors and holes."""
    out = Decoded()
    out.sectors_total = len(data) // SECTOR
    if len(data) % SECTOR:
        out.problems.append(Problem(out.sectors_total, "partial_sector", f"{len(data) % SECTOR} trailing bytes"))
    stream = _Stream(out)
    expected = 0
    for i in range(out.sectors_total):
        buf = data[i * SECTOR: (i + 1) * SECTOR]
        reason, info = sector_check(buf)
        if reason is None and out.session is not None and info["session"] != out.session:
            reason = "session"
        if reason is None and out.session is not None and info["seq"] < expected:
            reason = "sequence"
        if reason is not None:
            out.problems.append(Problem(i, reason, _detail(reason, info, out)))
            if not resync:
                out.stopped_at = i
                break
            continue
        if out.session is None:
            out.session = info["session"]
            expected = info["seq"]
        if info["seq"] > expected:
            out.problems.append(Problem(i, "hole", f"sequence {info['seq']}, expected {expected}"))
            if not resync:
                out.stopped_at = i
                break
            stream.lose_pending("sectors missing")
        expected = info["seq"] + 1
        out.sectors_ok += 1
        stream.feed(i, buf[SEC_HEADER: SEC_HEADER + PAYLOAD], info["first"])
        if out.ended:
            break
    out.ends_inside_record = out.stopped_at is None and not out.ended and bool(stream.pending)
    return out


def _detail(reason: str, info: dict, out: Decoded) -> str:
    if reason == "session":
        return f"session {info['session']:#018x}, the file's is {out.session:#018x}"
    if reason == "sequence":
        return f"sequence {info['seq']}"
    return ""


def records_of(dec: Decoded, kind: str) -> list[dict]:
    return [r for r in dec.records if r["kind"] == kind]


def write_csv(dec: Decoded, directory: Path) -> list[Path]:
    directory.mkdir(parents=True, exist_ok=True)
    written = []
    for kind in sorted({r["kind"] for r in dec.records}):
        rows = records_of(dec, kind)
        cols: list[str] = []
        for r in rows:
            for k in r:
                if k not in cols:
                    cols.append(k)
        path = directory / f"{kind}.csv"
        with path.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols, restval="")
            w.writeheader()
            w.writerows(rows)
        written.append(path)
    hpath = directory / "header.json"
    hpath.write_text(json.dumps(dec.header, indent=2, default=str))
    written.append(hpath)
    return written


def summary(dec: Decoded) -> str:
    lines = [f"sectors: {dec.sectors_ok} valid of {dec.sectors_total}; session: "
             + (f"{dec.session:#018x}" if dec.session is not None else "none")]
    counts: dict[str, int] = {}
    for r in dec.records:
        counts[r["kind"]] = counts.get(r["kind"], 0) + 1
    lines.append("records: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())))
    lines.append("end record: " + ("yes" if dec.ended else "NO (the log is cut short)"))
    if dec.ends_inside_record:
        lines.append("the last sector ends inside a record (the partial record is not returned)")
    for p in dec.problems:
        lines.append(f"PROBLEM {p}")
    if dec.stopped_at is not None:
        lines.append(f"stopped at sector {dec.stopped_at}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Decode a MARV onboard flight log (.mlog)")
    ap.add_argument("file", type=Path)
    ap.add_argument("--resync", action="store_true", help="skip invalid sectors and holes instead of stopping at the first")
    ap.add_argument("--csv-dir", type=Path, help="write one CSV per record kind and header.json here")
    ap.add_argument("--header", action="store_true", help="print the header as JSON")
    args = ap.parse_args(argv)
    dec = decode(args.file.read_bytes(), resync=args.resync)
    if args.csv_dir:
        write_csv(dec, args.csv_dir)
    if args.header:
        print(json.dumps(dec.header, indent=2, default=str))
    print(summary(dec))
    return 0 if dec.clean else 1


if __name__ == "__main__":
    sys.exit(main())
