#!/usr/bin/env python3
"""The ground log (ground spec GS-8): every byte sent to and received from the device, with a monotonic timestamp.

File `ground.log` in the run directory. All integers little-endian.

  header   8 bytes   magic b"MARVGL01"
           u64       wall_ns   time.time_ns() when the log was created
           u64       mono_ns   time.monotonic_ns() at the same instant
  record   u8        kind: 1 RX = bytes received from the device, 2 TX = bytes sent to the device, 3 EVENT = UTF-8 text
           u64       mono_ns   time.monotonic_ns() when the chunk was read from (RX) or written to (TX) the device
           u32       length    payload bytes
           payload   length bytes

A record is one chunk as the operating system handed it over: no frame is parsed, so a chunk can hold part of a frame or
several. Concatenating the payloads of one kind gives exactly the byte stream of that direction. The writer flushes every
record (one write call), so a crash loses at most a record the reader recognises as truncated and ignores.

  python tools/gs/groundlog.py <ground.log> [--rx | --tx | --info]   writes that direction's byte stream to stdout
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

MAGIC = b"MARVGL01"
HEADER = struct.Struct("<8sQQ")
RECORD = struct.Struct("<BQI")
RX, TX, EVENT = 1, 2, 3
KIND_NAMES = {RX: "rx", TX: "tx", EVENT: "event"}


@dataclass(frozen=True)
class Record:
    kind: int
    t_ns: int
    data: bytes


@dataclass(frozen=True)
class Header:
    wall_ns: int
    mono_ns: int


class GroundLog:
    """Append-only writer. Not thread safe; the helper writes from its event loop only."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self._file = open(self.path, "xb", buffering=0)
        self._file.write(HEADER.pack(MAGIC, time.time_ns(), time.monotonic_ns()))

    def write(self, kind: int, data: bytes, t_ns: int | None = None) -> None:
        if kind not in KIND_NAMES:
            raise ValueError(f"unknown record kind {kind}")
        if t_ns is None:
            t_ns = time.monotonic_ns()
        self._file.write(RECORD.pack(kind, t_ns, len(data)) + data)

    def event(self, text: str) -> None:
        self.write(EVENT, text.encode("utf-8"))

    def close(self) -> None:
        if not self._file.closed:
            os.fsync(self._file.fileno())
            self._file.close()


def read_header(path: Path) -> Header:
    with open(path, "rb") as f:
        raw = f.read(HEADER.size)
    if len(raw) < HEADER.size:
        raise ValueError(f"{path}: shorter than the ground log header")
    magic, wall_ns, mono_ns = HEADER.unpack(raw)
    if magic != MAGIC:
        raise ValueError(f"{path}: not a ground log (magic {magic!r})")
    return Header(wall_ns, mono_ns)


def read_records(path: Path) -> Iterator[Record]:
    """Every complete record in order. A truncated last record (a crash mid-write) ends the iteration."""
    read_header(path)
    with open(path, "rb") as f:
        f.seek(HEADER.size)
        while True:
            raw = f.read(RECORD.size)
            if len(raw) < RECORD.size:
                return
            kind, t_ns, length = RECORD.unpack(raw)
            if kind not in KIND_NAMES:
                raise ValueError(f"{path}: unknown record kind {kind} at offset {f.tell() - RECORD.size}")
            data = f.read(length)
            if len(data) < length:
                return
            yield Record(kind, t_ns, data)


def byte_stream(path: Path, kind: int) -> Iterator[bytes]:
    """The chunks of one direction (RX or TX), in order: the stream the device sent or was sent."""
    for r in read_records(path):
        if r.kind == kind:
            yield r.data


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log", type=Path)
    which = ap.add_mutually_exclusive_group()
    which.add_argument("--rx", action="store_true", help="write the received byte stream to stdout")
    which.add_argument("--tx", action="store_true", help="write the sent byte stream to stdout")
    which.add_argument("--info", action="store_true", help="print the header, the record counts and the events (default)")
    a = ap.parse_args(argv)
    if a.rx or a.tx:
        out = sys.stdout.buffer
        for chunk in byte_stream(a.log, RX if a.rx else TX):
            out.write(chunk)
        return 0
    h = read_header(a.log)
    count = {k: 0 for k in KIND_NAMES.values()}
    size = dict(count)
    print(f"wall_ns {h.wall_ns} mono_ns {h.mono_ns}")
    for r in read_records(a.log):
        name = KIND_NAMES[r.kind]
        count[name] += 1
        size[name] += len(r.data)
        if r.kind == EVENT:
            print(f"event t_ns {r.t_ns}: {r.data.decode('utf-8', 'replace')}")
    for name in count:
        print(f"{name}: {count[name]} records, {size[name]} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
