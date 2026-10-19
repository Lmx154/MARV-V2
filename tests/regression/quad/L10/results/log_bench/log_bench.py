#!/usr/bin/env python3
"""Bench only (onboard logger unit U2, bench log image MARV_FC3V2_LOG_BENCH): drive the image's `log` console commands and pull the file.

  log_bench.py run  --seconds N --out FILE.mlog [--console DEV] [--link DEV]   log N seconds, stop, wait for the file to close, pull it
  log_bench.py pull --index I --out FILE.mlog [--console DEV] [--link DEV]     pull file I (the card's /marv/log/0000000I.mlog)
  both take --dump-timeout S (seconds allowed for the dump, default 300; the soak's 567 MB needs about 1600 s at the measured 0.37 MB/s)

`run` sends `log start` (the vehicle is not armed; the image's bench trigger logs without arming), waits N seconds, sends `log stop`, waits
for the logger to leave PostRoll (the end record is written and the file truncated to its used bytes), then sends `log dump <index>`:
the image streams the file's bytes on the link port (USB interface 2) while the console reports START/DONE. The raw console of the
whole run is written next to the output as FILE.console.log. Every read is time-bounded. Exit 0 if the dump completed with every byte.
"""
from __future__ import annotations

import argparse
import fcntl
import os
import re
import select
import struct
import sys
import termios
import tty
import time
from pathlib import Path

CONSOLE = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if00"
LINK = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if02"
POSTROLL_AND_CLOSE_MARGIN_S = 30.0  # labelled harness value: log_post_roll_s is 5 s (PLACEHOLDER), the close is a metadata write
DUMP_TIMEOUT_S = 300.0  # labelled harness value: 64 MiB at no less than 0.22 MB/s


def open_port(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)  # every byte of the link port's dump arrives untouched (no CR/LF mapping, no XON/XOFF, no echo)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    attrs[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR))
    return fd


class Bench:
    def __init__(self, console: str, link: str, console_log: Path):
        self.con = open_port(console)
        self.lnk = open_port(link)
        self.text = b""
        self.pending_link = bytearray()
        self.console_log = console_log.open("wb")

    def pump(self, timeout: float) -> None:
        ready, _, _ = select.select([self.con, self.lnk], [], [], timeout)
        for fd in ready:
            try:
                data = os.read(fd, 1 << 20)
            except BlockingIOError:
                continue
            if fd == self.con:
                self.text += data
                self.console_log.write(data)
            else:
                self.pending_link += data

    def send(self, line: str) -> None:
        os.write(self.con, line.encode() + b"\n")

    def wait_for(self, pattern: str, timeout: float, since: int = 0) -> re.Match[str] | None:
        end = time.time() + timeout
        rx = re.compile(pattern)
        while time.time() < end:
            m = rx.search(self.text[since:].decode("utf-8", "replace"))
            if m:
                return m
            self.pump(0.2)
        return None

    def close(self) -> None:
        self.console_log.close()
        os.close(self.con)
        os.close(self.lnk)


def pull(b: Bench, index: int, out: Path, dump_timeout_s: float = DUMP_TIMEOUT_S) -> bool:
    mark = len(b.text)
    b.pending_link.clear()
    b.send(f"log dump {index}")
    m = b.wait_for(r"log dump: (START index=\d+ file_bytes=\d+ bytes=(\d+)|REFUSED[^\n]*)", 10.0, mark)
    if m is None or m.group(2) is None:
        print("pull: no START from the image:", m.group(0) if m else "(no answer)", file=sys.stderr)
        return False
    want = int(m.group(2))
    end = time.time() + dump_timeout_s
    while len(b.pending_link) < want and time.time() < end:
        b.pump(0.5)
    out.write_bytes(bytes(b.pending_link[:want]))
    done = b.wait_for(r"log dump: (DONE|ABORTED|END OF FILE|READ ERROR)[^\n]*", 10.0, mark)
    print(f"pull: {len(b.pending_link)} of {want} bytes -> {out}; image says: {done.group(0) if done else '(nothing)'}")
    return len(b.pending_link) >= want and done is not None and done.group(1) == "DONE"


def run(b: Bench, seconds: float, out: Path, dump_timeout_s: float = DUMP_TIMEOUT_S) -> bool:
    b.pump(1.0)
    mark = len(b.text)
    b.send("log start")
    m = b.wait_for(r"log start: (bench trigger on|REFUSED[^\n]*)", 10.0, mark)
    if m is None or not m.group(1).startswith("bench"):
        print("run: log start failed:", m.group(0) if m else "(no answer)", file=sys.stderr)
        return False
    print(f"run: logging for {seconds:.0f} s (hold the board still, then rotate it as asked)")
    end = time.time() + seconds
    while time.time() < end:
        b.pump(0.5)
    mark = len(b.text)
    b.send("log stop")
    h = b.wait_for(r"log hist [^\n]*", POSTROLL_AND_CLOSE_MARGIN_S, mark)
    t = b.wait_for(r"log: PostRoll -> (\w+) [^\n]*file=(\d+)[^\n]*", 5.0, mark)
    if h is None or t is None:
        print("run: the logger did not leave PostRoll in time", file=sys.stderr)
        return False
    print("run:", t.group(0))
    print("run:", h.group(0))
    return pull(b, int(t.group(2)), out, dump_timeout_s)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("verb", choices=["run", "pull"])
    ap.add_argument("--seconds", type=float)
    ap.add_argument("--index", type=int)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--console", default=CONSOLE)
    ap.add_argument("--link", default=LINK)
    ap.add_argument("--dump-timeout", type=float, default=DUMP_TIMEOUT_S)
    a = ap.parse_args()
    b = Bench(a.console, a.link, a.out.with_suffix(".console.log"))
    try:
        if a.verb == "run":
            if a.seconds is None:
                ap.error("run needs --seconds")
            ok = run(b, a.seconds, a.out, a.dump_timeout)
        else:
            if a.index is None:
                ap.error("pull needs --index")
            ok = pull(b, a.index, a.out, a.dump_timeout)
    finally:
        b.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
