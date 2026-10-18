#!/usr/bin/env python3
"""Bench only (ground-segment unit 13): capture the FC3v2 console into a raw log, bounded in time.

  capture.py <device> <out.log> <max_seconds> [stop_text]

Opens the CDC port with DTR asserted (raw, 115200 8N1 as the board ignores the rate), reads until `max_seconds` elapse or the
text `stop_text` has been seen (then reads 3 s more, so the lines that follow it are kept), writes every byte to <out.log>
and exits 0 if the stop text was seen (or none was asked for), 1 on the time limit. Never an unbounded read.
"""
import fcntl
import os
import select
import struct
import sys
import termios
import time


def main() -> int:
    dev, out, max_s = sys.argv[1], sys.argv[2], float(sys.argv[3])
    stop = sys.argv[4].encode() if len(sys.argv) > 4 else None
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[3] &= ~(termios.ICANON | termios.ECHO)
    attrs[1] &= ~termios.OPOST
    attrs[0] &= ~termios.ICRNL
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR))
    buf = b""
    start = time.time()
    seen_at = None
    while time.time() - start < max_s:
        if seen_at is not None and time.time() - seen_at > 3:
            break
        ready, _, _ = select.select([fd], [], [], 0.2)
        if ready:
            try:
                buf += os.read(fd, 65536)
            except BlockingIOError:
                pass
        if stop is not None and seen_at is None and stop in buf:
            seen_at = time.time()
    os.close(fd)
    with open(out, "wb") as f:
        f.write(buf)
    print(f"captured {len(buf)} bytes in {time.time() - start:.1f} s to {out}")
    return 0 if stop is None or seen_at is not None else 1


if __name__ == "__main__":
    sys.exit(main())
