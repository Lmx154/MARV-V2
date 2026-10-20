#!/usr/bin/env python3
"""The fake board console of the GCS end-to-end test (ground-segment unit GX-16; tools/gs/e2e.sh light mode). A test tool, never flown.

It stands where the board's console CDC stands for the helper's console relay (tools/gs/console.py): a pty whose slave is linked under
--byid-dir as MARV_E2E_CONSOLE_NAME (an `-if00` entry the helper lists as a console interface). While a host has the slave open it
  - prints BANNER once, CONNECT_DELAY_S after the open (the helper's serial setup flushes the input queue just after it opens the port),
    then one TICK line a second, as the board prints its stats line a second;
  - answers every complete line it receives (a line ends at LF, as the board's console parser ends it) with one line,
    `received <n> bytes: <the line as JSON>`, so that a test can see exactly what reached the "board", spaces and all.
The fake knows no command of the board's. It opens no real port and prints "fake console ready" once its link exists.

  uv run python tools/gs/e2e_fake_console.py --byid-dir DIR
"""

from __future__ import annotations

import argparse
import json
import os
import select
import time
from pathlib import Path

NAME = "usb-MARV_E2E_console-if00"
BANNER = "marv_fc3v2 fake console (e2e)"
CONNECT_DELAY_S = 0.3  # harness value: after the host opens the slave, before the banner
TICK_S = 1.0  # harness value: the period of the stats-like line
POLL_MS = 100  # harness value: the poll period of the loop


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--byid-dir", type=Path, required=True)
    a = ap.parse_args(argv)
    master, slave = os.openpty()
    path = os.ttyname(slave)
    os.close(slave)  # no opener of ours: POLLHUP on the master says no host has the port open
    a.byid_dir.mkdir(parents=True, exist_ok=True)
    (a.byid_dir / NAME).symlink_to(path)
    poll = select.poll()
    poll.register(master, select.POLLIN)
    print("fake console ready", flush=True)
    host = False
    opened_at = 0.0
    banner_sent = False
    next_tick = 0.0
    ticks = 0
    pending = b""
    start = time.monotonic()
    while True:
        now = time.monotonic()
        events = poll.poll(POLL_MS)
        hung_up = any(e & select.POLLHUP for _, e in events)
        if hung_up and not any(e & select.POLLIN for _, e in events):
            host, banner_sent, pending = False, False, b""
            time.sleep(POLL_MS / 1000)  # poll() reports the hang-up at once: do not spin
            continue
        if not host:
            host, opened_at, next_tick = True, now, now + CONNECT_DELAY_S + TICK_S
        if any(e & select.POLLIN for _, e in events):
            try:
                data = os.read(master, 4096)
            except OSError:
                continue
            pending += data
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                text = line.decode("utf-8", "replace")
                os.write(master, f"received {len(line)} bytes: {json.dumps(text)}\r\n".encode("utf-8"))
        if host and not banner_sent and now - opened_at >= CONNECT_DELAY_S:
            banner_sent = True
            os.write(master, f"{BANNER}\r\n".encode("utf-8"))
        if host and banner_sent and now >= next_tick:
            ticks += 1
            next_tick = now + TICK_S
            os.write(master, f"[{now - start:7.3f}] tick {ticks}\r\n".encode("utf-8"))


if __name__ == "__main__":
    raise SystemExit(main())
