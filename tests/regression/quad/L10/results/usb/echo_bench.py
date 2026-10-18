#!/usr/bin/env python3
"""Bench only (ground-segment unit 12a): echo check of the link port and the IMU spacing under its traffic.

  echo_bench.py <out_prefix> <idle_s> <mb_check> <mb_sustained> <idle_after_s>

Image: MARV_FC3V2_USB_BENCH=ON and MARV_FC3V2_BENCH_STATS=ON. Reads the console (if00) throughout and keeps its raw bytes in
<out_prefix>.console.log. After <idle_s> seconds of idle it writes <mb_check> MiB of random bytes to the link port (if02) and reads the
echo back concurrently (a write that is not matched by reads would stall on the board's 64 byte FIFOs), compares sha256 and
reports bytes, seconds and kB/s; then the same for <mb_sustained> MiB (the sustained traffic the IMU spacing is read against);
then <idle_after_s> seconds of idle. Phase boundaries (host clock) and the arrival time of every console line go to
<out_prefix>.phases.json for analyze_echo.py. Every phase is bounded: a stalled echo ends after 20 s without progress.
"""
import hashlib
import json
import os
import select
import sys
import time

from usb_common import LINK, ConsoleReader, open_raw

MIB = 1 << 20
STALL_S = 20.0


def echo(fd, data):
    """Write data to the link port while reading the echo; returns (echoed bytes, seconds, stalled)."""
    sent = 0
    got = bytearray()
    t0 = last = time.time()
    while len(got) < len(data):
        now = time.time()
        if now - last > STALL_S:
            return bytes(got), now - t0, True
        wlist = [fd] if sent < len(data) else []
        r, w, _ = select.select([fd], wlist, [], 0.5)
        if r:
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                chunk = b""
            if chunk:
                got += chunk
                last = time.time()
        if w:
            try:
                n = os.write(fd, data[sent:sent + 4096])
            except BlockingIOError:
                n = 0
            if n:
                sent += n
                last = time.time()
    return bytes(got), time.time() - t0, False


def drain(fd, seconds):
    end = time.time() + seconds
    n = 0
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            try:
                n += len(os.read(fd, 65536))
            except BlockingIOError:
                pass
    return n


def main():
    prefix, idle_s, mb_check, mb_sust, idle_after = sys.argv[1], float(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), float(sys.argv[5])
    console = ConsoleReader()
    console.start()
    link = open_raw(LINK)
    phases = {}
    drained = drain(link, 1.0)  # whatever the host's own probing left in the board's FIFO
    print(f"drained {drained} stale bytes from the link port", flush=True)
    time.sleep(idle_s)
    result = {"drained_stale_bytes": drained}
    for name, mb in (("check", mb_check), ("sustained", mb_sust)):
        data = os.urandom(mb * MIB)
        want = hashlib.sha256(data).hexdigest()
        t_start = time.time()
        got, secs, stalled = echo(link, data)
        t_end = time.time()
        have = hashlib.sha256(got).hexdigest()
        phases[name] = [t_start, t_end]
        result[name] = {"bytes_sent": len(data), "bytes_echoed": len(got), "seconds": round(secs, 3),
                        "kB_per_s_each_way": round(len(got) / 1000 / secs, 1) if secs else None,
                        "sha256_sent": want, "sha256_echoed": have, "equal": want == have, "stalled": stalled}
        print(name, json.dumps(result[name]), flush=True)
    t_idle_after = time.time()
    time.sleep(idle_after)
    t_end = time.time()
    phases["idle_before"] = [console.lines[0][0] if console.lines else t_idle_after, phases["check"][0]]
    phases["idle_after"] = [t_idle_after, t_end]
    stale = drain(link, 1.0)
    result["stale_after"] = stale
    console.stop()
    os.close(link)
    with open(prefix + ".console.log", "wb") as f:
        f.write(bytes(console.raw))
    with open(prefix + ".phases.json", "w") as f:
        json.dump({"phases": phases, "result": result, "line_times": [t for t, _ in console.lines]}, f)
    print("console bytes", len(console.raw), "lines", len(console.lines))
    ok = all(result[k]["equal"] and not result[k]["stalled"] for k in ("check", "sustained"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
