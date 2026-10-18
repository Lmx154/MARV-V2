#!/usr/bin/env python3
"""Bench only (ground-segment unit 12a): USB-initiated resets against the reset gate.

  reset_refusal.py <out_prefix> <expect>      expect = refused | honoured

Reads the console (if00) throughout. Actions, one at a time, each followed by 4 s: `picotool info` (the device must be found),
`picotool reboot` (flash boot), `picotool reboot -u` (BOOTSEL), a 1200-baud line coding on if00 (a second open of the console
port, set to 1200 baud, then back to 115200), a 1200-baud open and close of the link port (if02). After each action the
`usb_reset_refused` field of the newest stats line is read, with the IMU time `t=` and sample count.
  expect refused (no grant, the flight image): the board must keep running -- `t` and the sample count keep increasing across the whole run, the
  console port never goes away -- and the counter must increase by one for each reset request that reached the board (the two picotool reboots,
  unless picotool could not open the device, and the two 1200-baud line codings; `picotool info` and the 115200 restore do not count).
  expect honoured (the control build whose gate always returns true, never committed as firmware): the flash-boot request must
  restart the board (the console port goes away and returns, `t` restarts).
Raw console is kept in <out_prefix>.console.log; the per-action results go to <out_prefix>.json. Exit 0 if the expectation holds.
"""
import json
import os
import re
import subprocess
import sys
import termios
import time

from usb_common import CONSOLE, LINK, ConsoleReader, open_raw, set_baud


def snapshot(console):
    m = console.last(r"usb_reset_refused=(\d+)")
    t = console.last(r"samples/s=[\d.]+ ticks/s=[\d.]+ .*? t=(\d+) gyro")
    n = console.last(r"total samples=(\d+)")
    return {"usb_reset_refused": int(m.group(1)) if m else None, "imu_t_us": int(t.group(1)) if t else None,
            "total_samples": int(n.group(1)) if n else None, "console_port_present": os.path.exists(CONSOLE),
            "lines": len(console.lines)}


def run(cmd):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=40)
        return {"cmd": " ".join(cmd), "rc": p.returncode, "out": (p.stdout + p.stderr).strip()[-600:]}
    except subprocess.TimeoutExpired:
        return {"cmd": " ".join(cmd), "rc": "timeout", "out": ""}


def baud_touch(dev, label):
    fd = open_raw(dev, termios.B1200)
    time.sleep(1.5)
    set_baud(fd, termios.B115200)
    time.sleep(0.5)
    os.close(fd)
    return {"cmd": f"1200 baud open/close {label}", "rc": 0, "out": ""}


def main():
    prefix, expect = sys.argv[1], sys.argv[2]
    console = ConsoleReader()
    console.start()
    time.sleep(4)
    steps = [("start", None, snapshot(console))]
    actions = [("picotool info", lambda: run(["picotool", "info"])),
               ("picotool reboot", lambda: run(["picotool", "reboot"])),
               ("picotool reboot -u", lambda: run(["picotool", "reboot", "-u"])),
               ("1200 baud on if00", lambda: baud_touch(CONSOLE, "if00")),
               ("1200 baud on if02", lambda: baud_touch(LINK, "if02"))]
    for name, fn in actions:
        res = fn()
        time.sleep(4)
        steps.append((name, res, snapshot(console)))
        print(name, "->", json.dumps(res)[:300], json.dumps(steps[-1][2]), flush=True)
        if expect == "honoured" and name == "picotool reboot":
            break
    console.stop()
    with open(prefix + ".console.log", "wb") as f:
        f.write(bytes(console.raw))
    with open(prefix + ".json", "w") as f:
        json.dump(steps, f, indent=1)
    ts = [int(x) for x in re.findall(r"samples/s=[\d.]+ ticks/s=[\d.]+ .*? t=(\d+) gyro", bytes(console.raw).decode(errors="replace"))]
    mono = all(b > a for a, b in zip(ts, ts[1:]))
    print("imu t monotonic across the run:", mono, "lines:", len(ts))
    counters = [s[2]["usb_reset_refused"] for s in steps]
    print("counter after each step:", counters)
    if expect == "refused":
        # A picotool run that could not open the device (usbfs node root-only, "unable to connect") sent no request: no refusal.
        want = [counters[0]]
        for name, res, _ in steps[1:]:
            reached = not (res and "unable to connect" in res["out"])
            counts = ("reboot" in name or "1200" in name) and reached
            want.append(want[-1] + (1 if counts else 0))
        ok = mono and counters == want and all(s[2]["console_port_present"] for s in steps)
        print("expected counters", want, "->", "PASS" if ok else "FAIL")
    else:
        ok = not mono or not steps[-1][2]["console_port_present"] or counters[-1] != counters[0]
        print("honoured: board restarted" if not mono else "honoured: NOT restarted -> FAIL")
        ok = not mono
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
