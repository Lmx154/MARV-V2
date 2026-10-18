#!/usr/bin/env python3
"""Bench only (ground-segment unit 12a): the console's line layout, a capture against a reference capture.

  fields_diff.py <reference.log> <capture.log>

Every number (decimal, signed, hex, with a fraction) becomes N, the version word after `marv_fc3v2` becomes V; the distinct line
templates of each file are compared, and for the per-second `imu dt=` line the ` | ` separated groups and their `name=` field names
are compared one by one. Prints the templates only the capture has, only the reference has, and the field names added or removed in the
stats line. Exit 0 if the only difference is added fields named on the command line after `--allow`.
"""
import re
import sys


def template(line):
    line = re.sub(r"^(marv_fc3v2 ).*", r"\1V", line)
    return re.sub(r"0x[0-9a-fA-F]+|[+-]?\d+(?:\.\d+)?", "N", line.rstrip("\r\n"))


def lines(path):
    return [l for l in open(path, errors="replace").read().replace("\r", "").split("\n") if l]


def fields(line):
    return re.findall(r"([A-Za-z_/]+)=", line)


def main():
    ref, cap = sys.argv[1], sys.argv[2]
    allow = set(sys.argv[4:]) if len(sys.argv) > 3 and sys.argv[3] == "--allow" else set()
    rl, cl = lines(ref), lines(cap)
    rt, ct = {template(l) for l in rl}, {template(l) for l in cl}
    # The first and last capture lines may be cut by the capture window.
    ct_full = {template(l) for l in cl[1:-1]}
    only_cap = ct_full - rt
    only_ref = {template(l) for l in rl[1:-1]} - ct
    r_stats = next(l for l in reversed(rl[:-1]) if l.startswith("imu dt="))
    c_stats = next(l for l in reversed(cl[:-1]) if l.startswith("imu dt="))
    rf, cf = fields(r_stats), fields(c_stats)
    added = [f for f in cf if f not in rf]
    removed = [f for f in rf if f not in cf]
    order_same = [f for f in cf if f in rf] == [f for f in rf if f in cf]
    print(f"stats line fields: reference {len(rf)}, capture {len(cf)}; added {added}; removed {removed}; common fields in the same order: {order_same}")
    print(f"non-stats line templates only in the capture: {sorted(t for t in only_cap if not t.startswith('imu dt='))}")
    print(f"non-stats line templates only in the reference: {sorted(t for t in only_ref if not t.startswith('imu dt='))}")
    ok = not removed and order_same and set(added) <= allow and not [t for t in only_cap | only_ref if not t.startswith("imu dt=")]
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
