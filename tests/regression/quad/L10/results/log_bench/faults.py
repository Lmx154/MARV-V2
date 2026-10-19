#!/usr/bin/env python3
"""Bench only (onboard logger unit U2b, bench log image MARV_FC3V2_LOG_BENCH): the fault cases L-3 and L-4 on the board.

  faults.py powercut --cycles 3 --out-dir DIR      reset the board through the Debug Probe while it logs, then pull and check the cut file
  faults.py cardoff --out-dir DIR                  `log card off` while Ready, and while Logging
  faults.py full --out-dir DIR                     log until the preallocated file is used up (build the image with a small
                                                   MARV_FC3V2_LOG_BENCH_FILE_BYTES so it takes seconds)

The raw console of every case is written to DIR/<case>*.console.log (the image's bytes as received), the pulled files to
DIR/<case>*.mlog, and the checks print as lines starting `CHECK`. SAFETY: the image links the flight composition and the motor outputs; nothing here arms it, the ESC is
not powered, no props. The reset is tools/target/flash.sh --probe-only (reset halt on the cores, then the chip's watchdog reboot):
the SD card keeps its power, so this cuts the writer, not the card (a real power cut needs the physical runbook in README.md).
"""
from __future__ import annotations

import argparse
import mmap
import os
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[6] / "tools" / "log"))
import log_bench  # noqa: E402
import mlog  # noqa: E402

ROOT = Path(__file__).resolve().parents[6]
READY_TIMEOUT_S = 40.0  # labelled harness value: boot (about 1 s), mount, f_expand and the header write
REENUMERATE_TIMEOUT_S = 30.0  # labelled harness value: the console node returns after the chip reboot
PULL_TIMEOUT_S = 600.0  # labelled harness value: 16 MiB at the measured 0.37 MB/s is 45 s
CUT_WAITS_S = (7.3, 9.9, 12.6)  # labelled harness values: seconds of logging before each reset (arbitrary, unequal, so the cuts land at different phases)
STATE_RE = r"log state=(\w+) trigger=(\d) card=(\w+) file=(\d+) start_sector=(\d+) capacity=(\d+) cursor=(\d+)"
LINE_RE = re.compile(r"log state=[^\n]*\n")
RATE_RE = re.compile(r"rate executions/s=([\d.]+) executions=(\d+) timing_faults=(\d+)")
ATT_RE = re.compile(r"attitude faults=(\d+) timing_faults=(\d+)")


def open_bench(out: Path) -> log_bench.Bench:
    deadline = time.time() + REENUMERATE_TIMEOUT_S
    while True:
        try:
            return log_bench.Bench(log_bench.CONSOLE, log_bench.LINK, out)
        except OSError:
            if time.time() > deadline:
                raise
            time.sleep(0.5)


def wait_ready(b: log_bench.Bench, since: int = 0) -> re.Match[str] | None:
    return b.wait_for(STATE_RE.replace(r"(\w+) trigger", "(Ready) trigger"), READY_TIMEOUT_S, since)


def settle(b: log_bench.Bench, seconds: float) -> int:
    end = time.time() + seconds
    while time.time() < end:
        b.pump(0.2)
    return len(b.text)


def status_lines(b: log_bench.Bench, since: int = 0) -> list[re.Match[str]]:
    return list(re.finditer(STATE_RE, b.text[since:].decode("utf-8", "replace")))


def transitions(b: log_bench.Bench, since: int = 0) -> list[str]:
    text = b.text[since:].decode("utf-8", "replace")
    return [m.group(0)[:90] for m in re.finditer(r"log: \w+ -> \w+ [^\n]*", text)]


def timing(b: log_bench.Bench, since: int = 0, tag: str = "") -> None:
    text = b.text[since:].decode("utf-8", "replace")
    r = [(int(m.group(2)), int(m.group(3))) for m in RATE_RE.finditer(text)]
    a = [(int(m.group(1)), int(m.group(2))) for m in ATT_RE.finditer(text)]
    rates = [float(m.group(1)) for m in RATE_RE.finditer(text)]
    if r and a:
        print(f"CHECK timing{tag}: {len(r)} status lines; rate executions/s min {min(rates)} max {max(rates)}; rate timing_faults "
              f"{sorted({x[1] for x in r})}; attitude faults {sorted({x[0] for x in a})} timing_faults {sorted({x[1] for x in a})}")


def decode_file(path: Path, resync: bool = False) -> mlog.Decoded:
    with path.open("rb") as fh, mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ) as data:
        return mlog.decode(data, resync=resync)


def classify(path: Path) -> dict:
    """Walk every sector of a pulled file: the contiguous valid prefix of the file's session and what lies after it."""
    data = path.read_bytes()
    n = len(data) // mlog.SECTOR
    out = {"sectors": n, "prefix": 0, "after_bad_magic": 0, "after_bad_crc": 0, "after_other_session": 0,
           "after_same_session": 0, "first_bad": None, "first_bad_reason": None, "session": None}
    expected = None
    in_prefix = True
    for i in range(n):
        buf = data[i * mlog.SECTOR:(i + 1) * mlog.SECTOR]
        reason, info = mlog.sector_check(buf)
        if in_prefix and reason is None and out["session"] is None:
            out["session"], expected = info["session"], info["seq"]
        if in_prefix:
            if reason is None and info["session"] == out["session"] and info["seq"] == expected:
                out["prefix"] += 1
                expected += 1
                continue
            in_prefix = False
            out["first_bad"] = i
            out["first_bad_reason"] = reason or ("session" if info["session"] != out["session"] else "sequence")
        if reason == "magic":
            out["after_bad_magic"] += 1
        elif reason is not None:
            out["after_bad_crc"] += 1
        elif info["session"] != out["session"]:
            out["after_other_session"] += 1
        else:
            out["after_same_session"] += 1
    return out


def reset_board(log: Path) -> float:
    env = dict(os.environ, MARV_PROBE_SERIAL=os.environ.get("MARV_PROBE_SERIAL", "900678EF479C3A0E"))
    t0 = time.time()
    r = subprocess.run([str(ROOT / "tools/target/flash.sh"), "--probe-only"], env=env, capture_output=True, text=True, timeout=120)
    log.write_text(r.stdout + r.stderr)
    return t0


def pull_file(b: log_bench.Bench, index: int, out: Path) -> bool:
    return log_bench.pull(b, index, out, PULL_TIMEOUT_S)


def powercut(a: argparse.Namespace) -> int:
    a.out_dir.mkdir(parents=True, exist_ok=True)
    ok_all = True
    b = open_bench(a.out_dir / "powercut.console.log")
    m = wait_ready(b)
    if m is None:
        print("CHECK powercut: the logger never reached Ready", file=sys.stderr)
        return 1
    for cycle in range(1, a.cycles + 1):
        idx = int(m.group(4))
        wait_s = CUT_WAITS_S[(cycle - 1) % len(CUT_WAITS_S)]
        mark = len(b.text)
        b.send("log start")
        s = b.wait_for(r"log start: (bench trigger on|REFUSED[^\n]*)", 10.0, mark)
        print(f"cycle {cycle}: file {idx}: {s.group(0) if s else '(no answer)'}")
        settle(b, wait_s)
        lines = status_lines(b, mark)
        last = lines[-1] if lines else None
        print(f"cycle {cycle}: last status before the reset: " + (last.group(0) if last else "(none)"))
        cursor = int(last.group(7)) if last else 0
        b.console_log.write(f"\n=== host: resetting the board through the probe (cycle {cycle}, {wait_s} s after log start) ===\n".encode())
        t_reset = reset_board(a.out_dir / f"powercut{cycle}.reset.log")
        print(f"cycle {cycle}: reset command took {time.time() - t_reset:.1f} s")
        b.close()
        time.sleep(2.0)
        b = open_bench(a.out_dir / f"powercut{cycle}.after.console.log")
        m = wait_ready(b)
        if m is None:
            print(f"CHECK powercut cycle {cycle}: FAIL the logger did not reach Ready after the reset", file=sys.stderr)
            print(b.text[-2000:].decode("utf-8", "replace"))
            return 1
        new_idx = int(m.group(4))
        print(f"cycle {cycle}: after the reset: {m.group(0)}")
        print(f"cycle {cycle}: transitions after boot: {transitions(b)}")
        out = a.out_dir / f"powercut{cycle}.mlog"
        if not pull_file(b, idx, out):
            print(f"CHECK powercut cycle {cycle}: FAIL the pull of file {idx}", file=sys.stderr)
            ok_all = False
            continue
        c = classify(out)
        strict = decode_file(out)
        resync = decode_file(out, resync=True)
        prefix_bytes = c["prefix"] * mlog.SECTOR
        print(f"cycle {cycle}: file {idx}: {c['sectors']} sectors, valid contiguous prefix {c['prefix']} sectors = {prefix_bytes} B "
              f"(console cursor at the last status line before the cut: {cursor} B)")
        print(f"cycle {cycle}: first sector outside the prefix: {c['first_bad']} ({c['first_bad_reason']}); after it: bad magic "
              f"{c['after_bad_magic']}, bad CRC {c['after_bad_crc']}, valid sectors of another session (stale) {c['after_other_session']}, "
              f"valid sectors of the same session {c['after_same_session']}")
        print(f"cycle {cycle}: strict decode: {strict.sectors_ok} valid sectors, stopped at sector {strict.stopped_at}, "
              f"end record {strict.ended}, problems: {[str(p) for p in strict.problems][:3]}, ends inside a record {strict.ends_inside_record}")
        print(f"cycle {cycle}: resync decode: {resync.sectors_ok} valid sectors, {len(resync.problems)} problems, end record {resync.ended}")
        good = (c["prefix"] >= cursor // mlog.SECTOR and strict.stopped_at == c["first_bad"] and not strict.ended
                and c["after_same_session"] == 0 and new_idx == idx + 1)
        print(f"CHECK powercut cycle {cycle}: {'PASS' if good else 'FAIL'}: prefix >= last console cursor, the strict decoder stops at the first "
              f"bad sector, no valid sector of the cut session after it, the board re-prepared file {new_idx}")
        ok_all = ok_all and good
    b.close()
    return 0 if ok_all else 1


def cardoff(a: argparse.Namespace) -> int:
    a.out_dir.mkdir(parents=True, exist_ok=True)
    b = open_bench(a.out_dir / "cardoff.console.log")
    m = wait_ready(b)
    if m is None:
        print("CHECK cardoff: the logger never reached Ready", file=sys.stderr)
        return 1
    idx0 = int(m.group(4))
    print(f"start: {m.group(0)}")
    base = settle(b, 4.0)
    timing(b, 0, " (baseline, Ready, card on)")

    # Case A: the card goes away while Ready.
    mark = len(b.text)
    b.send("log card off")
    b.wait_for(r"log card: off", 5.0, mark)
    settle(b, 6.0)
    st = status_lines(b, mark)
    print(f"A (Ready, card off, 6 s): state {st[-1].group(1)} card={st[-1].group(3)} (a Ready logger holds its prepared file; it probes nothing)")
    mk2 = len(b.text)
    b.send("log start")
    b.wait_for(r"log start: (bench trigger on|REFUSED[^\n]*)", 10.0, mk2)
    settle(b, 4.0)
    st = status_lines(b, mk2)
    print(f"A after `log start` with the card off: " + st[-1].group(0) + f" | transitions {transitions(b, mk2)}")
    full_line = LINE_RE.findall(b.text[mk2:].decode("utf-8", "replace"))[-1]
    print("A counters: " + full_line.strip()[full_line.index("ring pushed"):])
    mk3 = len(b.text)
    b.send("log stop")
    settle(b, 12.0)
    st = status_lines(b, mk3)
    print(f"A after `log stop` (trigger off, disarmed), 12 s: state {st[-1].group(1)}; transitions {transitions(b, mk3)}; "
          f"probe lines {len(re.findall(r'probe found no card', b.text[mk3:].decode('utf-8', 'replace')))}")
    mk4 = len(b.text)
    b.send("log card on")
    r = wait_ready(b, mk4)
    print(f"A after `log card on`: {r.group(0) if r else 'NOT Ready within the timeout'}")
    timing(b, 0, " (case A, whole run so far)")
    ok = r is not None

    # Case B: the card goes away while Logging.
    mk5 = len(b.text)
    idx_b = int(r.group(4)) if r else 0
    b.send("log start")
    b.wait_for(r"log start: (bench trigger on|REFUSED[^\n]*)", 10.0, mk5)
    settle(b, 5.0)
    pre = status_lines(b, mk5)[-1]
    print(f"B Logging: {pre.group(0)}")
    mk6 = len(b.text)
    b.send("log card off")
    settle(b, 6.0)
    st = status_lines(b, mk6)
    seen_states = sorted({s.group(1) for s in st})
    print(f"B (Logging, card off, trigger still on, 6 s): states {seen_states}; last {st[-1].group(0)}; transitions {transitions(b, mk6)}; "
          f"probe lines {len(re.findall(r'probe found no card', b.text[mk6:].decode('utf-8', 'replace')))} (a probe while the trigger is on would be a defect)")
    full_line = LINE_RE.findall(b.text[mk6:].decode("utf-8", "replace"))[-1]
    print("B counters: " + full_line.strip()[full_line.index("ring pushed"):])
    mk7 = len(b.text)
    b.send("log stop")
    settle(b, 12.0)
    st = status_lines(b, mk7)
    print(f"B after `log stop`, 12 s: state {st[-1].group(1)}; transitions {transitions(b, mk7)}; probe lines "
          f"{len(re.findall(r'probe found no card', b.text[mk7:].decode('utf-8', 'replace')))}")
    mk8 = len(b.text)
    b.send("log card on")
    r2 = wait_ready(b, mk8)
    print(f"B after `log card on`: {r2.group(0) if r2 else 'NOT Ready within the timeout'}")
    timing(b, 0, " (whole run)")
    ok = ok and r2 is not None
    # The file the card went away from is on the card up to the last completed write.
    if r2 is not None:
        out = a.out_dir / "cardoff_B.mlog"
        if pull_file(b, idx_b, out):
            strict = decode_file(out)
            c = classify(out)
            print(f"B file {idx_b}: valid prefix {c['prefix']} sectors, strict decode stopped at {strict.stopped_at}, end record {strict.ended}, "
                  f"imu records {strict.records and len([r for r in strict.records if r['kind'] == 'imu'])}")
        else:
            ok = False
    print(f"CHECK cardoff: {'PASS' if ok else 'FAIL'} (see the lines above for the state machine)")
    b.close()
    return 0 if ok else 1


def full(a: argparse.Namespace) -> int:
    a.out_dir.mkdir(parents=True, exist_ok=True)
    b = open_bench(a.out_dir / "full.console.log")
    m = wait_ready(b)
    if m is None:
        print("CHECK full: the logger never reached Ready", file=sys.stderr)
        return 1
    idx = int(m.group(4))
    cap = int(m.group(6))
    print(f"start: {m.group(0)}")
    settle(b, 2.0)
    mark = len(b.text)
    b.send("log start")
    b.wait_for(r"log start: (bench trigger on|REFUSED[^\n]*)", 10.0, mark)
    t = b.wait_for(r"log: Logging -> (\w+) [^\n]*", a.timeout, mark)
    print(f"transition: {t.group(0)[:120] if t else '(none within the timeout)'}")
    settle(b, 10.0)
    st = status_lines(b, mark)
    print(f"10 s later: {st[-1].group(0)}")
    full_line = LINE_RE.findall(b.text[mark:].decode("utf-8", "replace"))[-1]
    print("counters: " + full_line.strip()[full_line.index("ring pushed"):])
    still_alive = len(re.findall(r"imu dt=", b.text[mark:].decode("utf-8", "replace")))
    print(f"console imu lines since log start: {still_alive} (the board keeps running; no panic)")
    timing(b, mark, " (during and after the fill)")
    mk2 = len(b.text)
    b.send("log stop")
    r = wait_ready(b, mk2)
    print(f"after `log stop`: transitions {transitions(b, mk2)}; {r.group(0) if r else 'NOT Ready'}")
    out = a.out_dir / "full.mlog"
    ok = r is not None and t is not None and t.group(1) == "Full" and pull_file(b, idx, out)
    if ok:
        strict = decode_file(out)
        s = [x for x in strict.records if x["kind"] == "end"]
        print(f"file {idx}: capacity {cap} B; pulled {out.stat().st_size} B; sectors {strict.sectors_ok} valid of {strict.sectors_total}; "
              f"end record {strict.ended} {s[0] if s else None}; problems {[str(p) for p in strict.problems][:3]}")
        ok = strict.clean and bool(s) and s[0]["reason"] == "full"
    print(f"CHECK full: {'PASS' if ok else 'FAIL'}: the logger entered Full, the file closed with an End record (reason full), decoded clean")
    b.close()
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("case", choices=["powercut", "cardoff", "full"])
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=180.0)
    a = ap.parse_args()
    return {"powercut": powercut, "cardoff": cardoff, "full": full}[a.case](a)


if __name__ == "__main__":
    sys.exit(main())
