#!/usr/bin/env python3
"""Post-link check of invariant L-5 (core EMB-6, "The RAM log buffer is at least the log rate x the card's worst measured SD
write stall") for the FC3v2 bench log image (MARV_FC3V2_LOG_BENCH, unit U2; serena memory
quad-m4/architect-onboard-logger-2026-10-18) and the flight image with the logger (MARV_FC3V2_FLIGHT_LOG, unit U4-4; serena
memory quad-m4/architect-logger-u4-2026-10-20).

The rule is fw/log/include/marv/log/budget.hpp's ring_covers_stall, evaluated on the image's own parameter set and ring:
  imu_hz        = 1e6 x tick_period_den / tick_period_num_us   (one Imu record per tick)
  rate_tick_hz  = imu_hz / rate_loop_divisor                     (one RateTick record per rate tick)
  manual_hz     = log_manual_rate_hz                             (one ManualControl record per RC frame)
  bytes_per_s   = imu_hz x Imu bytes + rate_tick_hz x RateTick bytes + manual_hz x ManualControl bytes
  pass          iff (ring cells - 1) x cell bytes >= bytes_per_s x sd_log_write_stall_max_s   (the ring keeps one cell free)
The record sizes are the decoder's (tools/log/mlog.py RING_SIZES, which the U1 round-trip tests hold equal to format.hpp); a
cell is a RateTick record's size (format.hpp: "a rate-tick record is one cell"). The parameter values are the set's
params_manifest.json defaults. The ring's bytes are the size of the image's symbol marv::hal_target::g_log_ring_cells
(fw/hal/target/src/log_tick.cpp), read with nm, or given with --ring-bytes. With --arena-symbol the ring is the one the arena
logger makes of the disarmed arena (fw/log/src/arena_logger.cpp, take_lease): the symbol's bytes (its address and size, nm) less
the --staging-bytes at its start, from the first cell boundary after them, in whole cells.

Usage: check_log_ring.py --manifest <params_manifest.json>
                         (--elf <image> --nm <nm> [--arena-symbol <mangled name> --staging-bytes N] | --ring-bytes N)
                         [--delete-on-failure | --warn]
--warn (the measurement build MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN only, invariant L-5a): a ring that does not cover the stall is printed
as a warning (LOG-RING: WARN) and the exit status is 0; the image is kept.
Exit status: 0 the ring covers the stall (or --warn); 1 it does not (lines tagged LOG-RING; with --delete-on-failure the ELF is
removed); 2 usage, a missing parameter, a missing symbol or a file that cannot be read.
"""

from __future__ import annotations

import argparse
import json
import struct
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "log"))
import mlog  # noqa: E402

RING_SYMBOL = "_ZN4marv10hal_target16g_log_ring_cellsE"  # marv::hal_target::g_log_ring_cells
CELL_BYTES = mlog.RING_SIZES[mlog.K_RATE]
PARAMS = ("tick_period_num_us", "tick_period_den", "rate_loop_divisor", "log_manual_rate_hz", "sd_log_write_stall_max_s")
MICROSECONDS_PER_SECOND = 1e6


def param_values(manifest: Path) -> dict[str, float]:
    entries = json.loads(manifest.read_text())["params"]
    out = {}
    for name in PARAMS:
        if name not in entries:
            raise KeyError(f"{manifest} has no parameter {name}")
        e = entries[name]
        bits = int(e["default_bits"], 16)
        out[name] = struct.unpack("<f", struct.pack("<I", bits))[0] if e["type"] == "f32" else struct.unpack(
            "<i", struct.pack("<I", bits))[0]
    return out


def symbol_of(elf: Path, nm: str, symbol: str) -> tuple[int, int]:
    """(address, size) of the one definition of `symbol` in `elf`."""
    out = subprocess.run([nm, "-S", "--defined-only", str(elf)], capture_output=True, text=True, check=True).stdout
    found = [(int(p[0], 16), int(p[1], 16)) for p in (line.split() for line in out.splitlines()) if len(p) == 4 and p[3] == symbol]
    if len(found) != 1:
        raise KeyError(f"{elf} defines {len(found)} symbols named {symbol}, not one")
    return found[0]


def ring_bytes_of(elf: Path, nm: str) -> int:
    try:
        return symbol_of(elf, nm, RING_SYMBOL)[1]
    except KeyError as e:
        raise KeyError(f"{e} (marv::hal_target::g_log_ring_cells)") from None


def arena_ring_bytes(address: int, size: int, staging: int) -> int:
    """The ring the arena logger makes of an arena at `address` of `size` bytes: after the staging, from the first cell boundary."""
    start = -(-(address + staging) // CELL_BYTES) * CELL_BYTES
    end = address + size
    return (end - start) // CELL_BYTES * CELL_BYTES if end > start else 0


def requirement(p: dict[str, float]) -> tuple[float, float]:
    """(bytes per second of the fast stream, the bytes the ring must hold)."""
    imu_hz = MICROSECONDS_PER_SECOND * p["tick_period_den"] / p["tick_period_num_us"]
    rate_hz = imu_hz / p["rate_loop_divisor"]
    rate = (imu_hz * mlog.RING_SIZES[mlog.K_IMU] + rate_hz * mlog.RING_SIZES[mlog.K_RATE]
            + p["log_manual_rate_hz"] * mlog.RING_SIZES[mlog.K_MANUAL])
    return rate, rate * p["sd_log_write_stall_max_s"]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--elf", type=Path)
    ap.add_argument("--nm", default="nm")
    ap.add_argument("--ring-bytes", type=int)
    ap.add_argument("--arena-symbol")
    ap.add_argument("--staging-bytes", type=int)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--delete-on-failure", action="store_true")
    mode.add_argument("--warn", action="store_true")
    a = ap.parse_args(argv)
    if (a.elf is None) == (a.ring_bytes is None):
        print("LOG-RING: give exactly one of --elf and --ring-bytes", file=sys.stderr)
        return 2
    if (a.arena_symbol is None) != (a.staging_bytes is None) or (a.arena_symbol is not None and a.elf is None):
        print("LOG-RING: --arena-symbol and --staging-bytes go together, with --elf", file=sys.stderr)
        return 2
    try:
        p = param_values(a.manifest)
        if a.ring_bytes is not None:
            ring = a.ring_bytes
        elif a.arena_symbol is not None:
            address, size = symbol_of(a.elf, a.nm, a.arena_symbol)
            ring = arena_ring_bytes(address, size, a.staging_bytes)
        else:
            ring = ring_bytes_of(a.elf, a.nm)
    except (OSError, KeyError, ValueError, subprocess.CalledProcessError) as e:
        print(f"LOG-RING: {e}", file=sys.stderr)
        return 2
    rate, need = requirement(p)
    cells = ring // CELL_BYTES
    have = (cells - 1) * CELL_BYTES if cells > 0 else 0
    margin_s = (have - need) / rate
    what = (f"ring {ring} B ({cells} cells, {have} B usable) vs fast stream {rate:.1f} B/s x stall "
            f"{p['sd_log_write_stall_max_s']:.6g} s = {need:.1f} B (margin {margin_s * 1e3:+.1f} ms)")
    if have >= need:
        print(f"check_log_ring: L-5 holds: {what}")
        return 0
    if a.warn:
        print(f"LOG-RING: WARN: L-5 does not hold (core EMB-6 is open in this measurement build, invariant L-5a): {what}")
        return 0
    print(f"LOG-RING: L-5 FAILS (core EMB-6): {what}")
    if a.delete_on_failure and a.elf is not None:
        a.elf.unlink(missing_ok=True)
        print(f"LOG-RING: removed {a.elf}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
