"""Invariant L-5 of the onboard flight logger (core EMB-6): tools/ci/check_log_ring.py fails a ring smaller than the fast stream x the
stall budget entry, with its negative controls. The manifest here is a scenario input in the generated manifest's format (the values
are the FC3v2 log set's: 7500 Hz ticks, rate divisor 2, 125 receiver frames/s, stall 0.809 s); the record sizes are the decoder's."""

from __future__ import annotations

import json
import math
import stat
import struct
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CHECK = ROOT / "tools" / "ci" / "check_log_ring.py"
sys.path.insert(0, str(ROOT / "tools" / "log"))
import mlog  # noqa: E402

CELL = mlog.RING_SIZES[mlog.K_RATE]
SYMBOL = "_ZN4marv10hal_target16g_log_ring_cellsE"


def f32_bits(v: float) -> str:
    return f"0x{struct.unpack('<I', struct.pack('<f', v))[0]:08x}"


def i32_bits(v: int) -> str:
    return f"0x{struct.unpack('<I', struct.pack('<i', v))[0]:08x}"


def manifest(tmp_path: Path, *, stall_s: float = 0.809, drop: str | None = None) -> Path:
    # tick period 1953125 us / 14649 ticks = 7500 Hz (tick_period_num_us / tick_period_den, as the generated set)
    params = {
        "tick_period_num_us": {"type": "i32", "default_bits": i32_bits(1953125)},
        "tick_period_den": {"type": "i32", "default_bits": i32_bits(14649)},
        "rate_loop_divisor": {"type": "i32", "default_bits": i32_bits(2)},
        "log_manual_rate_hz": {"type": "f32", "default_bits": f32_bits(125.0)},
        "sd_log_write_stall_max_s": {"type": "f32", "default_bits": f32_bits(stall_s)},
    }
    if drop is not None:
        del params[drop]
    path = tmp_path / "params_manifest.json"
    path.write_text(json.dumps({"params": params}))
    return path


def needed_bytes(stall_s: float) -> float:
    imu_hz = 1e6 * 14649 / 1953125
    rate = (imu_hz * mlog.RING_SIZES[mlog.K_IMU] + imu_hz / 2 * mlog.RING_SIZES[mlog.K_RATE]
            + 125 * mlog.RING_SIZES[mlog.K_MANUAL])
    return rate * struct.unpack("<f", struct.pack("<f", stall_s))[0]


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(CHECK), *args], capture_output=True, text=True, check=False)


def smallest_ring_cells(stall_s: float) -> int:
    """The fewest cells that hold the stream for the stall: (cells - 1) x cell bytes >= need."""
    return math.ceil(needed_bytes(stall_s) / CELL) + 1


def test_the_design_ring_holds_the_stall():
    # 256 KiB is the bench ring (architect ruling item 3)
    assert smallest_ring_cells(0.809) * CELL <= 262144


def test_a_ring_of_the_smallest_size_passes_and_one_cell_less_fails(tmp_path):
    m = str(manifest(tmp_path))
    cells = smallest_ring_cells(0.809)
    ok = run("--manifest", m, "--ring-bytes", str(cells * CELL))
    assert ok.returncode == 0, ok.stdout + ok.stderr
    assert "L-5 holds" in ok.stdout
    bad = run("--manifest", m, "--ring-bytes", str((cells - 1) * CELL))
    assert bad.returncode == 1
    assert "LOG-RING: L-5 FAILS" in bad.stdout


def test_negative_control_a_ring_too_small_for_a_longer_stall_fails(tmp_path):
    ring = 262144
    assert run("--manifest", str(manifest(tmp_path)), "--ring-bytes", str(ring)).returncode == 0
    longer = tmp_path / "longer"
    longer.mkdir()
    r = run("--manifest", str(manifest(longer, stall_s=1.0)), "--ring-bytes", str(ring))
    assert r.returncode == 1
    assert "L-5 FAILS" in r.stdout


def test_an_empty_ring_fails(tmp_path):
    assert run("--manifest", str(manifest(tmp_path)), "--ring-bytes", "0").returncode == 1


def write_fake_nm(tmp_path: Path, size: int) -> Path:
    nm = tmp_path / "fake_nm"
    nm.write_text(f"#!/bin/sh\necho '20000000 {size:08x} B {SYMBOL}'\n")
    nm.chmod(nm.stat().st_mode | stat.S_IXUSR)
    return nm


def test_the_image_path_reads_the_symbol_size_and_deletes_a_failing_image(tmp_path):
    m = str(manifest(tmp_path))
    elf = tmp_path / "image.elf"
    elf.write_bytes(b"x")
    nm = write_fake_nm(tmp_path, smallest_ring_cells(0.809) * CELL)
    ok = run("--manifest", m, "--elf", str(elf), "--nm", str(nm), "--delete-on-failure")
    assert ok.returncode == 0, ok.stdout + ok.stderr
    assert elf.exists()
    nm = write_fake_nm(tmp_path, (smallest_ring_cells(0.809) - 1) * CELL)
    bad = run("--manifest", m, "--elf", str(elf), "--nm", str(nm), "--delete-on-failure")
    assert bad.returncode == 1
    assert not elf.exists()


def test_usage_and_input_errors_are_not_a_pass(tmp_path):
    m = str(manifest(tmp_path))
    assert run("--manifest", m).returncode == 2
    assert run("--manifest", m, "--ring-bytes", "1", "--elf", "x").returncode == 2
    missing = tmp_path / "missing"
    missing.mkdir()
    r = run("--manifest", str(manifest(missing, drop="sd_log_write_stall_max_s")), "--ring-bytes", "262144")
    assert r.returncode == 2
    assert "sd_log_write_stall_max_s" in r.stderr
    elf = tmp_path / "nosym.elf"
    elf.write_bytes(b"x")
    nosym = tmp_path / "nm_nosym"
    nosym.write_text("#!/bin/sh\necho '20000000 00000010 B other'\n")
    nosym.chmod(nosym.stat().st_mode | stat.S_IXUSR)
    assert run("--manifest", m, "--elf", str(elf), "--nm", str(nosym)).returncode == 2
