"""The lockstep plugin's flock on the HIL port (ground-segment unit GX-10, invariant H-1: a HIL run owns its board port).

Skipped unless `gz sim --force-version 8` reports 8.x, the host-gz-quadfc build holds libmarv_gz_lockstep.so and a C compiler is
present; the gz CI step fails on a skip. No board is involved: the board's port is a pty (the plugin opens any tty, reads it for the
board's hello and finds none). A pty has no modem lines, so a test-only LD_PRELOAD shim (compiled here) answers the plugin's DTR
ioctls (TIOCMBIC, TIOCMBIS) with success where the pty says ENOTTY; nothing else is changed. The shim is a stand-in for the board's CDC
port, not for the plugin: the plugin under test is the real build.

What is checked, both orders of two openers of one port:
  - The helper (tools/gs/device.py SerialDevice, which flocks its port) holds the port: the plugin's <hil> open is refused with a
    REFUSED line that names the port, and gz stops (an abort); the helper keeps the port.
  - The plugin holds the port (a run waiting for the board's hello; the lock is read from /proc/locks, the port is not opened to look):
    the helper's open is refused (DeviceError), and so is a plain flock(2); after the run is gone the helper opens it (the control: it was
    the plugin's lock).
"""

import asyncio
import fcntl
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
for sub in ("sim", "card", "target"):
    sys.path.insert(0, str(ROOT / "tools" / sub))
sys.path.insert(0, str(ROOT / "tools" / "gs"))
import device  # noqa: E402
import hil  # noqa: E402
import hil_compare  # noqa: E402
import run_hil_s4  # noqa: E402
import run_scenario  # noqa: E402

BUILD = ROOT / "build" / "host-gz-quadfc"
PLUGIN_DIR = BUILD / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"
PARAM_DEFAULTS = BUILD / hil_compare.PARAM_DEFAULTS_REL

pytestmark = pytest.mark.skipif(
    not (PLUGIN.exists() and PARAM_DEFAULTS.exists() and run_scenario.gz8_available() and shutil.which("cc")),
    reason="gz-sim 8, the host-gz-quadfc build of libmarv_gz_lockstep.so or a C compiler is absent")

# Labelled harness values, bounding hung processes only (they decide no result).
REFUSAL_WAIT_S = 180  # gz starts, loads the plugin and aborts on the refusal
LOCK_WAIT_S = 120  # gz starts and the plugin reaches its open of the port
ITERATIONS = 100  # the run never gets to its first tick: it waits for the board's hello (or is refused)

SHIM = r"""
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <stdarg.h>
#include <sys/ioctl.h>
int ioctl(int fd, unsigned long req, ...) {
  static int (*real)(int, unsigned long, void*) = 0;
  if (!real) real = (int (*)(int, unsigned long, void*))dlsym(RTLD_NEXT, "ioctl");
  va_list ap;
  va_start(ap, req);
  void* arg = va_arg(ap, void*);
  va_end(ap);
  int r = real(fd, req, arg);
  if (r != 0 && (req == TIOCMBIC || req == TIOCMBIS) && (errno == ENOTTY || errno == EINVAL)) return 0;
  return r;
}
"""


@pytest.fixture(scope="module")
def shim(tmp_path_factory):
    d = tmp_path_factory.mktemp("shim")
    src, so = d / "dtr_shim.c", d / "dtr_shim.so"
    src.write_text(SHIM)
    subprocess.run(["cc", "-shared", "-fPIC", "-O1", "-o", str(so), str(src), "-ldl"], check=True)
    return so


@pytest.fixture
def pty_port():
    master, slave = os.openpty()
    yield os.ttyname(slave)
    os.close(master)
    os.close(slave)


def start_gz(tmp_path, port, shim_so):
    values_crc = hil_compare.values_crc(hil_compare.load_table(PARAM_DEFAULTS))
    stem, num, den, seed = run_hil_s4.world(tmp_path, port, run_hil_s4.SAMPLES_PER_RECORD, run_hil_s4.DELAY_TICKS, values_crc)
    cmd = [*run_scenario.GZ_ARGS, "-s", "-r", "--iterations", str(ITERATIONS), "--seed", str(seed), f"{stem}.sdf"]
    err = open(tmp_path / "gz.stderr.txt", "w", encoding="utf-8")
    proc = subprocess.Popen(cmd, env=run_scenario.gz_env(PLUGIN_DIR, {"LD_PRELOAD": str(shim_so)}), stdout=subprocess.DEVNULL, stderr=err,
                            start_new_session=True)
    return proc, err


def stop_gz(proc, err):
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait()
    err.close()


def stderr_of(tmp_path):
    return (tmp_path / "gz.stderr.txt").read_text(encoding="utf-8", errors="replace")


async def try_helper_open(port):
    dev = device.SerialDevice(port, 115200, lambda b: None, lambda r: None)
    dev.close()


def test_the_plugin_is_refused_by_a_port_the_helper_holds(tmp_path, shim, pty_port):
    async def main():
        dev = device.SerialDevice(pty_port, 115200, lambda b: None, lambda r: None)  # the helper's connection, flock included
        try:
            proc, err = start_gz(tmp_path, pty_port, shim)
            try:
                code = await asyncio.to_thread(proc.wait, REFUSAL_WAIT_S)
            finally:
                stop_gz(proc, err)
            return code, hil.flock_holders(pty_port)
        finally:
            dev.close()
    code, holders = asyncio.run(main())
    stderr = stderr_of(tmp_path)
    assert code != 0 and "marv_gz_lockstep: REFUSED: <hil>: the board's port " + pty_port + " is in use by another process" in stderr, stderr[-2000:]
    assert holders == [os.getpid()]  # the helper still held the port when the plugin was refused


def test_the_helper_and_a_plain_flock_are_refused_by_a_port_the_plugin_holds_and_get_it_when_the_run_is_gone(tmp_path, shim, pty_port):
    proc, err = start_gz(tmp_path, pty_port, shim)
    try:
        deadline = time.monotonic() + LOCK_WAIT_S
        while not hil.flock_holders(pty_port):
            assert proc.poll() is None, "gz ended before the plugin took the port:\n" + stderr_of(tmp_path)[-2000:]
            assert time.monotonic() < deadline, "the plugin did not take the port"
            time.sleep(0.05)
        assert hil.flock_holders(pty_port) != [os.getpid()]  # the holder is the gz process
        assert "REFUSED" not in stderr_of(tmp_path)
        with pytest.raises(device.DeviceError, match="cannot configure"):
            asyncio.run(try_helper_open(pty_port))
        other = os.open(pty_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            with pytest.raises(BlockingIOError):
                fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)
        finally:
            os.close(other)
        assert proc.poll() is None  # the refused openers did not disturb the run
    finally:
        stop_gz(proc, err)
    assert hil.flock_holders(pty_port) == []
    asyncio.run(try_helper_open(pty_port))  # control: the lock was the plugin's
