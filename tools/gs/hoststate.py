"""The host's state for the GCS status bar: load average, free memory and the heavy-lock holder.

The heavy lock is ONE flock(2) lock on one file, taken by ci/heavy.sh (which ci/dev_check.sh and ci/local_ci.sh run under) and
checked here: the helper refuses heavy work (a run) while someone holds it. The file is HEAVY_LOCK_DEFAULT unless $MARV_HEAVY_LOCK
names another; the two must agree, so a test or a second checkout that sets one sets the other. The holder, while the lock is
held, is the one line ci/heavy.sh writes into the file ("pid=<pid> user=<user> since=<UTC> cmd=<command>"). This module only
looks: it never takes the lock for longer than one non-blocking probe and never writes the file.
"""

from __future__ import annotations

import errno
import fcntl
import os
from pathlib import Path

HEAVY_LOCK_DEFAULT = Path("/tmp/marv-heavy.lock")  # the same path as ci/heavy.sh's default; a fixed path, so every checkout and shell agrees
HOLDER_READ_MAX = 512  # implementation constant: the holder line is far shorter; a longer file is truncated to this
MEMINFO = Path("/proc/meminfo")


def heavy_lock_path() -> Path:
    return Path(os.environ.get("MARV_HEAVY_LOCK") or HEAVY_LOCK_DEFAULT)


def heavy_lock_state(path: Path | None = None) -> dict:
    """{"held": bool, "holder": str | None, "path": str}. A probe takes a shared lock without blocking and drops it at once;
    a file that does not exist, or that cannot be opened, is not held."""
    path = Path(path) if path is not None else heavy_lock_path()
    state = {"held": False, "holder": None, "path": str(path)}
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return state
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except OSError as e:
            if e.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
                return state
            state["held"] = True
            text = os.pread(fd, HOLDER_READ_MAX, 0).decode("utf-8", "replace").strip()
            state["holder"] = text or None
        else:
            fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)
    return state


def meminfo() -> dict:
    """{"mem_total_bytes", "mem_available_bytes"} from /proc/meminfo (kB there); None values where it cannot be read."""
    out = {"mem_total_bytes": None, "mem_available_bytes": None}
    try:
        lines = MEMINFO.read_text(encoding="ascii").splitlines()
    except OSError:
        return out
    keys = {"MemTotal:": "mem_total_bytes", "MemAvailable:": "mem_available_bytes"}
    for line in lines:
        fields = line.split()
        if len(fields) >= 3 and fields[0] in keys and fields[2] == "kB":
            out[keys[fields[0]]] = int(fields[1]) * 1024  # 1 kB = 1024 B in /proc/meminfo
    return out


def host_state(lock_path: Path | None = None) -> dict:
    try:
        load = list(os.getloadavg())
    except OSError:
        load = None
    return {"loadavg": load, "cpus": os.cpu_count(), **meminfo(), "heavy_lock": heavy_lock_state(lock_path)}
