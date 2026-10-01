"""The CPU count this process can actually use: the affinity mask, capped by the cgroup CPU quota (v2 cpu.max, v1
cfs_quota_us / cfs_period_us), rounded up, at least 1. os.cpu_count() ignores both (inside `docker run --cpus 4` it reports
the host's CPUs)."""

from __future__ import annotations

import math
import os
from pathlib import Path

CGROUP_V2_MAX = Path("/sys/fs/cgroup/cpu.max")
CGROUP_V1_DIRS = (Path("/sys/fs/cgroup/cpu"), Path("/sys/fs/cgroup/cpu,cpuacct"))


def _ceil_ratio(quota: str, period: str) -> int | None:
    """ceil(quota / period) for a positive integer quota and period; None for 'max', -1 or anything unreadable."""
    try:
        q, p = int(quota), int(period)
    except ValueError:
        return None
    return math.ceil(q / p) if q > 0 and p > 0 else None


def _v2_limit(path: Path = CGROUP_V2_MAX) -> int | None:
    try:
        quota, period = path.read_text().split()[:2]
    except (OSError, ValueError):
        return None
    return _ceil_ratio(quota, period)


def _v1_limit(dirs=CGROUP_V1_DIRS) -> int | None:
    for d in dirs:
        try:
            return _ceil_ratio((d / "cpu.cfs_quota_us").read_text().strip(), (d / "cpu.cfs_period_us").read_text().strip())
        except OSError:
            continue
    return None


def usable_cpus() -> int:
    n = len(os.sched_getaffinity(0))
    for limit in (_v2_limit(), _v1_limit()):
        if limit is not None:
            n = min(n, limit)
    return max(n, 1)
