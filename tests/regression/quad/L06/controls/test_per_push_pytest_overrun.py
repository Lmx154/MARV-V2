"""Planted overrun for the per-push pytest file-time guard (tools/ci/pytest_file_time_guard.py). The test sleeps three
times the per-push limit (labelled factor 3, as the CTest control in this directory), so the guard must fail the session.
It is outside every normal collection (the tools step names tests/regression/quad/L0n/tools); ci/run_ci.sh
(per_push_pytest_control) runs it by path with MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE set, so it needs seconds, not the limit."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[5] / "tools" / "ci"))

import pytest_file_time_guard

PLANTED_FACTOR = 3  # labelled factor: overrun several times the limit, as the CTest control (per_push_timeout_planted)


def test_planted_overrun():
    time.sleep(PLANTED_FACTOR * pytest_file_time_guard.limit_seconds())
