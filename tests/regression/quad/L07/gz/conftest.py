"""Shared setup of the L7 gz suite (quad spec 4.0 M3, decision 0042 item 4).

Every test of this directory is skipped only when `gz sim --force-version 8` does not report 8.x or the host-gz-l7 build of
libmarv_gz_lockstep.so is absent (`cmake --preset host-gz-l7 && cmake --build --preset host-gz-l7`); the gz CI step fails on a
skip. Helpers: l7_gz.py, next to this file.
"""

import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import l7_gz  # noqa: E402

REASON = "gz-sim 8 or the host-gz-l7 build of libmarv_gz_lockstep.so is absent"


def pytest_collection_modifyitems(config, items):
    if l7_gz.run_scenario.plugin_present(l7_gz.PLUGIN_DIR) and l7_gz.run_scenario.gz8_available():
        return
    skip = pytest.mark.skip(reason=REASON)
    for item in items:
        if HERE in Path(str(item.fspath)).resolve().parents:
            item.add_marker(skip)
