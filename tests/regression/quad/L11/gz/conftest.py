"""Shared setup of the L11 gz suite (decision 0020 section 1, item c, and section 2, I1).

Every test of this directory is skipped only when `gz sim --force-version 8` does not report 8.x or the host-gz-l11 build of
libmarv_gz_lockstep.so is absent (`cmake --preset host-gz-l11 && cmake --build --preset host-gz-l11`); the gz CI step fails
on a skip. The tests that run on the host-gz-l5 build (a refusal and the I1 guard) fail, not skip, when that build is absent
(`cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5`): CI group l11 builds both. Every run is truth-fed,
perfect-model: not a validation run. Helpers: l11_gz.py, next to this file.
"""

import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import l11_gz  # noqa: E402

REASON = "gz-sim 8 or the host-gz-l11 build of libmarv_gz_lockstep.so is absent"


def pytest_collection_modifyitems(config, items):
    if l11_gz.run_scenario.plugin_present(l11_gz.PLUGIN_DIR_L11) and l11_gz.run_scenario.gz8_available():
        return
    skip = pytest.mark.skip(reason=REASON)
    for item in items:
        if HERE in Path(str(item.fspath)).resolve().parents:
            item.add_marker(skip)
