"""Shared setup of the L5 gz suite (quad spec 4 L5 T4; docs/decisions/0006 F).

Every test of this directory is skipped only when `gz sim --force-version 8` does not report 8.x or the host-gz-l5 build of
libmarv_gz_lockstep.so is absent (`cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5`); the gz CI step fails
on a skip. Every run is truth-fed, perfect-model (tools/sim/run_l5.py): not a validation run.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import run_l5  # noqa: E402
import run_scenario  # noqa: E402

HERE = Path(__file__).resolve().parent
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05"
PLUGIN_DIR = run_l5.DEFAULT_PLUGIN_DIR
T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference"
REASON = "gz-sim 8 or the host-gz-l5 build of libmarv_gz_lockstep.so is absent"


def pytest_collection_modifyitems(config, items):
    if run_scenario.plugin_present(PLUGIN_DIR) and run_scenario.gz8_available():
        return
    skip = pytest.mark.skip(reason=REASON)
    for item in items:
        if HERE in Path(str(item.fspath)).resolve().parents:
            item.add_marker(skip)


@pytest.fixture(scope="session")
def live_params():
    """The plugin build's parameter table (name -> value), the values the firmware in the plugin runs with."""
    _, defaults = run_l5.build_parameters(PLUGIN_DIR)
    return run_l5.l4.read_param_defaults(defaults)
