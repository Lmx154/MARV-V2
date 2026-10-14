"""Planted refusals of the plugin's <vertical_source> element (decision 0020 section 1, item c, 'Plugin': refused unless
<attitude_source>truth</attitude_source> is present and the SIL library has MARV_SIL_TRUTH_VERTICAL).

Skipped only as conftest.py says. Each planted world is a good world with one change and must be refused (non-zero exit,
`marv_gz_lockstep: REFUSED: <reason>` on stderr, no log):
  - the element without <attitude_source>, on the host-gz-l11 build;
  - the element on the host-gz-l5 build, whose SIL library (marv_sil_l5_attitude_scripted, TRUTH_STATE only) has no
    marv_truth_vertical_set, so the plugin is built without MARV_GZ_TRUTH_VERTICAL.
Negative controls: the good world of each, the same text without the planted change, runs and logs (on the host-gz-l11
build with both elements; on the host-gz-l5 build without the element).
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import l11_gz  # noqa: E402

import lockstep_log  # noqa: E402
import run_scenario  # noqa: E402

CASES = [
    ("without_attitude_source", dict(l11=True, attitude=False), dict(l11=True), l11_gz.PLUGIN_DIR_L11,
     l11_gz.NEEDS_ATTITUDE),
    ("on_the_l5_build", dict(l11=False), dict(l11=False, vertical=False), l11_gz.PLUGIN_DIR_L5, l11_gz.NEEDS_LIBRARY),
]


@pytest.mark.parametrize("name,planted,good,plugin_dir,reason", CASES, ids=[c[0] for c in CASES])
def test_planted_vertical_element_is_refused(tmp_path, name, planted, good, plugin_dir, reason):
    assert run_scenario.plugin_present(plugin_dir), f"{plugin_dir}: the plugin is not built"
    log = tmp_path / "run.bin"
    text = l11_gz.world(tmp_path, log, **planted)
    assert l11_gz.VERTICAL_ELEMENT in text
    r = l11_gz.run(text, tmp_path, plugin_dir)
    assert r.returncode != 0, "the run was not refused"
    assert l11_gz.REFUSED + reason in r.stderr, run_scenario.stderr_tail(r.stderr)
    assert not log.exists(), "a refused run wrote a log"


@pytest.mark.parametrize("name,planted,good,plugin_dir,reason", CASES, ids=[c[0] for c in CASES])
def test_control_the_good_world_runs_and_logs(tmp_path, name, planted, good, plugin_dir, reason):
    log = tmp_path / "run.bin"
    r = l11_gz.run(l11_gz.world(tmp_path, log, **good), tmp_path, plugin_dir)
    assert r.returncode == 0 and "REFUSED" not in r.stderr, run_scenario.stderr_tail(r.stderr)
    d = lockstep_log.read(log)
    assert run_scenario.complete_trailer(d, l11_gz.ITERATIONS, l11_gz.M)
    assert all(("vertical" in t) == good.get("vertical", True) for t in d["ticks"])
