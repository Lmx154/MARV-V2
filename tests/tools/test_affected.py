"""ci/affected.py, the blast-radius selector: the fail-safe, the full roots, the ignore list, one real mapping and the
manifest check."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ci"))

import affected  # noqa: E402

SMALL_MAP = """\
full_roots: [core/**]
ignore: [docs/**]
rules:
  - paths: [lib/a/**]
    ctest_labels: [L01]
    run_ci_steps: [step_a]
    pytest_paths: []
    gz_groups: []
    why: lib a
"""

SMALL_UNIVERSE = {"ctest_labels": ["L01", "L02"], "run_ci_steps": ["step_a", "step_b"], "pytest_paths": ["t"],
                  "gz_groups": ["l2"]}


def small_select(tmp_path, changed):
    path = tmp_path / "map.yaml"
    path.write_text(SMALL_MAP)
    return affected.select(changed, affected.load_map(path), SMALL_UNIVERSE, base="abc")


def real_select(changed):
    mapping = affected.load_map()
    universe = affected.repo_universe()
    universe["pytest_paths"] = affected.map_pytest_paths(mapping)
    return affected.select(changed, mapping, universe)


def test_unmapped_path_selects_full(tmp_path):
    r = small_select(tmp_path, ["lib/a/x.cpp", "somewhere/new_file.txt"])
    assert r["full"] is True
    assert r["full_reasons"] == ["somewhere/new_file.txt: unmapped"]
    assert r["ctest_labels"] == ["L01", "L02"] and r["run_ci_steps"] == ["step_a", "step_b"]
    assert r["gz_groups"] == ["l2"] and r["pytest_paths"] == ["t"]


def test_full_root_path_selects_full(tmp_path):
    r = small_select(tmp_path, ["core/deep/x.hpp"])
    assert r["full"] is True
    assert r["full_reasons"] == ["core/deep/x.hpp: core/**"]
    assert r["ctest_labels"] == ["L01", "L02"]


def test_docs_only_change_selects_nothing(tmp_path):
    r = small_select(tmp_path, ["docs/handoff.md", "docs/decisions/0001-x.md"])
    assert r["full"] is False and r["full_reasons"] == []
    assert [r[k] for k in ("ctest_labels", "run_ci_steps", "pytest_paths", "gz_groups")] == [[], [], [], []]
    assert real_select(["docs/handoff.md", "LICENSE"])["full"] is False


def test_fw_vertical_selects_l11_and_not_l02():
    r = real_select(["fw/vertical/include/marv/vertical/vertical_law.hpp"])
    assert r["full"] is False
    assert "L11" in r["ctest_labels"] and "L02" not in r["ctest_labels"]
    assert "l11" in r["gz_groups"] and "l2" not in r["gz_groups"]


def test_python_change_selects_test_files_and_other_changes_select_directories():
    mapping = affected.load_map()
    universe = affected.repo_universe()
    universe["pytest_paths"] = affected.map_pytest_paths(mapping)
    index = affected.PythonIndex(ROOT, affected.repo_files(ROOT), universe)
    noise = "tests/regression/quad/L06/tools/test_l6_noise_term.py"
    r = affected.select(["tools/sim/l6_noise_term.py"], mapping, universe, pyindex=index)
    # this file names the module in a string, so it is in the closure too
    assert [p for p in r["pytest_paths"] if p.startswith("tests/regression")] == [noise]
    # a scenario file is not Python: the rule's directories stay
    r = affected.select(["scenarios/quad/L04/acro.yaml"], mapping, universe, pyindex=index)
    assert "tests/regression/quad/L04/tools" in r["pytest_paths"]
    # flatten.py: every ctest label and the card gates, and the tests whose closure holds it
    notes = []
    r = affected.select(["tools/card/flatten.py"], mapping, universe, pyindex=index, notes=notes)
    assert r["ctest_labels"] == universe["ctest_labels"] and "product_table_host" in r["run_ci_steps"]
    assert "tests/regression/quad/L04/tools/test_rate_design.py" in r["pytest_paths"]
    assert not any(p.endswith("/tools") for p in r["pytest_paths"])
    # a nightly test selects itself only in the full run
    nightly = "tests/regression/quad/L06/tools/test_chirp_admission.py"
    selected = lambda **kw: affected.select([nightly], mapping, universe, pyindex=index, **kw)["pytest_paths"]  # noqa: E731
    assert nightly not in selected()
    assert nightly in selected(per_push=False)


def test_check_manifest_passes_on_the_repository():
    mapping = affected.load_map()
    universe = affected.repo_universe()
    universe["pytest_paths"] = affected.map_pytest_paths(mapping)
    assert affected.check_manifest(mapping, universe) == []
    assert affected.cross_checks(mapping, universe)[0] == []
