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


def real_select(changed, **kw):
    mapping = affected.load_map()
    universe = affected.repo_universe()
    universe["pytest_paths"] = affected.map_pytest_paths(mapping)
    return affected.select(changed, mapping, universe, **kw)


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
    ff_eval = "tests/regression/quad/L06/tools/test_l6_ff_eval.py"
    r = affected.select(["tools/sim/l6_ff_eval.py"], mapping, universe, pyindex=index)
    assert [p for p in r["pytest_paths"] if p.startswith("tests/regression")] == [ff_eval]
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


# ------------------------------------------------------------------------------------------------ M0: the register, the cards, CMake, editor files

BUDGET = "design/budget.yaml"
AXES = ("ctest_labels", "run_ci_steps", "pytest_paths", "gz_groups")
QUAD = "tests/regression/quad/"  # the paths below are joined at run time: a literal would make this file a reader of them (the two split names too: a whole helper name would make it an importer)
GZ_T4_READERS = [QUAD + p for p in (
    "L02/tools/test_t4_rule.py", "L04/gz/test_t4_acro.py", "L04/gz/test_t4_steps.py", "L05/gz/test_t4_recovery.py",
    "L05/gz/test_t4_steps.py", "L05/gz/test_t4_yaw.py", "L07/gz/l7_" + "gz.py", "L07/gz/test_t4_hover_estimate.py",
    "L07/results/hover_kick/hover_" + "kick.py", "L11/gz/test_t4_vertical.py")]


def budget_select(monkeypatch, make_base):
    """select() for design/budget.yaml when the base holds make_base(committed text) and the head holds the committed text."""
    mapping = affected.load_map()
    universe = affected.repo_universe()
    universe["pytest_paths"] = affected.map_pytest_paths(mapping)
    head = (ROOT / BUDGET).read_text(encoding="utf-8")
    base = make_base(head)
    assert base != head
    monkeypatch.setattr(affected, "file_at", lambda rev, path, root=ROOT: head if rev is None else base)
    tracked = affected.repo_files(ROOT)
    register = affected.register_change(mapping, "BASE", None, [BUDGET], tracked)
    notes = []
    result = affected.select([BUDGET], mapping, universe, pyindex=affected.PythonIndex(ROOT, tracked, universe), register=register,
                             notes=notes, per_push=False)
    return result, register, notes


def test_budget_without_a_key_diff_is_a_full_root():
    r = real_select([BUDGET])
    assert r["full"] is True and r["full_reasons"] == [f"{BUDGET}: design/**"]


def test_one_budget_value_change_selects_the_readers_of_that_key(monkeypatch):
    # t4_settle_fraction 0.02 -> 0.05 in the base: only the T4 behaviour tests name it (t4_rule.py only in comments)
    result, register, _ = budget_select(monkeypatch, lambda t: t.replace("t4_settle_fraction:\n  value: 0.02", "t4_settle_fraction:\n  value: 0.05"))
    assert register["keys"] == {"t4_settle_fraction": "changed"} and register["structural"] is False
    assert register["readers"] == {"t4_settle_fraction": GZ_T4_READERS}
    assert result["full"] is False
    assert result["gz_groups"] == ["l11", "l2", "l4", "l5", "l7"]
    # the readers are tests of gz groups: no ctest label beyond the register's own, no tools tests of the readers
    assert result["ctest_labels"] == ["L07", "L11"]  # L07: the results rule of hover_kick.py, a reader of the key
    assert result["pytest_paths"] == [QUAD + p for p in ("L01/tools/test_card_lint.py", "L01/tools/test_flatten_report.py",
                                                         "L06/tools/test_l6_ff_eval.py")]
    assert {"product_table_host", "product_table_m33", "l1_report", "l1_card_lint"} <= set(result["run_ci_steps"])
    assert "att_t3_reference_reproduces" not in result["run_ci_steps"]


def test_a_new_budget_parameter_also_selects_the_id_manifest_readers(monkeypatch):
    result, register, _ = budget_select(monkeypatch, lambda t: t.split("\nt4_settle_fraction:")[0] + "\n")
    assert register["keys"] == {"t4_settle_fraction": "added"} and register["structural"] is True
    assert result["full"] is False
    assert result["ctest_labels"] == ["L04", "L05", "L07", "L11"]
    assert QUAD + "L04/tools/test_rate_design.py" in result["pytest_paths"]
    assert result["gz_groups"] == ["l11", "l2", "l4", "l5", "l7"]


def test_a_budget_text_change_selects_only_what_reads_the_file(monkeypatch):
    result, register, _ = budget_select(monkeypatch, lambda t: t.replace("A test-harness", "A test harness"))
    assert register["text_only"] == ["t4_settle_fraction"] and register["structural"] is False
    assert result["full"] is False and result["gz_groups"] == [] and result["ctest_labels"] == ["L11"]


def test_a_budget_key_that_cannot_be_mapped_falls_back_to_full_and_says_why(monkeypatch):
    # p: one character, a whole-word scan of every file would match unrelated text
    result, register, notes = budget_select(monkeypatch, lambda t: t.replace("p:\n  value: UNKNOWN", "p:\n  value: 0.5", 1))
    assert register["readers"] == {"p": None}
    assert result["full"] is True
    assert len(result["full_reasons"]) == 1 and result["full_reasons"][0].startswith(f"{BUDGET}: key p: its name has 1 character(s)")
    assert "reason design/budget.yaml: key p: its name" in affected.summary(result, notes)


def test_a_budget_key_read_by_a_full_root_selects_everything_and_names_the_reader(monkeypatch):
    # ci/run_ci.sh reads per_push_check_time_max for the CTest TIMEOUT of every test
    result, *_ = budget_select(monkeypatch, lambda t: t.replace("per_push_check_time_max:\n  value: ", "per_push_check_time_max:\n  value: 9", 1))
    assert result["full"] is True
    assert "ci/run_ci.sh: ci/run_ci.sh (reads budget key per_push_check_time_max)" in result["full_reasons"]


def test_a_budget_key_the_card_generators_read_selects_their_consumers(monkeypatch):
    result, register, _ = budget_select(monkeypatch, lambda t: t.replace("PM_min:\n  value: 0.7853981633974483", "PM_min:\n  value: 0.5", 1))
    assert "tools/card/rate.py" in register["readers"]["PM_min"]
    assert result["full"] is False
    assert result["ctest_labels"] == sorted(affected.repo_universe()["ctest_labels"])
    assert QUAD + "L04/tools/test_rate_design.py" in result["pytest_paths"]


def test_the_card_selects_its_consumers_and_a_new_card_only_the_lint():
    card = real_select(["vehicles/uzh_neurobem_5in.yaml"])
    assert card["full"] is False
    assert card["ctest_labels"] == ["L01", "L02", "L03", "L04", "L05", "L06", "L11"]
    assert card["gz_groups"] == ["l11", "l2", "l4", "l5", "l7"] and "product_table_host" in card["run_ci_steps"]
    assert real_select(["sensors/profiles/marv_v2_board_default.yaml"]) == {**card, "changed": ["sensors/profiles/marv_v2_board_default.yaml"]}
    for new in ("vehicles/new_quad.yaml", "sensors/profiles/new_board.yaml"):
        assert real_select([new])["full"] is False and real_select([new])["gz_groups"] == card["gz_groups"]  # consumption unknown
        r = real_select([new], consumed=set())
        assert r["full"] is False and r["ctest_labels"] == [] and r["gz_groups"] == []
        assert r["run_ci_steps"] == ["l1_card_lint", "tools_tests"]
        assert r["pytest_paths"] == [QUAD + "L01/tools/test_card_lint.py"]


def test_cmake_lists_select_their_directory_and_only_the_roots_select_everything():
    assert real_select(["fw/rate/CMakeLists.txt"])["ctest_labels"] == real_select(["fw/rate/include/marv/rate/rate.hpp"])["ctest_labels"]
    assert real_select(["tests/regression/quad/L04/unit/rate/CMakeLists.txt"])["ctest_labels"] == ["L04"]
    assert real_select(["tests/regression/quad/L06/controls/CMakeLists.txt"])["ctest_labels"] == ["L06"]
    ci = real_select(["tools/ci/CMakeLists.txt"])
    assert ci["full"] is False and "g1_lint" in ci["run_ci_steps"] and "tests/regression/quad/L00/tools" in ci["pytest_paths"]
    # the directory is a full root: its CMakeLists.txt is too; the top level and the label machinery stay full
    for path in ("fw/params/CMakeLists.txt", "fw/prim/CMakeLists.txt", "CMakeLists.txt", "CMakePresets.json",
                 "cmake/flags.cmake", "tests/regression/CMakeLists.txt"):
        assert real_select([path])["full"] is True, path


def test_agent_and_editor_files_select_nothing_wherever_they_are():
    for path in (".claude/settings.json", ".serena/project.yml", ".vscode/settings.json", ".idea/workspace.xml",
                 "fw/rate/src/rate.cpp.swp", "fw/prim/include/.constants.hpp.swo", "ci/run_ci.sh~", "tests/regression/quad/L04/gz/test_t4_steps.py.swp"):
        r = real_select([path])
        assert r["full"] is False and [r[k] for k in AXES] == [[], [], [], []], path


def test_a_card_or_profile_something_consumes_selects_what_the_committed_card_selects(tmp_path):
    """zzcardx is named by a param-set line of fw/params/CMakeLists.txt; its profile only by the card; zzlone by nothing."""
    files = {
        "fw/params/CMakeLists.txt": 'marv_add_card_param_set(marv_params_zz CARD "${PROJECT_SOURCE_DIR}/vehicles/zzcardx.yaml")\n',
        "vehicles/zzcardx.yaml": "sensor_profile: zzprof\n",
        "sensors/profiles/zzprof.yaml": "x: 1\n",
        "vehicles/zzlone.yaml": "y: 1\n",
        "sensors/profiles/zzorphan.yaml": "z: 1\n",
        "vehicles/zzring_a.yaml": "peer: zzring_b\n",
        "vehicles/zzring_b.yaml": "peer: zzring_a\n",
    }
    for rel, text in files.items():
        (tmp_path / rel).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / rel).write_text(text)
    mapping = affected.load_map()
    cards = sorted(p for p in files if p.startswith(("vehicles/", "sensors/")))
    consumed = affected.consumed_card_files(mapping, cards, sorted(files), tmp_path)
    assert consumed == {"vehicles/zzcardx.yaml", "sensors/profiles/zzprof.yaml"}
    committed = real_select(["vehicles/uzh_neurobem_5in.yaml"])
    for path in sorted(consumed):
        r = real_select([path], consumed=consumed)
        assert r["full"] is False
        assert {k: r[k] for k in AXES} == {k: committed[k] for k in AXES}, path
    for path in ("vehicles/zzlone.yaml", "sensors/profiles/zzorphan.yaml", "vehicles/zzring_a.yaml"):
        r = real_select([path], consumed=consumed)
        assert r["run_ci_steps"] == ["l1_card_lint", "tools_tests"] and r["gz_groups"] == [] and r["ctest_labels"] == [], path
