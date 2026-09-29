import importlib.util
import json
import os
from pathlib import Path

import pytest

CHECK = Path(__file__).resolve().parents[2] / "tools" / "ci" / "check_g3.py"
_spec = importlib.util.spec_from_file_location("check_g3", CHECK)
g3 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(g3)

NM_ARCHIVE = """
param.cpp.o:
                 U _GLOBAL_OFFSET_TABLE_
00000000000001ab T marv::param_name(marv::ParamId)
0000000000000000 b marv::(anonymous namespace)::g_table
0000000000000000 W std::span<marv::ParamRecord const, 19ul>::size() const
                 w __gmon_start__

other.cpp.o:
0000000000000010 T marv::truth::planted_truth_state()
                 U marv::hal_sim::hal_sim_reset()
00000010 T marv_truth_planted_c
"""


def test_parse_nm_members_types_and_demangled_names():
    symbols = g3.parse_nm(NM_ARCHIVE)
    assert symbols[0] == ("param.cpp.o", "U", "_GLOBAL_OFFSET_TABLE_")
    assert ("param.cpp.o", "T", "marv::param_name(marv::ParamId)") in symbols
    assert ("param.cpp.o", "W", "std::span<marv::ParamRecord const, 19ul>::size() const") in symbols
    assert ("param.cpp.o", "w", "__gmon_start__") in symbols
    assert ("other.cpp.o", "U", "marv::hal_sim::hal_sim_reset()") in symbols
    assert ("other.cpp.o", "T", "marv_truth_planted_c") in symbols
    assert len(symbols) == 8


def test_parse_nm_rejects_garbage():
    with pytest.raises(g3.G3Error):
        g3.parse_nm("this is not nm output\n")


def test_parse_nm_empty_output_is_no_symbols():
    assert g3.parse_nm("\n\nempty.o:\n") == []


@pytest.mark.parametrize(
    "name, pattern",
    [
        ("marv::truth::planted_truth_state()", "marv::truth::"),
        ("marv_truth_planted_c", "marv_truth_"),
        ("marv::sil::apply_overrides(int)", "marv::sil::"),
        ("marv_sil_init", "marv_sil_"),
        ("marv::hal_sim::reset()", "marv::hal_sim::"),
        ("marv::hal_sim::(anonymous namespace)::state", "marv::hal_sim::"),
        ("void foo<marv::truth::State>(marv::truth::State*)", "marv::truth::"),
    ],
)
def test_forbidden_symbols_match_defined_and_undefined(name, pattern):
    for kind in ("T", "U", "b", "W"):
        found = g3.find_forbidden_symbols([("m.o", kind, name)])
        assert found == [("m.o", kind, name, pattern)]


@pytest.mark.parametrize(
    "name",
    [
        "marv::param_get(marv::ParamId)",
        "marv::truthiness()",
        "marv::silly::x()",
        "marv::hal_panic(char const*)",
        "marv::hal_time_us()",
        "marv_composition_l0_step",
        "_ZN4marv5truth3fooEv",
    ],
)
def test_clean_symbols_do_not_match(name):
    assert g3.find_forbidden_symbols([("m.o", "T", name)]) == []


@pytest.mark.parametrize("name", ["marv_sil_init", "marv_sil_info_get", "marv_sil_status_str"])
def test_sil_exports_accept_marv_sil(name):
    assert g3.non_sil_exports([("", "T", name)]) == []


@pytest.mark.parametrize("name", ["planted_extra_export", "marv_sil", "marv_sil_", "_init", "marv::sil::x()", "marv_silx"])
def test_sil_exports_reject_others(name):
    assert g3.non_sil_exports([("", "T", name)]) == [name]


def test_include_dirs_all_option_spellings(tmp_path):
    d = str(tmp_path)
    args = ["g++", "-Ifw/a", "-I", "fw/b", "-isystem", "fw/c", "-isystemfw/d", "-iquote", "fw/e", "-iquotefw/f",
            "-idirafter", "fw/g", "-c", "x.cpp", "-o", "x.o", "-Wall", "-Iabs" ]
    got = g3.include_dirs(args, d)
    want = [os.path.realpath(os.path.join(d, p)) for p in
            ("fw/a", "fw/b", "fw/c", "fw/d", "fw/e", "fw/f", "fw/g", "abs")]
    assert got == want


def test_include_dirs_absolute_and_relative_dotdot(tmp_path):
    build = tmp_path / "build" / "host-debug"
    got = g3.include_dirs(["-I/abs/dir", "-I../../fw/sil/include"], str(build))
    assert got == [os.path.realpath("/abs/dir"), os.path.realpath(tmp_path / "fw" / "sil" / "include")]


@pytest.mark.parametrize(
    "rel, forbidden",
    [
        ("fw/sil/include", "fw/sil"),
        ("fw/sil", "fw/sil"),
        ("fw/hal/sim/include", "fw/hal/sim"),
        ("sim/null_plant/include", "sim"),
        ("tests/unit/sil/composition/include", "tests"),
        ("fw/hal/include", None),
        ("fw/hal", None),
        ("fw/sched/include", None),
        ("fw/silly/include", None),
        ("simulator/include", None),
        ("testsuite", None),
        ("build/host-debug/generated", None),
    ],
)
def test_forbidden_include_dir(tmp_path, rel, forbidden):
    path = os.path.realpath(tmp_path / rel)
    assert g3.forbidden_include_dir(path, str(tmp_path)) == forbidden


def _database(tmp_path, includes):
    build = tmp_path / "build"
    build.mkdir()
    obj = build / "fw" / "x" / "CMakeFiles" / "x.dir" / "src" / "x.cpp.o"
    manifest = {
        "flight_targets": [{"name": "x", "archive": str(build / "libx.a"), "objects": [str(obj)]}],
        "sil_libraries": [],
    }
    command = "g++ " + " ".join(includes) + " -c ../fw/x/src/x.cpp -o fw/x/CMakeFiles/x.dir/src/x.cpp.o"
    database = [
        {"directory": str(build), "command": command, "file": str(tmp_path / "fw/x/src/x.cpp"),
         "output": "fw/x/CMakeFiles/x.dir/src/x.cpp.o"},
        {"directory": str(build), "command": "g++ -I../fw/sil/include -c ../tests/t.cpp -o t.o",
         "file": str(tmp_path / "tests/t.cpp"), "output": "t.o"},
    ]
    cc = build / "compile_commands.json"
    cc.write_text(json.dumps(database))
    return manifest, cc


def test_check_includes_clean_ignores_non_flight_tus(tmp_path, capsys):
    manifest, cc = _database(tmp_path, ["-I../fw/x/include", "-isystem", "../fw/hal/include"])
    assert g3.check_includes(manifest, cc, tmp_path) == 0
    assert "1 translation units, 0 violations" in capsys.readouterr().out


@pytest.mark.parametrize(
    "flags",
    [
        ["-I../fw/hal/sim/include"],
        ["-I", "../fw/hal/sim/include"],
        ["-isystem", "../fw/sil/include"],
        ["-iquote../sim/null_plant/include"],
        ["-I../fw/x/include", "-I../tests/unit/sil/composition/include"],
    ],
)
def test_check_includes_flags_harness_dirs(tmp_path, capsys, flags):
    manifest, cc = _database(tmp_path, flags)
    assert g3.check_includes(manifest, cc, tmp_path) == 1
    assert capsys.readouterr().out.startswith("G3-INCLUDE x:")


def test_check_includes_arguments_form(tmp_path, capsys):
    manifest, cc = _database(tmp_path, [])
    database = json.loads(cc.read_text())
    database[0].pop("command")
    database[0]["arguments"] = ["g++", "-isystem", "../fw/hal/sim/include", "-c", "x.cpp"]
    cc.write_text(json.dumps(database))
    assert g3.check_includes(manifest, cc, tmp_path) == 1


def test_check_includes_is_never_vacuous(tmp_path):
    manifest, cc = _database(tmp_path, [])
    with pytest.raises(g3.G3Vacuous):
        g3.check_includes({"flight_targets": [], "sil_libraries": []}, cc, tmp_path)
    manifest["flight_targets"][0]["objects"] = [str(tmp_path / "build" / "nowhere.o")]
    with pytest.raises(g3.G3Vacuous):
        g3.check_includes(manifest, cc, tmp_path)


def test_check_symbols_and_exports_are_never_vacuous():
    empty = {"flight_targets": [], "sil_libraries": []}
    with pytest.raises(g3.G3Vacuous):
        g3.check_symbols(empty, "nm")
    with pytest.raises(g3.G3Vacuous):
        g3.check_exports(empty, "nm")


def test_main_reports_missing_artifact_as_error(tmp_path, capsys):
    manifest = tmp_path / "m.json"
    manifest.write_text(json.dumps({
        "flight_targets": [{"name": "x", "archive": str(tmp_path / "missing.a"), "objects": []}],
        "sil_libraries": [],
    }))
    assert g3.main(["symbols", "--manifest", str(manifest)]) == 2
    assert capsys.readouterr().out.startswith("G3-ERROR")


def test_main_empty_manifest_is_vacuous(tmp_path, capsys):
    manifest = tmp_path / "m.json"
    manifest.write_text(json.dumps({"flight_targets": [], "sil_libraries": []}))
    assert g3.main(["symbols", "--manifest", str(manifest)]) == 2
    assert capsys.readouterr().out.startswith("G3-VACUOUS")
