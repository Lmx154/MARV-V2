import importlib.util
import json
import os
from pathlib import Path

import pytest

CHECK = Path(__file__).resolve().parents[5] / "tools" / "ci" / "check_g3.py"
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


def test_include_dirs_strip_sysroot_relative_equals(tmp_path):
    d = str(tmp_path)
    args = ["g++", "-I=fw/a", "-I", "=fw/b", "-isystem=fw/c", "-isystem", "=fw/d", "-iquote=fw/e"]
    want = [os.path.realpath(os.path.join(d, p)) for p in ("fw/a", "fw/b", "fw/c", "fw/d", "fw/e")]
    assert g3.include_dirs(args, d) == want


def test_include_dirs_ignore_forced_include_options(tmp_path):
    assert g3.include_dirs(["-include", "fw/a.h", "-imacros", "fw/b.h", "-includefw/c.h"], str(tmp_path)) == []


def test_forced_includes_all_spellings(tmp_path):
    d = str(tmp_path)
    args = ["g++", "-include", "fw/a.h", "-includefw/b.h", "-imacros", "fw/c.h", "-imacrosfw/d.h", "-Ifw/x",
            "-isystem", "fw/y", "-c", "x.cpp", "-o", "x.o", "-include", "/abs/e.h"]
    want = [os.path.realpath(os.path.join(d, p)) for p in ("fw/a.h", "fw/b.h", "fw/c.h", "fw/d.h", "/abs/e.h")]
    assert g3.forced_includes(args, d) == want


def test_forced_includes_relative_dotdot(tmp_path):
    build = tmp_path / "build" / "host-debug"
    got = g3.forced_includes(["-include", "../../fw/sil/include/marv_sil.h"], str(build))
    assert got == [os.path.realpath(tmp_path / "fw" / "sil" / "include" / "marv_sil.h")]


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
        "flight_targets": [{"name": "x", "type": "STATIC_LIBRARY", "file": str(build / "libx.a"), "objects": [str(obj)]}],
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


@pytest.mark.parametrize(
    "flags",
    [
        ["-include", "../fw/hal/sim/include/marv/hal_sim/hal_sim.hpp"],
        ["-include../fw/sil/include/marv_sil.h"],
        ["-imacros", "../sim/null_plant/include/marv/null_plant/prng.hpp"],
        ["-imacros../tests/unit/prim/test_util.hpp"],
        ["-include", "../fw/x/include/ok.hpp", "-include", "../tests/unit/sil/sil_test_util.hpp"],
    ],
)
def test_check_includes_flags_forced_includes(tmp_path, capsys, flags):
    manifest, cc = _database(tmp_path, flags)
    assert g3.check_includes(manifest, cc, tmp_path) == 1
    out = capsys.readouterr().out
    assert out.startswith("G3-INCLUDE x:")
    assert "forced include" in out


@pytest.mark.parametrize(
    "flags",
    [
        ["-I=../fw/hal/sim/include"],
        ["-isystem=../fw/sil/include"],
        ["-I", "=../sim/null_plant/include"],
        ["-isystem", "=../tests/unit"],
    ],
)
def test_check_includes_flags_sysroot_relative_dirs(tmp_path, capsys, flags):
    manifest, cc = _database(tmp_path, flags)
    assert g3.check_includes(manifest, cc, tmp_path) == 1
    assert capsys.readouterr().out.startswith("G3-INCLUDE x:")


def test_check_includes_allows_clean_forced_include_and_sysroot_dir(tmp_path, capsys):
    manifest, cc = _database(tmp_path, ["-include", "../fw/x/include/ok.hpp", "-I=../fw/hal/include"])
    assert g3.check_includes(manifest, cc, tmp_path) == 0
    assert "0 violations" in capsys.readouterr().out


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
        "flight_targets": [{"name": "x", "type": "STATIC_LIBRARY", "file": str(tmp_path / "missing.a"), "objects": []}],
        "sil_libraries": [],
    }))
    assert g3.main(["symbols", "--manifest", str(manifest)]) == 2
    assert capsys.readouterr().out.startswith("G3-ERROR")


def test_main_empty_manifest_is_vacuous(tmp_path, capsys):
    manifest = tmp_path / "m.json"
    manifest.write_text(json.dumps({"flight_targets": [], "sil_libraries": []}))
    assert g3.main(["symbols", "--manifest", str(manifest)]) == 2
    assert capsys.readouterr().out.startswith("G3-VACUOUS")


def _interface(name, dirs, system_dirs=None):
    return {"name": name, "type": "INTERFACE_LIBRARY", "include_dirs": dirs, "system_include_dirs": system_dirs or [""]}


def test_check_includes_interface_library_clean(tmp_path, capsys):
    manifest, cc = _database(tmp_path, [])
    manifest["flight_targets"].append(_interface("iface", [str(tmp_path / "fw/hal/include"), ""]))
    assert g3.check_includes(manifest, cc, tmp_path) == 0
    out = capsys.readouterr().out
    assert "2 flight targets (1 INTERFACE)" in out
    assert "0 violations" in out


@pytest.mark.parametrize("rel", ["fw/hal/sim/include", "fw/sil/include", "sim/null_plant/include", "tests/unit"])
@pytest.mark.parametrize("field", ["include_dirs", "system_include_dirs"])
def test_check_includes_interface_library_harness_dir_is_a_violation(tmp_path, capsys, rel, field):
    manifest, cc = _database(tmp_path, [])
    iface = _interface("iface", [""])
    iface[field] = [str(tmp_path / rel)]
    manifest["flight_targets"].append(iface)
    assert g3.check_includes(manifest, cc, tmp_path) == 1
    assert capsys.readouterr().out.startswith("G3-INCLUDE iface: INTERFACE include directory")


def test_check_includes_interface_library_relative_dir_is_taken_from_the_repo_root(tmp_path, capsys):
    manifest, cc = _database(tmp_path, [])
    manifest["flight_targets"].append(_interface("iface", ["fw/hal/sim/include"]))
    assert g3.check_includes(manifest, cc, tmp_path) == 1
    assert capsys.readouterr().out.startswith("G3-INCLUDE iface:")


def test_check_includes_only_interface_targets_is_vacuous(tmp_path):
    _, cc = _database(tmp_path, [])
    manifest = {"flight_targets": [_interface("iface", [str(tmp_path / "fw/hal/include")])], "sil_libraries": []}
    with pytest.raises(g3.G3Vacuous):
        g3.check_includes(manifest, cc, tmp_path)


def test_check_includes_interface_entry_without_dirs_is_an_error(tmp_path):
    manifest, cc = _database(tmp_path, [])
    manifest["flight_targets"].append({"name": "iface", "type": "INTERFACE_LIBRARY"})
    with pytest.raises(g3.G3Error):
        g3.check_includes(manifest, cc, tmp_path)


def test_check_includes_object_and_executable_entries(tmp_path, capsys):
    build = tmp_path / "build"
    build.mkdir()
    obj_o = build / "fw" / "o" / "o.cpp.o"
    exe_o = build / "fw" / "e" / "main.cpp.o"
    manifest = {
        "flight_targets": [
            {"name": "obj", "type": "OBJECT_LIBRARY", "objects": [str(obj_o)]},
            {"name": "exe", "type": "EXECUTABLE", "file": str(build / "exe"), "objects": [str(exe_o)]},
        ],
        "sil_libraries": [],
    }
    database = [
        {"directory": str(build), "command": "g++ -I../fw/o/include -c ../fw/o/o.cpp -o fw/o/o.cpp.o",
         "file": "o.cpp", "output": "fw/o/o.cpp.o"},
        {"directory": str(build), "command": "g++ -include ../tests/t.hpp -c ../fw/e/main.cpp -o fw/e/main.cpp.o",
         "file": "main.cpp", "output": "fw/e/main.cpp.o"},
    ]
    cc = build / "compile_commands.json"
    cc.write_text(json.dumps(database))
    assert g3.check_includes(manifest, cc, tmp_path) == 1
    out = capsys.readouterr().out
    assert out.count("G3-INCLUDE") == 1
    assert out.startswith("G3-INCLUDE exe:")
    assert "2 translation units" in out


def test_check_includes_object_library_without_entry_is_vacuous(tmp_path):
    manifest, cc = _database(tmp_path, [])
    manifest["flight_targets"].append({"name": "obj", "type": "OBJECT_LIBRARY", "objects": [str(tmp_path / "no.o")]})
    with pytest.raises(g3.G3Vacuous):
        g3.check_includes(manifest, cc, tmp_path)


BOGUS = {"name": "gen", "type": "UTILITY"}


def test_unknown_target_type_fails_the_include_check(tmp_path):
    manifest, cc = _database(tmp_path, [])
    manifest["flight_targets"].append(BOGUS)
    with pytest.raises(g3.G3Error, match="gen.*UTILITY"):
        g3.check_includes(manifest, cc, tmp_path)


def test_unknown_target_type_fails_the_symbol_check():
    manifest = {"flight_targets": [BOGUS], "sil_libraries": []}
    with pytest.raises(g3.G3Error, match="gen.*UTILITY"):
        g3.check_symbols(manifest, "nm")


def test_target_without_type_is_an_error():
    with pytest.raises(g3.G3Error):
        g3.target_kind({"name": "old", "archive": "libold.a", "objects": []})


@pytest.mark.parametrize("command", ["symbols", "includes"])
def test_main_unknown_target_type_is_g3_error(tmp_path, capsys, command):
    manifest = tmp_path / "m.json"
    manifest.write_text(json.dumps({"flight_targets": [BOGUS], "sil_libraries": []}))
    (tmp_path / "compile_commands.json").write_text("[]")
    assert g3.main([command, "--manifest", str(manifest)]) == 2
    out = capsys.readouterr().out
    assert out.startswith("G3-ERROR")
    assert "gen" in out and "UTILITY" in out


NM_CLEAN = "0000000000000000 T marv::ok()\n"
NM_TRUTH = "0000000000000000 T marv::truth::planted_truth_state()\n"


class _NmStub:
    """Replacement for run_nm that serves canned output per artifact path."""

    def __init__(self, outputs):
        self.outputs = outputs
        self.seen = []

    def __call__(self, nm, flags, artifact):
        self.seen.append(artifact)
        return g3.parse_nm(self.outputs[artifact])


def test_check_symbols_object_library_scans_every_object(monkeypatch, capsys):
    stub = _NmStub({"/b/a.o": NM_CLEAN, "/b/b.o": NM_TRUTH})
    monkeypatch.setattr(g3, "run_nm", stub)
    manifest = {"flight_targets": [{"name": "obj", "type": "OBJECT_LIBRARY", "objects": ["/b/a.o", "/b/b.o"]}],
                "sil_libraries": []}
    assert g3.check_symbols(manifest, "nm") == 1
    out = capsys.readouterr().out
    assert out.startswith("G3-SYMBOL obj (/b/b.o) member b.o: 'marv::truth::planted_truth_state()'")
    assert stub.seen == ["/b/a.o", "/b/b.o"]


def test_check_symbols_executable_and_interface_entries(monkeypatch, capsys):
    stub = _NmStub({"/b/exe": NM_CLEAN})
    monkeypatch.setattr(g3, "run_nm", stub)
    manifest = {
        "flight_targets": [
            {"name": "exe", "type": "EXECUTABLE", "file": "/b/exe", "objects": ["/b/m.o"]},
            _interface("iface", ["/x"]),
        ],
        "sil_libraries": [],
    }
    assert g3.check_symbols(manifest, "nm") == 0
    assert stub.seen == ["/b/exe"]
    assert "1 flight targets, 1 symbols scanned" in capsys.readouterr().out


def test_check_symbols_shared_and_module_are_scanned(monkeypatch):
    stub = _NmStub({"/b/s.so": NM_TRUTH, "/b/m.so": NM_CLEAN})
    monkeypatch.setattr(g3, "run_nm", stub)
    manifest = {
        "flight_targets": [
            {"name": "s", "type": "SHARED_LIBRARY", "file": "/b/s.so", "objects": []},
            {"name": "m", "type": "MODULE_LIBRARY", "file": "/b/m.so", "objects": []},
        ],
        "sil_libraries": [],
    }
    assert g3.check_symbols(manifest, "nm") == 1
    assert stub.seen == ["/b/s.so", "/b/m.so"]


def test_check_symbols_only_interface_targets_is_vacuous():
    manifest = {"flight_targets": [_interface("iface", ["/x"])], "sil_libraries": []}
    with pytest.raises(g3.G3Vacuous):
        g3.check_symbols(manifest, "nm")


def test_check_symbols_object_library_without_objects_is_vacuous():
    manifest = {"flight_targets": [{"name": "obj", "type": "OBJECT_LIBRARY", "objects": [""]}], "sil_libraries": []}
    with pytest.raises(g3.G3Vacuous):
        g3.check_symbols(manifest, "nm")
