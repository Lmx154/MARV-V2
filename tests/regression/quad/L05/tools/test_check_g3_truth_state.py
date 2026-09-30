"""Export rules of tools/ci/check_g3.py for the truth_state manifest flag (docs/decisions/0006 section B).

The checker is run through main() against a fake `nm` (a shell script that prints a per-artifact symbol list), so no
build is needed. The negative control of the CI step list lives here: a flagged manifest entry without the
marv_truth_state_set export must be a G3-ERROR (exit 2). The two library controls (an unflagged library that exports
marv_truth_state_set, a flagged one that also exports marv_truth_planted) are C++ shared libraries under
tests/regression/quad/L05/controls, run as CI steps.
"""

import importlib.util
import json
import stat
from pathlib import Path

import pytest

CHECK = Path(__file__).resolve().parents[5] / "tools" / "ci" / "check_g3.py"
_spec = importlib.util.spec_from_file_location("check_g3", CHECK)
g3 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(g3)

SIL_ONLY = ["marv_sil_init", "marv_sil_tick"]


@pytest.fixture
def fake_nm(tmp_path):
    """A `nm` that prints '<artifact>.syms' for `nm -D --defined-only <artifact>`."""
    script = tmp_path / "fake_nm"
    script.write_text('#!/bin/sh\ncat "$3.syms"\n')
    script.chmod(script.stat().st_mode | stat.S_IXUSR)
    return str(script)


def make_library(tmp_path, name, symbols, truth_state=None):
    artifact = tmp_path / f"lib{name}.so"
    artifact.write_text("")
    Path(f"{artifact}.syms").write_text("".join(f"0000000000001000 T {s}\n" for s in symbols))
    entry = {"name": name, "file": str(artifact)}
    if truth_state is not None:
        entry["truth_state"] = truth_state
    return entry


def run_exports(tmp_path, nm, entries, capsys):
    manifest = tmp_path / "g3_manifest.json"
    manifest.write_text(json.dumps({"flight_targets": [], "sil_libraries": entries}))
    status = g3.main(["exports", "--manifest", str(manifest), "--nm", nm])
    return status, capsys.readouterr().out


def test_flagged_library_with_the_export_passes(tmp_path, fake_nm, capsys):
    lib = make_library(tmp_path, "t", SIL_ONLY + ["marv_truth_state_set"], truth_state=True)
    status, out = run_exports(tmp_path, fake_nm, [lib], capsys)
    assert status == 0, out
    assert "G3-" not in out


@pytest.mark.parametrize("truth_state", [None, False])
def test_unflagged_library_is_the_product_rule(tmp_path, fake_nm, capsys, truth_state):
    lib = make_library(tmp_path, "p", SIL_ONLY, truth_state=truth_state)
    status, out = run_exports(tmp_path, fake_nm, [lib], capsys)
    assert status == 0, out


@pytest.mark.parametrize("truth_state", [None, False])
def test_unflagged_library_exporting_marv_truth_state_set_is_g3_export(tmp_path, fake_nm, capsys, truth_state):
    lib = make_library(tmp_path, "p", SIL_ONLY + ["marv_truth_state_set"], truth_state=truth_state)
    status, out = run_exports(tmp_path, fake_nm, [lib], capsys)
    assert status == 1
    assert "G3-EXPORT p" in out and "exports 'marv_truth_state_set'" in out


def test_flagged_library_with_an_extra_export_is_g3_export(tmp_path, fake_nm, capsys):
    lib = make_library(tmp_path, "t", SIL_ONLY + ["marv_truth_state_set", "marv_truth_planted"], truth_state=True)
    status, out = run_exports(tmp_path, fake_nm, [lib], capsys)
    assert status == 1
    assert "G3-EXPORT t" in out and "exports 'marv_truth_planted'" in out
    assert "exports 'marv_truth_state_set'" not in out


def test_flagged_library_without_the_export_is_g3_error(tmp_path, fake_nm, capsys):
    """Negative control: the flag without the export is a tool-level error, not a pass."""
    lib = make_library(tmp_path, "t", SIL_ONLY, truth_state=True)
    status, out = run_exports(tmp_path, fake_nm, [lib], capsys)
    assert status == 2
    assert out.startswith("G3-ERROR ") and "marv_truth_state_set" in out
    assert "G3-EXPORT" not in out


def test_flagged_library_with_only_the_truth_export_passes(tmp_path, fake_nm, capsys):
    lib = make_library(tmp_path, "t", ["marv_truth_state_set"], truth_state=True)
    status, out = run_exports(tmp_path, fake_nm, [lib], capsys)
    assert status == 0, out


@pytest.mark.parametrize("name", ["marv_truth_state_set2", "marv_truth_state_get", "planted", "marv_sil", "marv_truth_"])
def test_flagged_library_rejects_every_other_name(tmp_path, fake_nm, capsys, name):
    lib = make_library(tmp_path, "t", SIL_ONLY + ["marv_truth_state_set", name], truth_state=True)
    status, out = run_exports(tmp_path, fake_nm, [lib], capsys)
    assert status == 1
    assert f"exports '{name}'" in out


@pytest.mark.parametrize("flag", ["true", 1, None, "false"])
def test_a_non_boolean_flag_is_g3_error(tmp_path, fake_nm, capsys, flag):
    lib = make_library(tmp_path, "t", SIL_ONLY + ["marv_truth_state_set"])
    lib["truth_state"] = flag
    status, out = run_exports(tmp_path, fake_nm, [lib], capsys)
    assert status == 2
    assert out.startswith("G3-ERROR ")


def test_the_flag_is_per_library(tmp_path, fake_nm, capsys):
    flagged = make_library(tmp_path, "t", SIL_ONLY + ["marv_truth_state_set"], truth_state=True)
    product = make_library(tmp_path, "p", SIL_ONLY + ["marv_truth_state_set"], truth_state=False)
    status, out = run_exports(tmp_path, fake_nm, [flagged, product], capsys)
    assert status == 1
    assert "G3-EXPORT p" in out and "G3-EXPORT t" not in out
