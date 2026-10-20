"""DERIVE against a CMake build (ground-segment unit GX-13, tools/gs/derive.py; done-when 1): the .mcfg the helper derives from the repository's
flight card is byte-identical to the one a CMake build of the same card produces, and one edited value changes the .mcfg and the values derived
from it exactly as a CMake build of the same edited card does.

The CMake side is a configure and a build of the target `marv_params_fc3v2_generated` (fw/params/CMakeLists.txt) in a TEMPORARY COPY of the
repository's tracked files: the edited card is written into the copy, never into the repository (W-1). The helper side is DeriveManager with its real
steps. Both run the flatten for about a minute on every CPU, twice each, so this file is a merge-gate file (ci/run_ci.sh nightly_tools_tests) and is
run under ci/heavy.sh by hand (the evidence: tests/regression/quad/L10/results/gcs/gx13/README.md). It compares:
  - the .mcfg bytes,
  - every file of the flatten directory (the derivation reports included: the work directory mirrors the repository paths, so their headers are the
    repository-relative ones a CMake build prints),
  - params_manifest.json, params_provenance.json, param_ids.hpp and param_defaults.cpp,
  - and, between the unedited and the edited card, the set of parameters whose default changed.
"""

import asyncio
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "gs"))
sys.path.insert(0, str(ROOT / "tools" / "config"))

import derive as derive_mod  # noqa: E402
import mcfg  # noqa: E402

CARD = "vehicles/marv_7in_fc3v2.yaml"
SOURCE = "Luis 2026-10-21: scale reading 0.800 kg with the pack and props on (GX-13 test text)"
EDIT = {"key": "mass", "value": 0.8, "source": SOURCE}
SET = derive_mod.PARAMS_SET
STEP_TIMEOUT_S = 3600  # labelled harness value: bounds a hung CMake step in this test, it decides no result


class Library:
    def scan(self):
        return {"images": [], "unkeyed": []}


def run_derive(work_root: Path, edits: list[dict]) -> Path:
    """DeriveManager with its real steps on the repository's card; returns the derivation's work directory."""
    async def main() -> Path:
        m = derive_mod.DeriveManager(root=ROOT, run_dir=work_root, build_root=work_root, library=Library(), log=print, push=lambda e: None,
                                     lock_path=lambda: work_root / "heavy.lock", min_free_bytes=0)
        await m.start({"cmd": "derive", "card": CARD, "edits": edits})
        await m._task
        assert m.state["ok"] is True, m.state
        return m.derive_dir / m.state["id"]
    return asyncio.run(main())


def cmake_build(copy: Path) -> Path:
    """Configures (once) and builds the flight set in the copy; returns its generated directory."""
    env = {k: v for k, v in os.environ.items() if k not in ("MARV_CACHE_DIR", "MARV_HEAVY_HELD")}
    if not (copy / "build" / "host-debug" / "build.ninja").exists():
        subprocess.run(["cmake", "--preset", "host-debug", f"-DMARV_PYTHON={sys.executable}"], cwd=copy, env=env, check=True, timeout=STEP_TIMEOUT_S)
    subprocess.run(["cmake", "--build", "--preset", "host-debug", "--target", f"{SET}_generated"], cwd=copy, env=env, check=True, timeout=STEP_TIMEOUT_S)
    return copy / "build" / "host-debug" / "generated" / SET


def tracked_copy(dest: Path) -> None:
    names = subprocess.run(["git", "-C", str(ROOT), "ls-files", "-z"], capture_output=True, check=True).stdout.split(b"\0")
    for raw in filter(None, names):
        src = ROOT / os.fsdecode(raw)
        if src.is_symlink() or not src.is_file():
            continue
        target = dest / os.fsdecode(raw)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, target)


def manifest_defaults(path: Path) -> dict:
    return {n: e["default_bits"] for n, e in json.loads(path.read_text())["params"].items()}


def compare(work: Path, generated: Path) -> None:
    """The helper's result is the CMake build's, file by file."""
    assert (work / f"{SET}.mcfg").read_bytes() == (generated / f"{SET}.mcfg").read_bytes(), "the .mcfg differs"
    flat_cmake = {p.name: p for p in (generated / "flatten").iterdir()}
    flat_work = {p.name: p for p in (work / "flatten").iterdir()}
    assert set(flat_work) == set(flat_cmake), set(flat_work) ^ set(flat_cmake)
    for name, path in sorted(flat_cmake.items()):
        assert flat_work[name].read_bytes() == path.read_bytes(), f"flatten/{name} differs"
    for name in ("params_manifest.json", "params_provenance.json", "param_ids.hpp", "param_defaults.cpp"):
        mine = (work / "gen" / "marv" / "params" / name).read_text()
        theirs = (generated / "marv" / "params" / name).read_text()
        if name.endswith(".json"):  # the file label of each entry is the path of the flattened file, which lies in another directory
            mine, theirs = (json.dumps(strip_labels(json.loads(t)), sort_keys=True) for t in (mine, theirs))
        assert mine == theirs, f"{name} differs"


def strip_labels(doc):
    if isinstance(doc, dict):
        return {k: strip_labels(v) for k, v in doc.items() if k != "source_file"}
    if isinstance(doc, list):
        return [strip_labels(v) for v in doc]
    return doc


@pytest.fixture(scope="module")
def results(tmp_path_factory):
    base = tmp_path_factory.mktemp("derive-vs-cmake")
    copy = base / "repo"
    tracked_copy(copy)
    unedited = {"derive": run_derive(base / "helper-unedited", []), "cmake": cmake_build(copy)}
    cmake_unedited = {p.name: p.read_bytes() for p in [unedited["cmake"] / f"{SET}.mcfg"]}
    unedited_defaults = manifest_defaults(unedited["cmake"] / "marv" / "params" / "params_manifest.json")
    edited_derive = run_derive(base / "helper-edited", [EDIT])
    edited_card = edited_derive / "root" / CARD
    shutil.copyfile(edited_card, copy / CARD)  # the temporary copy's card, not the repository's
    edited = {"derive": edited_derive, "cmake": cmake_build(copy)}
    return {"unedited": unedited, "edited": edited, "cmake_unedited_mcfg": cmake_unedited[f"{SET}.mcfg"], "unedited_defaults": unedited_defaults}


def test_the_mcfg_derived_from_the_unedited_card_is_the_cmake_builds_byte_for_byte(results):
    compare(results["unedited"]["derive"], results["unedited"]["cmake"])


def test_one_edited_value_changes_the_mcfg_and_the_derived_values_exactly_as_a_cmake_build_of_the_edited_card_does(results):
    compare(results["edited"]["derive"], results["edited"]["cmake"])
    edited_mcfg = (results["edited"]["derive"] / f"{SET}.mcfg").read_bytes()
    assert edited_mcfg != (results["unedited"]["derive"] / f"{SET}.mcfg").read_bytes()
    assert edited_mcfg != results["cmake_unedited_mcfg"]
    mine = manifest_defaults(results["edited"]["derive"] / "gen" / "marv" / "params" / "params_manifest.json")
    theirs = manifest_defaults(results["edited"]["cmake"] / "marv" / "params" / "params_manifest.json")
    base = results["unedited_defaults"]
    changed_mine = {n for n in mine if mine[n] != base[n]}
    changed_theirs = {n for n in theirs if theirs[n] != base[n]}
    assert changed_mine == changed_theirs and "mass" in changed_mine
    manifest = json.loads((results["edited"]["derive"] / "gen" / "marv" / "params" / "params_manifest.json").read_text())["params"]
    assert {n for n in changed_mine if manifest[n]["method"] == "derived"}, "an edit of the mass changes values derived from it"
    cfg = mcfg.decode(edited_mcfg)
    assert [s.kind for s in cfg.sections] == ["quad"]


def test_the_repository_card_is_not_the_edited_one():
    """The comparison above edits the copy's card only."""
    assert "value: 0.732" in (ROOT / CARD).read_text()
