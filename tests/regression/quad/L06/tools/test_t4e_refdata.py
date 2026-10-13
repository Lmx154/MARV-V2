"""The refdata set quad/L06/t4e (tests/regression/quad/L06/t4e/reference; decision 0011's storage, decision 0019 C6a),
per push, without generating the whole set:
  (1) the set is registered (tools/refdata/refdata.py): its generator, its own input and the shared T3 fixtures exist,
      and SHA256SUMS names exactly the generator's outputs;
  (2) the committed t4e_inputs.txt equals t4e_reference.py --refresh-inputs from the repository (the profile, the card,
      the budget and the product parameter table): the set belongs to the current build's numbers;
  (3) one file of the set, l4_step_roll.txt, regenerated from the committed inputs, matches its SHA256SUMS line (the
      whole set is regenerated and checked in the full mode, ci/run_ci.sh; a perturbed input must fail, its negative
      control there).
"""

import hashlib
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "refdata"))
import refdata  # noqa: E402

SET = "quad/L06/t4e"
REF = refdata.REGISTRY[SET]
SLICE = "l4_step_roll"
PROCS = 2  # worker processes of the slice (the output does not depend on it)


def _generator(*args, cwd=ROOT):
    return subprocess.run([sys.executable, str(REF.source / REF.generator), *args], cwd=cwd, check=True,
                          capture_output=True, text=True).stdout


def _copy_inputs(d):
    for name, path in REF.input_paths():
        shutil.copyfile(path, d / name)


def test_the_set_is_registered():
    assert (REF.source / REF.generator).is_file()
    for _, path in REF.input_paths():
        assert path.is_file(), path
    for dep in REF.depends:
        assert (ROOT / dep).is_file(), dep
    names = _generator("--list").split()
    assert sorted(refdata.read_sums(REF)) == sorted(names)


def test_t4e_inputs_equal_the_refresh(tmp_path):
    _copy_inputs(tmp_path)
    (tmp_path / "t4e_inputs.txt").unlink()
    _generator("--refresh-inputs", "--dir", str(tmp_path))
    assert (tmp_path / "t4e_inputs.txt").read_bytes() == (REF.source / "t4e_inputs.txt").read_bytes()


def test_one_file_regenerates_to_its_sha256(tmp_path):
    _copy_inputs(tmp_path)
    _generator("--dir", str(tmp_path), "--procs", str(PROCS), "--only", SLICE)
    got = hashlib.sha256((tmp_path / f"{SLICE}.txt").read_bytes()).hexdigest()
    assert got == refdata.read_sums(REF)[f"{SLICE}.txt"]
