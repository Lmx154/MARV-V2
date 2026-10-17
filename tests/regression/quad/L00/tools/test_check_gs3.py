import importlib.util
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
spec = importlib.util.spec_from_file_location("check_gs3", ROOT / "tools" / "ci" / "check_gs3.py")
gs3 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gs3)

CHECK = ROOT / "tools" / "ci" / "check_gs3.py"
CONTROL = ROOT / "tests" / "regression" / "quad" / "L00" / "controls" / "gs3_mavlink_include.cpp"


def run(*args):
    return subprocess.run(["python3", str(CHECK), *args], capture_output=True, text=True, cwd=ROOT)


def test_clean_repository_passes():
    result = run()
    assert result.returncode == 0, result.stdout
    assert result.stdout.startswith("GS-3 clean")


def test_planted_control_fails():
    result = run(str(CONTROL))
    assert result.returncode == 1
    assert "GS3-VIOLATION" in result.stdout
    assert f"{CONTROL}:3: names MAVLink" in result.stdout


def test_control_is_rejected_for_the_identifier_too(tmp_path):
    f = tmp_path / "x.cpp"
    f.write_text("int a = PARAM_SET;\n")
    result = run(str(f))
    assert result.returncode == 1
    assert "in-scope identifier 'PARAM_SET'" in result.stdout


def test_marv_dialect_name_is_an_identifier(tmp_path):
    f = tmp_path / "x.cpp"
    f.write_text("int a = MARV_CONFIG_STATE_OK;\n")
    assert run(str(f)).returncode == 1


def test_identifier_must_be_a_whole_word(tmp_path):
    f = tmp_path / "x.cpp"
    f.write_text("int PARAM_SETTLED = 1; int MY_HEARTBEAT_S = 2;\n")
    assert run(str(f)).returncode == 0


def test_unreadable_file_is_an_error(tmp_path):
    assert run(str(tmp_path / "nope.cpp")).returncode == 2


def test_allow_list_globs():
    assert gs3.allowed("fw/link/src/codec_mavlink.cpp")
    assert gs3.allowed("tests/regression/quad/L00/controls/gs3_mavlink_include.cpp")
    assert gs3.allowed("gcs/src/lib/link/generated/messages.ts")
    assert not gs3.allowed("gcs/src/routes/+page.svelte")
    assert not gs3.allowed("fw/quad_fc/src/x.cpp")
    assert not gs3.allowed("fw/link/include/marv/link/messages.hpp")


def test_a_tracked_file_outside_the_allow_list_is_flagged(tmp_path):
    for sub in ("link",):
        (tmp_path / sub).mkdir()
    (tmp_path / "link" / "scope.txt").write_text("HEARTBEAT\n")
    (tmp_path / "link" / "marv.xml").write_text('<mavlink><messages><message name="MARV_X"/></messages></mavlink>')
    (tmp_path / "fw").mkdir()
    (tmp_path / "fw" / "a.cpp").write_text("#include <mavlink.h>\n")
    subprocess.run(["git", "init", "-q"], cwd=tmp_path, check=True)
    subprocess.run(["git", "add", "."], cwd=tmp_path, check=True)
    result = run("--root", str(tmp_path))
    assert result.returncode == 1
    assert "GS3-VIOLATION: fw/a.cpp:1" in result.stdout
