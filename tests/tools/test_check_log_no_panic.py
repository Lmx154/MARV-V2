"""Invariant L-3 of the onboard flight logger: fw/log never names hal_panic (tools/ci/check_log_no_panic.py), with its negative control."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CHECK = ROOT / "tools" / "ci" / "check_log_no_panic.py"


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(CHECK), *args], capture_output=True, text=True, check=False)


def test_fw_log_never_calls_hal_panic():
    r = run()
    assert r.returncode == 0, r.stdout + r.stderr


def test_negative_control_a_planted_call_is_found(tmp_path):
    (tmp_path / "bad.cpp").write_text('#include "marv/hal/hal.hpp"\nvoid f() noexcept { marv::hal_panic("card"); }\n')
    r = run(str(tmp_path))
    assert r.returncode == 1
    assert "bad.cpp:2: names hal_panic" in r.stdout
    assert "bad.cpp:1: includes marv/hal/hal.hpp" in r.stdout


def test_comments_and_strings_that_mention_the_rule_are_not_findings(tmp_path):
    (tmp_path / "ok.hpp").write_text('// never hal_panic\n/* hal_panic */\nconst char* k = "hal_panic";\n')
    assert run(str(tmp_path)).returncode == 0


def test_an_empty_directory_is_a_vacuous_run_not_a_pass(tmp_path):
    assert run(str(tmp_path)).returncode == 2
