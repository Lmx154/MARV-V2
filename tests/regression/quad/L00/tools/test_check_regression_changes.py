import subprocess
import sys
from pathlib import Path

import pytest

CHECKER = Path(__file__).resolve().parents[5] / "tools" / "ci" / "check_regression_changes.py"
TEMPLATE = CHECKER.parents[2] / "docs" / "decisions" / "0000-template.md"

FROZEN = "tests/regression/quad/L00/frozen_test.cpp"
OTHER = "tests/regression/quad/L00/other_test.cpp"

RECORD_HEADINGS = ("## What changed", "## Why", "## Evidence", "## Approval")


def record_text(*paths, headings=RECORD_HEADINGS):
    body = "# 0001: change\n\n"
    for heading in headings:
        body += f"{heading}\n\n" + "\n".join(f"Touches `{p}`." for p in paths) + "\n\n"
    return body


class Repo:
    def __init__(self, root):
        self.root = root

    def git(self, *args):
        subprocess.run(
            ["git", "-c", "user.name=t", "-c", "user.email=t@example.com", *args],
            cwd=self.root,
            check=True,
            capture_output=True,
            text=True,
        )

    def write(self, rel, text):
        path = self.root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)

    def commit(self, message):
        self.git("add", "-A")
        self.git("commit", "-q", "-m", message)

    def run_checker(self):
        return subprocess.run(
            [sys.executable, str(CHECKER), "--base", "base", "--head", "HEAD"],
            cwd=self.root,
            capture_output=True,
            text=True,
        )


@pytest.fixture
def repo(tmp_path):
    r = Repo(tmp_path)
    r.git("init", "-q", "-b", "main")
    r.write(FROZEN, "// frozen v1\n")
    r.write(OTHER, "// other v1\n")
    r.write("README.md", "x\n")
    r.write("docs/decisions/0000-template.md", TEMPLATE.read_text())
    r.commit("baseline")
    r.git("tag", "base")
    return r


def test_addition_only_passes(repo):
    repo.write("tests/regression/quad/L01/new_test.cpp", "// new\n")
    repo.commit("add L01")
    result = repo.run_checker()
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.startswith("PASS")


def test_no_regression_change_passes(repo):
    repo.write("README.md", "y\n")
    repo.commit("unrelated")
    assert repo.run_checker().returncode == 0


def test_modification_without_record_fails(repo):
    """Negative control: loosening a frozen test without a record must be caught."""
    repo.write(FROZEN, "// frozen v2, threshold loosened\n")
    repo.commit("loosen")
    result = repo.run_checker()
    assert result.returncode == 1, result.stdout + result.stderr
    assert result.stdout.startswith("FAIL")
    assert f"{FROZEN}: frozen file modified, deleted or renamed, but no decision record" in result.stdout
    assert "Traceback" not in result.stderr


def test_modification_with_record_passes(repo):
    repo.write(FROZEN, "// frozen v2\n")
    repo.write("docs/decisions/0001-fix-frozen.md", record_text(FROZEN))
    repo.commit("modify with record")
    result = repo.run_checker()
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.startswith("PASS")


@pytest.mark.parametrize("missing", RECORD_HEADINGS)
def test_record_missing_heading_fails(repo, missing):
    headings = tuple(h for h in RECORD_HEADINGS if h != missing)
    repo.write(FROZEN, "// frozen v2\n")
    repo.write("docs/decisions/0001-fix-frozen.md", record_text(FROZEN, headings=headings))
    repo.commit("modify with bad record")
    result = repo.run_checker()
    assert result.returncode == 1, result.stdout + result.stderr
    assert f"missing heading '{missing}'" in result.stdout


def test_record_referencing_different_path_fails(repo):
    repo.write(FROZEN, "// frozen v2\n")
    repo.write("docs/decisions/0001-fix-other.md", record_text(OTHER))
    repo.commit("record names wrong file")
    result = repo.run_checker()
    assert result.returncode == 1, result.stdout + result.stderr
    assert f"{FROZEN}: frozen file" in result.stdout
    assert f"{OTHER}: frozen file" not in result.stdout


def test_every_changed_path_must_be_referenced(repo):
    repo.write(FROZEN, "// frozen v2\n")
    repo.write(OTHER, "// other v2\n")
    repo.write("docs/decisions/0001-fix-frozen.md", record_text(FROZEN))
    repo.commit("two files, one referenced")
    result = repo.run_checker()
    assert result.returncode == 1, result.stdout + result.stderr
    assert f"{OTHER}: frozen file" in result.stdout
    assert f"{FROZEN}: frozen file" not in result.stdout


def test_path_prefix_is_not_a_reference(repo):
    repo.write(FROZEN, "// frozen v2\n")
    repo.write("docs/decisions/0001-fix-frozen.md", record_text(FROZEN + "x"))
    repo.commit("record names a longer path")
    assert repo.run_checker().returncode == 1


def test_path_with_extra_extension_is_not_a_reference(repo):
    repo.write(FROZEN, "// frozen v2\n")
    repo.write("docs/decisions/0001-fix-frozen.md", record_text(FROZEN + ".bak"))
    repo.commit("record names the path plus an extension")
    assert repo.run_checker().returncode == 1


def test_path_at_sentence_end_is_a_reference(repo):
    repo.write(FROZEN, "// frozen v2\n")
    repo.write("docs/decisions/0001-fix-frozen.md", record_text(FROZEN + "."))
    repo.commit("record names the path, then a full stop")
    result = repo.run_checker()
    assert result.returncode == 0, result.stdout + result.stderr


def test_deletion_without_record_fails(repo):
    (repo.root / FROZEN).unlink()
    repo.commit("delete")
    result = repo.run_checker()
    assert result.returncode == 1, result.stdout + result.stderr
    assert f"{FROZEN}: frozen file" in result.stdout


def test_deletion_with_record_passes(repo):
    (repo.root / FROZEN).unlink()
    repo.write("docs/decisions/0001-drop-frozen.md", record_text(FROZEN))
    repo.commit("delete with record")
    result = repo.run_checker()
    assert result.returncode == 0, result.stdout + result.stderr


def test_rename_without_record_fails(repo):
    new = "tests/regression/quad/L00/renamed_test.cpp"
    repo.git("mv", FROZEN, new)
    repo.commit("rename")
    result = repo.run_checker()
    assert result.returncode == 1, result.stdout + result.stderr
    assert f"{FROZEN}: frozen file" in result.stdout
    assert f"{new}: frozen file" in result.stdout


def test_rename_with_record_passes(repo):
    new = "tests/regression/quad/L00/renamed_test.cpp"
    repo.git("mv", FROZEN, new)
    repo.write("docs/decisions/0001-rename-frozen.md", record_text(FROZEN, new))
    repo.commit("rename with record")
    result = repo.run_checker()
    assert result.returncode == 0, result.stdout + result.stderr


def test_rename_out_of_regression_needs_record(repo):
    (repo.root / "tests" / "unit").mkdir()
    repo.git("mv", FROZEN, "tests/unit/moved_test.cpp")
    repo.commit("move out")
    result = repo.run_checker()
    assert result.returncode == 1, result.stdout + result.stderr
    assert f"{FROZEN}: frozen file" in result.stdout


def test_template_alone_is_not_a_record(repo):
    repo.write(FROZEN, "// frozen v2\n")
    repo.write("docs/decisions/0000-template.md", TEMPLATE.read_text() + f"\n{FROZEN}\n")
    repo.commit("edit template to mention the file")
    result = repo.run_checker()
    assert result.returncode == 1, result.stdout + result.stderr
    assert f"{FROZEN}: frozen file" in result.stdout


def test_record_numbered_0000_is_not_a_record(repo):
    repo.write(FROZEN, "// frozen v2\n")
    repo.write("docs/decisions/0000-fix-frozen.md", record_text(FROZEN))
    repo.commit("record numbered like the template")
    assert repo.run_checker().returncode == 1


def test_record_with_bad_name_is_not_a_record(repo):
    repo.write(FROZEN, "// frozen v2\n")
    repo.write("docs/decisions/1-Fix_Frozen.md", record_text(FROZEN))
    repo.commit("badly named record")
    assert repo.run_checker().returncode == 1


def test_unchanged_old_record_does_not_count(repo):
    repo.write("docs/decisions/0001-fix-frozen.md", record_text(FROZEN))
    repo.commit("earlier record")
    repo.git("tag", "-f", "base")
    repo.write(FROZEN, "// frozen v3\n")
    repo.commit("modify without a new record")
    assert repo.run_checker().returncode == 1


def test_base_is_merge_base(repo):
    repo.git("checkout", "-q", "-b", "feature")
    repo.write("README.md", "feature\n")
    repo.commit("feature work")
    repo.git("checkout", "-q", "main")
    repo.write(FROZEN, "// changed on main after the branch point\n")
    repo.write("docs/decisions/0001-main-change.md", record_text(FROZEN))
    repo.commit("main moves")
    repo.git("tag", "-f", "base")
    repo.git("checkout", "-q", "feature")
    result = repo.run_checker()
    assert result.returncode == 0, result.stdout + result.stderr
