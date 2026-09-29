import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("check_claude_md", ROOT / "tools" / "ci" / "check_claude_md.py")
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)

GOOD = """# X

## ACTIVE-spec rule

Only ACTIVE specs.

## Number rule

Every number has a provenance.

## UNKNOWN rule

Tag it UNKNOWN and stop.

## CI gates

G1 G2 G3 G4 G5 G6 G7 G8
"""


def test_good_file_passes():
    assert check.violations(GOOD) == []


def test_the_real_claude_md_passes():
    assert check.main([str(ROOT / "CLAUDE.md")]) == 0


def test_missing_section_fails():
    text = GOOD.replace("## UNKNOWN rule\n\nTag it UNKNOWN and stop.\n\n", "")
    assert check.violations(text) == ["section '## UNKNOWN rule' is missing"]


def test_empty_section_fails():
    text = GOOD.replace("Every number has a provenance.\n", "")
    assert check.violations(text) == ["section '## Number rule' is empty"]


def test_level_three_heading_does_not_count():
    text = GOOD.replace("## Number rule", "### Number rule")
    assert "section '## Number rule' is missing" in check.violations(text)


def test_missing_gate_fails():
    text = GOOD.replace("G7 ", "")
    assert check.violations(text) == ["section '## CI gates' does not name G7"]


def test_gate_name_must_be_a_whole_word():
    text = GOOD.replace("G1 ", "G10 ")
    assert "section '## CI gates' does not name G1" in check.violations(text)


def test_control_file_fails_for_the_unknown_rule_only():
    control = (ROOT / "tests" / "controls" / "g8_claude_md_missing_unknown.md").read_text()
    assert check.violations(control) == ["section '## UNKNOWN rule' is missing"]


def test_unreadable_file_is_an_error(tmp_path):
    assert check.main([str(tmp_path / "nope.md")]) == 2
