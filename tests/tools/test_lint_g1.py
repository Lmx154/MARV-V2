import importlib.util
from pathlib import Path

import pytest

LINT = Path(__file__).resolve().parents[2] / "tools" / "ci" / "lint_g1.py"
_spec = importlib.util.spec_from_file_location("lint_g1", LINT)
lint = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lint)


def bad(text):
    return [lit for _, lit in lint.scan_text(text)]


@pytest.mark.parametrize(
    "spelling",
    ["0", "1", "2", "0u", "1u", "2UL", "1ull", "0.0", "1.0", "2.0", "2.0f", "0.5", "0.5f", "0.5L", ".5", ".5f",
     "5e-1", "1e0", "2.", "0x0", "0x1", "0x2", "0b1", "00", "01", "02", "0'0", "0X1U", "0x1p0", "1.0e0f"],
)
def test_allowed_spellings(spelling):
    assert bad(f"auto v = {spelling};") == []


@pytest.mark.parametrize(
    "spelling",
    ["3", "4", "10", "1e-3f", "0x10", "9.81", "9.81f", "0.25", "0.50001", "1e3", "0b11", "010", "100u", "1'000",
     "3.0f", "1.0e-3", "0x1p3", "3_km", "0.5e1", "1'0e0f"],
)
def test_disallowed_spellings(spelling):
    assert bad(f"auto v = {spelling};") == [spelling]


def test_constexpr_initialiser_is_caught():
    assert bad("constexpr float k = 9.81f;") == ["9.81f"]


def test_literal_in_expression_is_caught_with_line_number():
    assert lint.scan_text("int f(int x) {\n  return x * 3;\n}\n") == [(2, "3")]


def test_line_comments_ignored():
    assert bad("int a = 1;  // 9.81 and 42\n") == []


def test_block_comments_ignored_and_lines_counted():
    assert lint.scan_text("/* 42\n 43 */\nint a = 7;\n") == [(3, "7")]


def test_line_comment_continuation_ignored():
    assert bad("// 42 \\\n 43\nint a = 1;\n") == []


def test_string_and_char_literals_ignored():
    assert bad('const char* s = "value 42 and \\"43\\""; char c = \'7\'; char q = \'\\\'\'; int a = 2;') == []


def test_prefixed_and_raw_strings_ignored():
    text = 'auto a = u8"42"; auto b = L"43"; auto c = R"x(44 ")" 45)x"; auto d = 3;'
    assert bad(text) == ["3"]


def test_include_lines_ignored():
    assert bad('#include <cstdint>\n#include "x86/vec3.hpp"\n#include <x86.h>\n') == []


def test_digits_in_identifiers_are_not_literals():
    assert bad("int vec3 = 1; int x_9 = 2; auto q0 = Mat3<float>();") == []


def test_digit_separators_are_one_literal():
    assert bad("auto a = 1'000'000; auto b = 0x1'0; auto c = 1'0;") == ["1'000'000", "0x1'0", "1'0"]


def test_separator_after_allowed_digit_does_not_start_char_literal():
    assert bad("auto a = 1'0; int b = 3;") == ["1'0", "3"]


def test_unparseable_literal_is_disallowed():
    assert bad("auto a = 1.2.3;") == ["1.2.3"]


def test_negative_literals_judged_on_magnitude_token():
    assert bad("int a = -1; int b = -3;") == ["3"]


def test_array_size_and_index_literals_are_caught():
    assert bad("float e[3]; e[4] = 0;") == ["3", "4"]


def test_exempt_file_is_constants_hpp_only():
    root = lint.ROOT
    assert lint.is_exempt(root / "fw" / "prim" / "include" / "marv" / "prim" / "constants.hpp")
    assert not lint.is_exempt(root / "fw" / "prim" / "include" / "marv" / "prim" / "vec.hpp")
    assert not lint.is_exempt(root / "tests" / "controls" / "constants.hpp")


def test_real_constants_hpp_holds_its_literal_and_repo_sources_are_otherwise_clean():
    constants = lint.EXEMPT
    assert bad(constants.read_text()) == ["3"]
    for path in lint.fw_sources():
        if path != constants:
            assert bad(path.read_text()) == [], path


@pytest.mark.parametrize(
    "line",
    [
        "int a = 1;  // NOLINT(readability-magic-numbers)",
        "// NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)",
        "// NOLINTBEGIN(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)",
        "// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)",
        "// NOLINT(google-*,readability-magic-numbers)",
        "int a = 1;  // NOLINT",
        "// NOLINTNEXTLINE",
        "// NOLINT(*)",
    ],
)
def test_nolint_for_magic_number_checks_is_rejected(line):
    assert len(lint.nolint_violations(line + "\n")) == 1


@pytest.mark.parametrize(
    "line",
    ["int a = 1;  // NOLINT(bugprone-branch-clone)", "// no lint here", "int a = 1;"],
)
def test_unrelated_nolint_is_accepted(line):
    assert lint.nolint_violations(line + "\n") == []


def test_cli_fails_on_explicit_planted_files_and_passes_clean_ones(tmp_path, monkeypatch):
    planted = tmp_path / "planted.hpp"
    planted.write_text("constexpr float k = 9.81f;\n")
    clean = tmp_path / "clean.hpp"
    clean.write_text("constexpr int k = 2;\n")
    nolint = tmp_path / "nolint.hpp"
    nolint.write_text("int a = 1;  // NOLINT(readability-magic-numbers)\n")
    assert lint.main([str(planted)]) == 1
    assert lint.main([str(nolint)]) == 1
    assert lint.main([str(clean)]) == 0


def test_cli_without_files_is_vacuous_when_no_fw_translation_units(tmp_path):
    (tmp_path / "compile_commands.json").write_text("[]")
    assert lint.main(["--build-dir", str(tmp_path)]) == 2
    assert lint.main(["--build-dir", str(tmp_path / "missing")]) == 2
