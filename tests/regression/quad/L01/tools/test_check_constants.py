import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
TOOL = ROOT / "tools" / "ci" / "check_constants.py"
_spec = importlib.util.spec_from_file_location("check_constants", TOOL)
cc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cc)

REAL = ROOT / "fw" / "prim" / "include" / "marv" / "prim" / "constants.hpp"
CONTROLS = ROOT / "tests" / "regression" / "quad" / "L01" / "controls"

VALID = """\
// Pi. Citation: the mathematical constant.
// Kind: math.
inline constexpr double kPi = 3.14159265358979323846;
"""


def findings(text):
    return cc.check_text(text, "planted.hpp")


def wrap(body):
    return "#pragma once\nnamespace marv::prim {\n\n" + body + "\n}  // namespace marv::prim\n"


def assert_one_reason(text, name, reason):
    got = findings(text)
    assert got, "expected a finding"
    assert all(f": {name}: " in f for f in got), got
    assert any(reason in f for f in got), got


def test_real_constants_hpp_passes():
    assert cc.check_text(REAL.read_text(), "constants.hpp") == []
    assert cc.main([]) == 0


def test_valid_planted_paragraph_passes():
    assert findings(wrap(VALID)) == []


def test_uncited_constant_fails():
    assert_one_reason(wrap("// Gravity.\n// Kind: physics.\ninline constexpr double kG = 9.80665;\n"),
                      "kG", "no 'Citation:'")


def test_constant_with_no_comment_at_all_fails():
    got = findings(wrap("inline constexpr double kG = 9.80665;\n"))
    assert any("no 'Citation:'" in f for f in got) and any("no 'Kind:" in f for f in got)


def test_citation_without_kind_fails():
    assert_one_reason(wrap("// Gravity. Citation: BIPM.\ninline constexpr double kG = 9.80665;\n"),
                      "kG", "no 'Kind:")


def test_citation_with_no_text_fails():
    assert_one_reason(wrap("// Gravity. Citation:\n// Kind: physics.\ninline constexpr double kG = 9.80665;\n"),
                      "kG", "has no text")


def test_kind_vehicle_fails():
    assert_one_reason(wrap("// Poles. Citation: a datasheet.\n// Kind: vehicle.\ninline constexpr int kP = 14;\n"),
                      "kP", "Kind 'vehicle' is not one of math, physics, standard")


def test_two_kind_tags_fail():
    assert_one_reason(
        wrap("// Pi. Citation: the mathematical constant.\n// Kind: math.\n// Kind: physics.\n"
             "inline constexpr double kPi = 3.14159265358979323846;\n"),
        "kPi", "more than one Kind tag")


def test_duplicate_identical_kind_tags_fail():
    assert_one_reason(
        wrap("// Pi. Citation: the mathematical constant.\n// Kind: math. Kind: math.\n"
             "inline constexpr double kPi = 3.14159265358979323846;\n"),
        "kPi", "more than one Kind tag")


@pytest.mark.parametrize(
    "citation",
    [
        "vehicles/uzh_neurobem_5in.yaml",
        "sensors/profiles/icm42688.yaml",
        "design/budget.yaml",
        "the vehicle card",
        "the Card",
        "the sensor profile",
        "the Hobbywing XRotor 2306 datasheet, 14 poles",
        "the motor Datasheet",
    ],
)
def test_citation_referring_to_vehicle_data_fails(citation):
    body = f"// Motor pole count. Citation: {citation}.\n// Kind: physics.\ninline constexpr int kMotorPoleCount = 14;\n"
    assert_one_reason(wrap(body), "kMotorPoleCount", "belong in the card or a sensor profile")


@pytest.mark.parametrize("placeholder", ["TODO", "tbd.", "UNKNOWN", "n/a", "FIXME"])
def test_placeholder_citation_fails(placeholder):
    body = f"// Gravity. Citation: {placeholder}\n// Kind: physics.\ninline constexpr double kG = 9.80665;\n"
    assert_one_reason(wrap(body), "kG", "is a placeholder, not a source")


def test_vehicle_word_inside_another_word_is_not_a_reference():
    body = "// Cardinal number. Citation: the discardable profiler notes.\n// Kind: math.\ninline constexpr int kN = 4;\n"
    assert findings(wrap(body)) == []


def test_citation_after_the_declaration_fails():
    body = "// Kind: math.\ninline constexpr double kPi = 3.14159265358979323846;\n// Citation: the mathematical constant.\n"
    assert_one_reason(wrap(body), "kPi", "no 'Citation:'")


def test_citation_in_the_previous_paragraph_does_not_count():
    body = VALID + "\ninline constexpr double kTau = 6.28318530717958647692;\n"
    assert_one_reason(wrap(body), "kTau", "no 'Citation:'")


def test_non_constexpr_numeric_definition_fails():
    body = "// Pi. Citation: the mathematical constant.\n// Kind: math.\ninline const double kPi = 3.14159265358979323846;\n"
    assert_one_reason(wrap(body), "kPi", "non-constexpr variable with a numeric initialiser")


@pytest.mark.parametrize("definition", ["int g_count = 3;", "static double g_x{2.5};", "extern const int kY = 7;"])
def test_uncited_non_constexpr_numeric_definitions_fail(definition):
    got = findings(wrap(VALID + "\n" + definition + "\n"))
    assert any("non-constexpr variable with a numeric initialiser" in f for f in got), got
    assert any("no 'Citation:'" in f for f in got), got


def test_function_and_declaration_without_numeric_initialiser_are_not_constants():
    body = "int f(int x) { return x + 3; }\nextern int g_declared;\nint g_zero;\nusing T = int;\n"
    assert findings(wrap(VALID + "\n" + body)) == []


def test_planted_uncited_constant_appended_to_a_valid_copy_of_the_real_file_fails():
    text = REAL.read_text()
    assert cc.check_text(text, "copy") == []
    planted = text.replace(
        "}  // namespace marv::prim",
        "\ninline constexpr double kPlantedUnknownName = 12.5;\n\n}  // namespace marv::prim",
    )
    assert planted != text
    got = cc.check_text(planted, "copy")
    assert got and all(": kPlantedUnknownName: " in f for f in got)
    assert any("no 'Citation:'" in f for f in got)


def test_planted_constant_appended_into_the_last_real_paragraph_inherits_its_citation_and_passes():
    text = REAL.read_text()
    planted = text.replace(
        "inline constexpr double kWgs84HeightQuadCoeff = 3.0;\n",
        "inline constexpr double kWgs84HeightQuadCoeff = 3.0;\ninline constexpr double kWgs84Extra = 5.0;\n",
    )
    assert planted != text
    assert cc.check_text(planted, "copy") == []


def test_removing_the_citation_line_from_the_real_wgs84_paragraph_fails():
    text = REAL.read_text()
    kept = [ln for ln in text.splitlines(keepends=True) if "Citation for every entry below:" not in ln]
    assert len(kept) == len(text.splitlines()) - 1
    got = cc.check_text("".join(kept), "copy")
    assert got
    assert all("kWgs84" in f for f in got)
    assert any("no 'Citation:'" in f for f in got)


def test_removing_the_kind_tag_from_the_real_wgs84_paragraph_fails():
    text = REAL.read_text()
    kept = [ln for ln in text.splitlines(keepends=True) if ln.strip() != "// Kind: standard."]
    assert len(kept) < len(text.splitlines())
    got = cc.check_text("".join(kept), "copy")
    assert any("kWgs84A" in f and "no 'Kind:" in f for f in got)


def test_file_with_no_constant_is_vacuous_and_fails():
    assert len(findings("#pragma once\nnamespace marv::prim {\n}\n")) == 1


def test_committed_controls_fail_with_the_intended_reason():
    uncited = cc.check_text((CONTROLS / "constants_uncited.hpp").read_text(), "u")
    assert uncited and any("no 'Citation:'" in f for f in uncited)
    vehicle = cc.check_text((CONTROLS / "constants_vehicle_number.hpp").read_text(), "v")
    assert vehicle and all("belong in the card or a sensor profile" in f for f in vehicle)
    assert cc.main([str(CONTROLS / "constants_uncited.hpp")]) == 1
    assert cc.main([str(CONTROLS / "constants_vehicle_number.hpp")]) == 1


def test_cli_exit_codes(tmp_path, capsys):
    good = tmp_path / "good.hpp"
    good.write_text(wrap(VALID))
    bad = tmp_path / "bad.hpp"
    bad.write_text(wrap("inline constexpr int kX = 7;\n"))
    assert cc.main([str(good)]) == 0
    assert cc.main([str(bad)]) == 1
    out = capsys.readouterr().out
    assert f"constants: {bad}:4: kX: " in out
    assert cc.main([str(tmp_path / "missing.hpp")]) == 2
    assert cc.main(["a", "b"]) == 2
    assert cc.main(["--help"]) == 2


def _plant_in_real_copy(snippet):
    text = REAL.read_text()
    assert cc.check_text(text, "copy") == []
    planted = text.replace("}  // namespace marv::prim", snippet + "\n}  // namespace marv::prim")
    assert planted != text
    return cc.check_text(planted, "copy")


@pytest.mark.parametrize(
    "define",
    ["#define K_PLANTED 3", "#define K_PLANTED (2 * 3.14)", "#define K_PLANTED(x) ((x) * 9)", "#define K_PLANTED \\\n  7"],
)
def test_define_with_numeric_replacement_fails_even_in_a_cited_paragraph(define):
    got = _plant_in_real_copy("// Cited. Citation: something.\n// Kind: math.\n" + define + "\n")
    assert len(got) == 1, got
    assert ": K_PLANTED: #define with a numeric literal; constants.hpp holds constexpr variables only" in got[0]


@pytest.mark.parametrize("define", ["#define MARV_PLANTED_H", "#define K_ALIAS kPi", "#define K_V2 kPi"])
def test_define_without_numeric_replacement_is_accepted(define):
    assert _plant_in_real_copy(define + "\n") == []


def test_define_line_number_is_reported():
    got = cc.check_text("#pragma once\n#define K 5\n" + VALID, "x.hpp")
    assert got and got[0].startswith("constants: x.hpp:2: K: #define")


@pytest.mark.parametrize(
    "enum, name",
    [
        ("enum { kPlanted = 3 };", "kPlanted"),
        ("enum class Mode : int { kA, kPlanted = 0x10, kC };", "kPlanted"),
        ("enum E { kA = 1 << 3 };", "kA"),
    ],
)
def test_enum_with_numeric_initialiser_fails_even_in_a_cited_paragraph(enum, name):
    got = _plant_in_real_copy("// Cited. Citation: something.\n// Kind: math.\n" + enum + "\n")
    assert len(got) == 1, got
    assert f": {name}: enum with an explicit numeric initialiser" in got[0]


def test_enum_without_numeric_initialiser_is_accepted():
    assert _plant_in_real_copy("enum class Mode { kA, kB, kC = kA };\n") == []
