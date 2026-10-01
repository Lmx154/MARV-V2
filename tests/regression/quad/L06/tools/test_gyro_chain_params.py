"""The gyro chain's derived parameters (tools/card/gyro_chain_params.py through tools/card/flatten.py --out-gyro-chain; L6
stage (b), decision 0013): the values equal the rules restated independently here, the generator refuses an ESC clock error
above the budget's requirement and any UNKNOWN input, and the figures match the design report (P5) at 2 % and at 3.8 %.

The oracle restates the rules from the K form of the digital notch and the Butterworth gain, not from gyro_chain_design.py.
Every acceptance test has a control that breaks it (core 7.2). Tolerances are derived on their lines.
"""

import cmath
import math
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import schema  # noqa: E402

FLATTEN = ROOT / "tools" / "card" / "flatten.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
PROFILE_REL = Path("sensors") / "profiles" / "marv_v2_board_default.yaml"
PROFILE = ROOT / PROFILE_REL

G = 9.80665            # standard gravity, m/s^2, exact by definition (1901 3rd CGPM)
TELEMETRY_STEP = 2.0 ** -8
NAMES = ("gyro_lpf_cutoff_hz", "gyro_notch_q_h1", "gyro_notch_q_h2", "gyro_notch_q_h3", "gyro_notch_omega_min")

# The design report's figures (P5, tools/card/gyro_chain_design.py --report): printed with four decimals (the cutoff, Q) and
# three (omega_th), so a recomputed value is within half a unit of the last printed decimal.
REPORT_TOL_4 = 5e-5
REPORT_TOL_3 = 5e-4
REPORT_2PCT = {"f_c": 625.4161, "q": (2.0070, 1.8224, 1.5373), "omega_th": 348.037}
REPORT_3P8PCT_Q = (1.1361, 1.0334, 0.8746)
# The bisection of the oracle: both sides are exact to double rounding of the tan, and the gain's sensitivity to Q is order
# 1 / Q, so 1e-9 relative leaves six decades.
Q_REL_TOL = 1e-9


def _profile_text():
    return PROFILE.read_text(encoding="utf-8")


ESC_VALUE = re.compile(r"(      esc_clock_error:\n        value: )2\n")


def mutated_profile(esc):
    text, n = ESC_VALUE.subn(lambda m: f"{m.group(1)}{esc}\n", _profile_text())
    assert n == 1
    return text


def run(*args):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True, check=False)


def flatten(tmp_path, *, profile_text=None, budget_text=None, name="run", extra=()):
    root = tmp_path / name / "root"
    (root / PROFILE_REL).parent.mkdir(parents=True)
    (root / PROFILE_REL).write_text(profile_text if profile_text is not None else _profile_text(), encoding="utf-8")
    budget = tmp_path / name / "budget.yaml"
    budget.write_text(budget_text if budget_text is not None else BUDGET.read_text(encoding="utf-8"), encoding="utf-8")
    out = tmp_path / name / "out"
    r = run(FLATTEN, "--card", CARD, "--budget", budget, "--out-card", out / "card.yaml", "--out-register",
            out / "register.yaml", "--scenario", SCENARIO, "--out-scenario", out / "scenario.yaml",
            "--out-gyro-chain", out / "gyro_chain.yaml", "--root", root, *extra)
    return r, out


def entries(out):
    return schema.load_yaml(out / "gyro_chain.yaml")


def budget_with(name, value):
    text, n = re.subn(rf"(?m)^({name}:\n  value: )[^\n]*\n", lambda m: f"{m.group(1)}{value}\n",
                      BUDGET.read_text(encoding="utf-8"))
    assert n == 1
    return text


# ---- the oracle: the rules restated from the K form ------------------------------------------------------------------


def inputs():
    card, sc, prof = (schema.load_yaml(p) for p in (CARD, SCENARIO, PROFILE))
    fs = 1e6 * sc["tick_period_den"]["value"] / sc["tick_period_num_us"]["value"]
    return {"fs": fs, "d": sc["rate_loop_divisor"]["value"], "omega_max": float(card["rotors"]["speed_range"]["value"][1]),
            "mass": card["mass"]["value"], "k": card["rotors"]["thrust_coeff"]["value"],
            "odr": prof["classes"]["imu"]["entries"]["odr_error"]["value"] * 1e-6}


def _gain_direct(f, f0, q, fs):
    """|H| of the digital notch at f (Hz), from the K form coefficients (K = tan(w0/2))."""
    k = math.tan(math.pi * f0 / fs)
    norm = 1 / (1 + k / q + k * k)
    b = ((1 + k * k) * norm, 2 * (k * k - 1) * norm, (1 + k * k) * norm)
    a = (1, 2 * (k * k - 1) * norm, (1 - k / q + k * k) * norm)
    zi = cmath.exp(-2j * math.pi * f / fs)
    return abs((b[0] + b[1] * zi + b[2] * zi * zi) / (a[0] + a[1] * zi + a[2] * zi * zi))


def worst_edge_gain(f0, q, eps, fs):
    return max(_gain_direct(f0 * (1 - eps), f0, q, fs), _gain_direct(f0 * (1 + eps), f0, q, fs))


def oracle_q(h, eps, a_min, i):
    f0 = h * i["omega_max"] / (2 * math.pi)
    lo, hi = 1e-3, 1e4
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if worst_edge_gain(f0, mid, eps, i["fs"]) <= a_min:
            lo = mid
        else:
            hi = mid
    return lo


def oracle_cutoff(a_min, i):
    """Digital Butterworth gain at f_r/2 equals a_min: |H|^2 = 1/(1 + x^4), x = tan(pi f_r/(2 f_s)) / tan(pi f_c / f_s)."""
    k_nyq = math.tan(math.pi * (i["fs"] / i["d"] / 2) / i["fs"])
    x = (1 / (a_min * a_min) - 1) ** 0.25
    return i["fs"] / math.pi * math.atan(k_nyq / x)


def oracle_omega_th(a_min, i):
    return math.sqrt(i["mass"] * G / (4 * i["k"])) * math.sqrt(a_min)


# ---- the generated values equal the rules ----------------------------------------------------------------------------


def test_generated_values_equal_the_rules_and_the_design_report(tmp_path):
    r, out = flatten(tmp_path)
    assert r.returncode == 0, r.stderr
    e = entries(out)
    assert list(e) == list(NAMES)
    i = inputs()
    eps = TELEMETRY_STEP + i["odr"] + 0.02
    assert e["gyro_lpf_cutoff_hz"]["value"] == pytest.approx(oracle_cutoff(0.1, i), rel=1e-12)
    for h, name in zip((1, 2, 3), NAMES[1:4]):
        assert e[name]["value"] == pytest.approx(oracle_q(h, eps, 0.1, i), rel=Q_REL_TOL)
    assert e["gyro_notch_omega_min"]["value"] == pytest.approx(oracle_omega_th(0.1, i), rel=1e-12)
    # Cross-check against the P5 report's printed figures at 2 %.
    assert e["gyro_lpf_cutoff_hz"]["value"] == pytest.approx(REPORT_2PCT["f_c"], abs=REPORT_TOL_4)
    for want, name in zip(REPORT_2PCT["q"], NAMES[1:4]):
        assert e[name]["value"] == pytest.approx(want, abs=REPORT_TOL_4)
    assert e["gyro_notch_omega_min"]["value"] == pytest.approx(REPORT_2PCT["omega_th"], abs=REPORT_TOL_3)
    # Controls: a_min x 1.1 or an ESC error of 0 gives different figures, by far more than the tolerances.
    assert abs(oracle_cutoff(0.11, i) - e["gyro_lpf_cutoff_hz"]["value"]) > 1.0
    assert abs(oracle_q(1, TELEMETRY_STEP + i["odr"], 0.1, i) - e["gyro_notch_q_h1"]["value"]) > 1.0
    assert abs(oracle_omega_th(0.11, i) - e["gyro_notch_omega_min"]["value"]) > 1.0


def test_each_parameter_carries_provenance_and_the_report_lists_the_values(tmp_path):
    r, out = flatten(tmp_path)
    assert r.returncode == 0, r.stderr
    e = entries(out)
    for name in NAMES:
        assert e[name]["type"] == "f32"
        assert e[name]["sigma"] == schema.UNKNOWN
        assert e[name]["method"].startswith("derived(") and e[name]["method"].endswith(")")
        assert "a_min" in e[name]["method"] and e[name]["source"].strip()
        assert "gyro_chain_attenuation_min" in e[name]["source"]
    assert {n: e[n]["unit"] for n in NAMES} == {"gyro_lpf_cutoff_hz": "Hz", "gyro_notch_q_h1": "1", "gyro_notch_q_h2": "1",
                                               "gyro_notch_q_h3": "1", "gyro_notch_omega_min": "rad/s"}
    for name in NAMES[1:4]:
        assert "esc_clock_error" in e[name]["source"] and "odr_error" in e[name]["source"]
        assert "esc_clock_error" in e[name]["method"]
    assert "standard gravity" in e["gyro_notch_omega_min"]["method"]
    report = (out / "gyro_chain_report.txt").read_text(encoding="utf-8")
    for name in NAMES[:1] + NAMES[4:]:
        assert name in report
    for name in NAMES:
        assert repr(e[name]["value"]) in report


def test_without_the_flag_the_other_outputs_are_byte_identical(tmp_path):
    r, out = flatten(tmp_path)
    assert r.returncode == 0, r.stderr
    plain = tmp_path / "plain"
    root = tmp_path / "run" / "root"
    r = run(FLATTEN, "--card", CARD, "--budget", tmp_path / "run" / "budget.yaml", "--out-card", plain / "card.yaml",
            "--out-register", plain / "register.yaml", "--scenario", SCENARIO, "--out-scenario", plain / "scenario.yaml",
            "--root", root)
    assert r.returncode == 0, r.stderr
    assert not (plain / "gyro_chain.yaml").exists()
    for f in ("card.yaml", "register.yaml", "scenario.yaml"):
        assert (plain / f).read_bytes() == (out / f).read_bytes()


# ---- the refusals ----------------------------------------------------------------------------------------------------


def test_the_budget_carries_the_two_entries_the_rule_reads():
    b = schema.load_yaml(BUDGET)
    assert b["gyro_chain_attenuation_min"]["value"] == 0.1
    assert b["esc_clock_error_max"]["value"] == 0.02


def test_an_esc_error_at_the_requirement_is_accepted_and_above_it_is_refused(tmp_path):
    r, _ = flatten(tmp_path, profile_text=mutated_profile(2), name="at")
    assert r.returncode == 0, r.stderr
    for n, esc in enumerate((2.01, 3.8, 100)):
        r, out = flatten(tmp_path, profile_text=mutated_profile(esc), name=f"above{n}")
        assert r.returncode != 0
        assert "esc_clock_error" in r.stderr and "esc_clock_error_max" in r.stderr
        assert not (out / "gyro_chain.yaml").exists()
        assert not (out / "card.yaml").exists()      # nothing is written on a refusal


def test_a_lower_requirement_refuses_the_committed_profile(tmp_path):
    r, out = flatten(tmp_path, budget_text=budget_with("esc_clock_error_max", "0.019"), name="tight")
    assert r.returncode != 0
    assert "esc_clock_error" in r.stderr and not (out / "gyro_chain.yaml").exists()


def test_a_negative_esc_error_is_refused(tmp_path):
    r, out = flatten(tmp_path, profile_text=mutated_profile(-1), name="neg")
    assert r.returncode != 0 and "esc_clock_error" in r.stderr
    assert not (out / "gyro_chain.yaml").exists()


@pytest.mark.parametrize("what,profile,budget,named", [
    ("profile esc_clock_error", lambda: mutated_profile("UNKNOWN"), None, "esc_clock_error"),
    ("profile odr_error", lambda: re.sub(r"(      odr_error:\n        value: )65\n", r"\1UNKNOWN\n", _profile_text()), None,
     "odr_error"),
    ("budget a_min", None, lambda: budget_with("gyro_chain_attenuation_min", "UNKNOWN"), "gyro_chain_attenuation_min"),
    ("budget esc_clock_error_max", None, lambda: budget_with("esc_clock_error_max", "UNKNOWN"), "esc_clock_error_max"),
])
def test_an_unknown_input_is_refused_naming_the_entry(tmp_path, what, profile, budget, named):
    r, out = flatten(tmp_path, profile_text=profile() if profile else None, budget_text=budget() if budget else None,
                     name="unknown")
    assert r.returncode != 0, what
    assert named in r.stderr, r.stderr
    assert not (out / "gyro_chain.yaml").exists()
    # Control: the unmutated inputs are accepted.
    r, _ = flatten(tmp_path, name="control")
    assert r.returncode == 0, r.stderr


# ---- the control: the ESC error moves Q ------------------------------------------------------------------------------


def test_esc_error_3p8_percent_gives_the_reports_q(tmp_path):
    base, out0 = flatten(tmp_path, name="base")
    assert base.returncode == 0, base.stderr
    r, out = flatten(tmp_path, profile_text=mutated_profile(3.8), budget_text=budget_with("esc_clock_error_max", "0.04"),
                     name="stm")
    assert r.returncode == 0, r.stderr
    e, e0 = entries(out), entries(out0)
    for want, name in zip(REPORT_3P8PCT_Q, NAMES[1:4]):
        assert e[name]["value"] == pytest.approx(want, abs=REPORT_TOL_4)
        assert abs(e[name]["value"] - e0[name]["value"]) > 0.5     # and differs from the 2 % figure by far more than the tolerance
    assert e["gyro_lpf_cutoff_hz"]["value"] == e0["gyro_lpf_cutoff_hz"]["value"]   # the cutoff does not depend on the ESC
    assert e["gyro_notch_omega_min"]["value"] == e0["gyro_notch_omega_min"]["value"]


# ---- the telemetry resolution comes from the profile -----------------------------------------------------------------


MANTISSA_VALUE = re.compile(r"(      telemetry_mantissa_bits:\n        value: )9\n")


def mutated_mantissa(value):
    text, n = MANTISSA_VALUE.subn(lambda m: f"{m.group(1)}{value}\n", _profile_text())
    assert n == 1
    return text


def test_nine_mantissa_bits_give_the_step_2_pow_minus_8_and_ten_bits_change_eps_and_q(tmp_path):
    r, out9 = flatten(tmp_path, name="m9")
    assert r.returncode == 0, r.stderr
    e9 = entries(out9)
    assert f"eps {TELEMETRY_STEP + inputs()['odr'] + 0.02!r}" in e9["gyro_notch_q_h1"]["source"]   # 2^-8, as before
    assert e9["gyro_notch_q_h1"]["value"] == pytest.approx(REPORT_2PCT["q"][0], abs=REPORT_TOL_4)   # today's values unchanged
    r, out10 = flatten(tmp_path, profile_text=mutated_mantissa(10), name="m10")
    assert r.returncode == 0, r.stderr
    e10 = entries(out10)
    i = inputs()
    eps10 = 2.0 ** -9 + i["odr"] + 0.02
    assert f"eps {eps10!r}" in e10["gyro_notch_q_h1"]["source"]
    for h, name in zip((1, 2, 3), NAMES[1:4]):
        assert e10[name]["value"] == pytest.approx(oracle_q(h, eps10, 0.1, i), rel=Q_REL_TOL)
        assert e10[name]["value"] > e9[name]["value"] + 0.01          # a finer telemetry grid allows a larger Q (control)
    assert e10["gyro_lpf_cutoff_hz"]["value"] == e9["gyro_lpf_cutoff_hz"]["value"]


@pytest.mark.parametrize("value", ["UNKNOWN", "1", "9.5"])
def test_an_unusable_mantissa_entry_is_refused_naming_it(tmp_path, value):
    r, out = flatten(tmp_path, profile_text=mutated_mantissa(value), name="bad")
    assert r.returncode != 0 and "telemetry_mantissa_bits" in r.stderr, r.stderr
    assert not (out / "gyro_chain.yaml").exists()
