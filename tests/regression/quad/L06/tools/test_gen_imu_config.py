"""IMU configuration generator (tools/card/gen_imu_config.py; L6 stage (a), decision 0012): the SI conversion, the accel
range rule, the refusals, the perturbation control and the derived-figure report. No simulator; the committed profile is
never edited (mutated copies are written under tmp_path, named like the profile because the linter requires it).

Float format rule: the header carries C hexadecimal floating literals (float.hex), so float.fromhex of a header number
is the generated double exactly. The expected values restate the generator's documented conversion order, written out
here independently from fixed literals (DEG = pi / 180, G = 9.80665, milli 1e-3, micro 1e-6).

Every acceptance test has a control that breaks it: the defect the test refuses is present in the control.
"""

import copy
import math
import re
import subprocess
import sys
from fractions import Fraction
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import gen_imu_config as gic  # noqa: E402
import schema  # noqa: E402

PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"
GEN = ROOT / "tools" / "card" / "gen_imu_config.py"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
DEG = math.pi / 180.0
G = 9.80665
FLOWN = (
    "gyro_noise_density", "gyro_bias_instability", "gyro_fifo_sensitivity", "gyro_fsr", "gyro_zero_rate_offset",
    "accel_noise_density", "accel_bias_instability", "accel_fifo_sensitivity", "accel_fsr", "accel_offset",
    "odr_error", "latency_samples",
)
HEX = r"(-?0x[0-9a-f.]+p[+-]\d+)"
FIELD_LINES = {
    "gyro_noise_density": r"c\.gyro\.noise_density = " + HEX,
    "gyro_bias_instability": r"c\.gyro\.bias_instability = " + HEX,
    "gyro_lsb": r"c\.gyro\.lsb = " + HEX,
    "gyro_full_scale": r"c\.gyro\.full_scale = " + HEX,
    "accel_noise_density": r"c\.accel\.noise_density = " + HEX,
    "accel_bias_instability": r"c\.accel\.bias_instability = " + HEX,
    "accel_lsb": r"c\.accel\.lsb = " + HEX,
    "accel_full_scale": r"c\.accel\.full_scale = " + HEX,
    "gyro_bound": r"kImuGyroTurnOnBiasBound = " + HEX,
    "accel_bound": r"kImuAccelTurnOnBiasBound = " + HEX,
    "odr_error": r"kImuOdrError = " + HEX,
}


def run(profile, out, *extra):
    return subprocess.run([sys.executable, str(GEN), "--profile", str(profile), "--out", str(out), *extra],
                          capture_output=True, text=True, check=False)


def fields(header_text):
    out = {k: float.fromhex(re.search(rx, header_text).group(1)) for k, rx in FIELD_LINES.items()}
    out["latency_samples"] = int(re.search(r"c\.latency_samples = (\d+)U;", header_text).group(1))
    return out


def mutated(tmp_path, name, edit):
    doc = copy.deepcopy(schema.load_yaml(PROFILE))
    edit(doc["classes"]["imu"]["entries"])
    d = tmp_path / name
    d.mkdir()
    p = d / PROFILE.name
    p.write_text(yaml.safe_dump(doc, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return p


def generate(profile, tmp_path, name="out"):
    out = tmp_path / f"{name}.hpp"
    r = run(profile, out)
    assert r.returncode == 0, r.stderr
    return out.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def committed(tmp_path_factory):
    d = tmp_path_factory.mktemp("committed")
    return generate(PROFILE, d)


def entry_value(name, doc=None):
    return (doc or schema.load_yaml(PROFILE))["classes"]["imu"]["entries"][name]["value"]


# ---- SI round trip -----------------------------------------------------------------------------------------------


def expected_si():
    v = entry_value
    return {
        "gyro_noise_density": (v("gyro_noise_density") * 1e-3) * DEG,
        "gyro_bias_instability": v("gyro_bias_instability") * DEG,
        "gyro_lsb": DEG / v("gyro_fifo_sensitivity"),
        "gyro_full_scale": v("gyro_fsr") * DEG,
        "accel_noise_density": (v("accel_noise_density")[2] * 1e-6) * G,
        "accel_bias_instability": (v("accel_bias_instability") * 1e-6) * G,
        "accel_lsb": G / v("accel_fifo_sensitivity"),
        "accel_full_scale": v("accel_fsr") * G,
        "gyro_bound": v("gyro_zero_rate_offset") * DEG,
        "accel_bound": (v("accel_offset") * 1e-3) * G,
        "odr_error": v("odr_error") * 1e-6,
    }


def test_si_round_trip_is_bit_exact(committed):
    got = fields(committed)
    for k, want in expected_si().items():
        assert got[k] == want, k
        assert got[k].hex() == want.hex(), k
    assert got["latency_samples"] == entry_value("latency_samples") == 1


def test_si_values_match_exact_rational_arithmetic(committed):
    # An independent check of the conversion itself (not of its rounding order): within 4 ulp of the exact rational
    # value, with pi taken from the same double (the generator's DEG is pi/180 of the double nearest pi).
    got = fields(committed)
    deg = Fraction(math.pi) / 180
    g = Fraction(980665, 100000)
    exact = {
        "gyro_noise_density": Fraction(38, 10) / 1000 * deg,
        "accel_lsb": g / 16384,
        "accel_bound": Fraction(20, 1000) * g,
        "odr_error": Fraction(65, 10**6),
    }
    for k, want in exact.items():
        assert abs(Fraction(got[k]) - want) <= 4 * Fraction(math.ulp(got[k])), k


def test_header_records_the_profile_hash_and_provenance(committed):
    import hashlib
    assert hashlib.sha256(PROFILE.read_bytes()).hexdigest() in committed.splitlines()[0]
    for name in ("gyro_noise_density", "gyro_fifo_sensitivity", "accel_offset", "odr_error", "latency_samples"):
        assert f"entry {name}," in committed
    assert "profile marv_v2_board_default" in committed
    assert "method: scenario" in committed and "status: UNVERIFIED" in committed
    # the committed nominal turn-on bias is not in the default config
    assert "turn_on_bias[i] =" in committed and "c.gyro.turn_on_bias" not in committed.split("imu_corner_config")[0]


def test_header_is_deterministic(tmp_path):
    assert generate(PROFILE, tmp_path, "a") == generate(PROFILE, tmp_path, "b")


# ---- accel range rule -------------------------------------------------------------------------------------------


@pytest.mark.parametrize("fsr, density", [(32, 110), (16, 80), (8, 70)])
def test_accel_range_picks_its_list_position(tmp_path, fsr, density):
    p = mutated(tmp_path, f"fsr{fsr}", lambda e: e["accel_fsr"].update(value=fsr))
    got = fields(generate(p, tmp_path))
    assert got["accel_noise_density"] == (density * 1e-6) * G
    assert got["accel_full_scale"] == fsr * G


def test_committed_profile_range_is_32g_and_110_ug(committed):
    assert entry_value("accel_fsr") == 32
    assert fields(committed)["accel_noise_density"] == (110 * 1e-6) * G


@pytest.mark.parametrize("fsr", [4, 12, 24, 64, 0])
def test_unmatched_accel_range_refuses(tmp_path, fsr):
    p = mutated(tmp_path, f"fsr{fsr}", lambda e: e["accel_fsr"].update(value=fsr))
    out = tmp_path / "o.hpp"
    r = run(p, out)
    assert r.returncode != 0
    assert "accel_noise_density" in r.stderr or "accel_fsr" in r.stderr
    assert not out.exists()


def test_accel_noise_list_of_wrong_length_refuses(tmp_path):
    p = mutated(tmp_path, "len", lambda e: e["accel_noise_density"].update(value=[70, 80]))
    r = run(p, tmp_path / "o.hpp")
    assert r.returncode != 0 and "accel_noise_density" in r.stderr


# ---- refusals ---------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", FLOWN)
def test_unknown_flown_field_refuses_naming_the_entry(tmp_path, name):
    p = mutated(tmp_path, name, lambda e: e[name].update(value="UNKNOWN"))
    out = tmp_path / "o.hpp"
    r = run(p, out)
    assert r.returncode != 0
    assert name in r.stderr
    assert not out.exists()


@pytest.mark.parametrize("name", FLOWN)
def test_missing_flown_field_refuses_naming_the_entry(tmp_path, name):
    p = mutated(tmp_path, name, lambda e: e.pop(name))
    r = run(p, tmp_path / "o.hpp")
    assert r.returncode != 0 and name in r.stderr


@pytest.mark.parametrize("name, unit", [
    ("gyro_noise_density", "deg/s/sqrt(Hz)"), ("gyro_bias_instability", "rad/s"), ("gyro_fifo_sensitivity", "LSB/dps"),
    ("gyro_fsr", "rad/s"), ("gyro_zero_rate_offset", "mdeg/s"), ("accel_noise_density", "mg/sqrt(Hz)"),
    ("accel_bias_instability", "mg"), ("accel_fifo_sensitivity", "LSB/mg"), ("accel_fsr", "m/s^2"),
    ("accel_offset", "g"), ("odr_error", "1"), ("latency_samples", "s"),
])
def test_wrong_unit_refuses_naming_the_entry(tmp_path, name, unit):
    p = mutated(tmp_path, name, lambda e: e[name].update(unit=unit))
    out = tmp_path / "o.hpp"
    r = run(p, out)
    assert r.returncode != 0
    assert name in r.stderr and "unit" in r.stderr
    assert not out.exists()


def test_negative_control_the_unmutated_copy_generates(tmp_path):
    p = mutated(tmp_path, "same", lambda e: None)
    assert run(p, tmp_path / "o.hpp").returncode == 0


def test_a_fractional_latency_refuses(tmp_path):
    p = mutated(tmp_path, "lat", lambda e: e["latency_samples"].update(value=1.5))
    r = run(p, tmp_path / "o.hpp")
    assert r.returncode != 0 and "latency_samples" in r.stderr


# ---- perturbation control --------------------------------------------------------------------------------------


def changed_lines(a, b):
    la, lb = a.splitlines(), b.splitlines()
    assert len(la) == len(lb)
    return [i for i, (x, y) in enumerate(zip(la, lb)) if x != y]


def test_perturbing_gyro_noise_density_changes_that_field_only(tmp_path):
    base = generate(mutated(tmp_path, "base", lambda e: None), tmp_path, "base")
    p = mutated(tmp_path, "pert", lambda e: e["gyro_noise_density"].update(value=e["gyro_noise_density"]["value"] * 1.1))
    pert = generate(p, tmp_path, "pert")
    fb, fp = fields(base), fields(pert)
    assert [k for k in fb if fb[k] != fp[k]] == ["gyro_noise_density"]
    assert fp["gyro_noise_density"] == ((3.8 * 1.1) * 1e-3) * DEG
    changed = changed_lines(base, pert)
    lines = pert.splitlines()
    # the hash line, the field's comment and the field's assignment, nothing else
    assert len(changed) == 3
    assert "sha256" in lines[changed[0]]
    assert "gyro_noise_density" in lines[changed[1]] and "c.gyro.noise_density" in lines[changed[2]]


def test_perturbation_control_a_different_entry_moves_a_different_field(tmp_path):
    base = generate(mutated(tmp_path, "base", lambda e: None), tmp_path, "base")
    p = mutated(tmp_path, "pert", lambda e: e["gyro_bias_instability"].update(
        value=e["gyro_bias_instability"]["value"] * 1.1))
    fb, fp = fields(base), fields(generate(p, tmp_path, "pert"))
    assert [k for k in fb if fb[k] != fp[k]] == ["gyro_bias_instability"]


# ---- report -----------------------------------------------------------------------------------------------------


def report_rows(profile, tmp_path):
    r = run(profile, tmp_path / "o.hpp", "--report", "--scenario", str(SCENARIO))
    assert r.returncode == 0, r.stderr
    rows = {}
    for line in r.stdout.splitlines()[2:]:
        name, *nums = line.split()
        rows[name] = [float(x) for x in nums]
    return rows


def test_report_matches_the_closed_form(tmp_path):
    fs = 1e6 * entry_value_scenario("tick_period_den") / entry_value_scenario("tick_period_num_us")
    assert fs == 6400.0
    rows = report_rows(PROFILE, tmp_path)
    si = expected_si()
    for s, n, b in (("gyro", si["gyro_noise_density"], si["gyro_bias_instability"]),
                    ("accel", si["accel_noise_density"], si["accel_bias_instability"])):
        N, B, tau, K, sigma_d = rows[s]
        assert N == n and B == b
        assert tau == pytest.approx(1.0, rel=1e-12)
        assert K == pytest.approx(math.sqrt(6.0) / 2.0 * b * b / n, rel=1e-12)
        assert sigma_d == pytest.approx(n * math.sqrt(fs / 2.0), rel=1e-12)


def entry_value_scenario(name):
    return schema.load_yaml(SCENARIO)[name]["value"]


def test_report_control_doubling_bias_instability_quadruples_tau_star(tmp_path):
    p = mutated(tmp_path, "b2", lambda e: e["gyro_bias_instability"].update(
        value=e["gyro_bias_instability"]["value"] * 2))
    rows = report_rows(p, tmp_path)
    assert rows["gyro"][2] == pytest.approx(0.25, rel=1e-12)
    assert rows["accel"][2] == pytest.approx(1.0, rel=1e-12)
