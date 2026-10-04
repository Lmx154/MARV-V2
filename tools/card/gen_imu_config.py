#!/usr/bin/env python3
"""Generic-IMU configuration generator (L6 stage (a), decision 0012).

  gen_imu_config.py --profile <profile>.yaml --out <header> [--report] [--scenario <scenario_values.yaml>]

Reads the IMU class of a sensor profile (lint.py's checks run first; an unlinted profile generates nothing) and writes a
C++ header, build directory only, never committed, that builds a marv_plant_imu_config (sim/plant/include/marv_plant.h)
in SI units. Every field comes from a profile entry; no number is defaulted. With --report the derived figures K, tau*
and sigma_d of both sensors are printed on stdout (the model derives them; they are not emitted).

Exit 0 on success. Exit 1 with one line per finding on stderr, and nothing written, when the profile does not lint
clean, or when a flown entry is missing, is UNKNOWN, is not a finite number of the right shape, or has a unit other than
the one this generator converts from. A unit is never guessed.

Conversions (SI = float64, applied left to right, the order is normative: the test reproduces it bit for bit):

  gyro noise_density     (v * 1e-3) * DEG       mdeg/s/sqrt(Hz) -> rad/s/sqrt(Hz)
  gyro bias_instability  v * DEG                deg/s -> rad/s
  gyro lsb               DEG / sensitivity      LSB/(deg/s) -> rad/s per LSB
  gyro full_scale        v * DEG                deg/s -> rad/s
  accel noise_density    (v * 1e-6) * G         ug/sqrt(Hz) -> m/s^2/sqrt(Hz)
  accel bias_instability (v * 1e-6) * G         ug -> m/s^2
  accel lsb              G / sensitivity        LSB/g -> m/s^2 per LSB
  accel full_scale       v * G                  g -> m/s^2
  gyro turn-on bound     v * DEG                deg/s -> rad/s
  accel turn-on bound    (v * 1e-3) * G         mg -> m/s^2
  odr_error              v * 1e-6               ppm -> dimensionless

DEG = pi / 180 (the double nearest pi, divided by 180). G = 9.80665 m/s^2, standard gravity, exact by definition
(1901 3rd CGPM; SI Brochure 9th ed.); the profile's reading of "g" as standard gravity is INFERRED. The prefixes 1e-3
and 1e-6 are the SI milli and micro. Floating values in the header are C hexadecimal floating literals (exact); the
decimal repr of the double is in the trailing comment.

Accel noise density follows the configured range (decision 0012, owner decision A): the profile entry is a list keyed
by full-scale range, one position per range in ACCEL_RANGES_G ("<= 8 / 16 / 32 g", DS-000577 rev 1.0 §3.2 Table 2; the
profile's note on accel_noise_density states the keying, INFERRED from that note since the entry has no key field). The
range is accel_fsr's value (the profile assumes +-32 g); it must equal one of ACCEL_RANGES_G exactly, else the generator
refuses. There is no nearest-range rule.

World element (decision 0019): imu_model_element(si, meta, signs) is the <imu_model> child of the lockstep plugin element
(tools/card/gen_world.py writes it) over the same SI values: latency_samples; <gyro> and <accel> each with noise_density,
bias_instability, lsb, full_scale and turn_on_bias_bound; turn_on_bias_signs (six integers in {-1, 0, +1}, gyro x y z
then accel x y z; the plugin puts s_i * bound in the axes, imu_corner_config's rule); profile_sha256. Floating values are
Python repr (the shortest decimal that reads back as the identical binary64; SDFormat cannot parse hex floats).

K and sigma_d are not emitted; the report applies the plant's own rules (sim/plant/src/imu_model.hpp):
  tau* = (N / B)^2,  K = (sqrt(6) / 2) B^2 / N,  sigma_d = N sqrt(f_s / 2),  f_s = 1e6 * tick_period_den /
  tick_period_num_us (design/scenario_values.yaml).
"""

from __future__ import annotations

import argparse
import hashlib
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import lint
import schema

ROOT = lint.ROOT
SCENARIO_FILE = Path("design") / "scenario_values.yaml"

# Standard acceleration of gravity, m/s^2: exact by definition (1901 3rd CGPM; SI Brochure 9th ed.).
G = 9.80665
DEG = math.pi / 180.0
MILLI = 1e-3
MICRO = 1e-6
# Full-scale ranges, in g, of the positions of accel_noise_density (DS-000577 rev 1.0 §3.2 Table 2: "<= 8 / 16 / 32 g").
ACCEL_RANGES_G = (8, 16, 32)

IMU_PATH = "classes.imu.entries"


class GenError(Exception):
    """Refusal to generate; `lines` are the findings, one per line."""

    def __init__(self, lines):
        super().__init__("; ".join(lines))
        self.lines = list(lines)


# name -> expected unit. A different unit text is a refusal.
UNITS = {
    "gyro_noise_density": "mdeg/s/sqrt(Hz)",
    "gyro_bias_instability": "deg/s",
    "gyro_fifo_sensitivity": "LSB/(deg/s)",
    "gyro_fsr": "deg/s",
    "gyro_zero_rate_offset": "deg/s",
    "accel_noise_density": "ug/sqrt(Hz)",
    "accel_bias_instability": "ug",
    "accel_fifo_sensitivity": "LSB/g",
    "accel_fsr": "g",
    "accel_offset": "mg",
    "odr_error": "ppm",
    "latency_samples": "1",
}


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _number(entries, name, file, findings, *, positive):
    """The finite number of entry `name`, or None with a finding."""
    path = f"{IMU_PATH}.{name}"
    entry = entries.get(name)
    if not isinstance(entry, dict):
        findings.add(file, path, "missing (the IMU configuration needs it)")
        return None
    if entry.get("unit") != UNITS[name]:
        findings.add(file, path, f"unit is {entry.get('unit')!r}, expected {UNITS[name]!r}")
        return None
    v = entry.get("value")
    if v == schema.UNKNOWN:
        findings.add(file, path, f"value is {schema.UNKNOWN}; the IMU configuration needs it")
        return None
    if isinstance(v, list):
        return v
    if not _is_num(v) or v < 0 or (positive and v == 0):
        findings.add(file, path, f"value must be a finite number {'> 0' if positive else '>= 0'}, got {v!r}")
        return None
    return float(v)


def _scalar(entries, name, file, findings, *, positive):
    v = _number(entries, name, file, findings, positive=positive)
    if isinstance(v, list):
        findings.add(file, f"{IMU_PATH}.{name}", "value must be a single number, got a list")
        return None
    return v


def _accel_noise_density(entries, fsr_g, file, findings):
    """The accel noise density (ug/sqrt(Hz)) at the configured range accel_fsr (rule: module docstring)."""
    path = f"{IMU_PATH}.accel_noise_density"
    v = _number(entries, "accel_noise_density", file, findings, positive=False)
    if v is None:
        return None
    if not isinstance(v, list) or len(v) != len(ACCEL_RANGES_G):
        findings.add(file, path, f"value must be a list of {len(ACCEL_RANGES_G)} numbers, one per range "
                                 f"{list(ACCEL_RANGES_G)} g, got {v!r}")
        return None
    if not all(_is_num(x) and x >= 0 for x in v):
        findings.add(file, path, f"every list position must be a finite number >= 0 (or the entry is {schema.UNKNOWN}), "
                                 f"got {v!r}")
        return None
    if fsr_g is None:
        return None
    if fsr_g not in ACCEL_RANGES_G:
        findings.add(file, path, f"accel_fsr {fsr_g!r} g matches no list position {list(ACCEL_RANGES_G)} g; refusing to "
                                 "pick a range")
        return None
    return float(v[ACCEL_RANGES_G.index(fsr_g)])


def _provenance(entry):
    def one_line(x):
        return re.sub(r"\s+", " ", str(x)).replace("*/", "* /").strip()

    status = entry.get("status", [])
    return (f"source: {one_line(entry.get('source'))}; method: {one_line(entry.get('method'))}; status: "
            f"{one_line(', '.join(status) if isinstance(status, list) else status)}")


def imu_config(profile_path):
    """(si, prov, meta): si is the ordered dict of SI values the header carries, prov maps each to its profile entry
    name(s) and provenance text, meta holds the profile id, its SHA-256 and the accel range. Raises GenError."""
    profile_path = Path(profile_path)
    out = schema.Findings()
    lint.lint_profile(profile_path, out)
    if out:
        raise GenError(out.lines())
    doc = schema.load_yaml(profile_path)
    entries = doc["classes"]["imu"]["entries"]
    f, fl = profile_path, schema.Findings()
    raw = {n: _scalar(entries, n, f, fl, positive=pos) for n, pos in (
        ("gyro_noise_density", False), ("gyro_bias_instability", False), ("gyro_fifo_sensitivity", True),
        ("gyro_fsr", True), ("gyro_zero_rate_offset", False), ("accel_bias_instability", False),
        ("accel_fifo_sensitivity", True), ("accel_fsr", True), ("accel_offset", False), ("odr_error", False))}
    raw["accel_noise_density"] = _accel_noise_density(entries, raw["accel_fsr"], f, fl)
    lat = _scalar(entries, "latency_samples", f, fl, positive=False)
    if lat is not None and lat != int(lat):
        fl.add(f, f"{IMU_PATH}.latency_samples", f"value must be a whole number of samples, got {lat!r}")
        lat = None
    if fl:
        raise GenError(fl.lines())
    r = raw
    si = {
        "latency_samples": int(lat),
        "gyro_noise_density": (r["gyro_noise_density"] * MILLI) * DEG,
        "gyro_bias_instability": r["gyro_bias_instability"] * DEG,
        "gyro_lsb": DEG / r["gyro_fifo_sensitivity"],
        "gyro_full_scale": r["gyro_fsr"] * DEG,
        "accel_noise_density": (r["accel_noise_density"] * MICRO) * G,
        "accel_bias_instability": (r["accel_bias_instability"] * MICRO) * G,
        "accel_lsb": G / r["accel_fifo_sensitivity"],
        "accel_full_scale": r["accel_fsr"] * G,
        "gyro_turn_on_bias_bound": r["gyro_zero_rate_offset"] * DEG,
        "accel_turn_on_bias_bound": (r["accel_offset"] * MILLI) * G,
        "odr_error": r["odr_error"] * MICRO,
    }
    names = {
        "latency_samples": ("latency_samples",),
        "gyro_noise_density": ("gyro_noise_density",),
        "gyro_bias_instability": ("gyro_bias_instability",),
        "gyro_lsb": ("gyro_fifo_sensitivity",),
        "gyro_full_scale": ("gyro_fsr",),
        "accel_noise_density": ("accel_noise_density", "accel_fsr"),
        "accel_bias_instability": ("accel_bias_instability",),
        "accel_lsb": ("accel_fifo_sensitivity",),
        "accel_full_scale": ("accel_fsr",),
        "gyro_turn_on_bias_bound": ("gyro_zero_rate_offset",),
        "accel_turn_on_bias_bound": ("accel_offset",),
        "odr_error": ("odr_error",),
    }
    prov = {k: [(n, _provenance(entries[n])) for n in ns] for k, ns in names.items()}
    meta = {"profile": doc["profile"], "sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
            "accel_range_g": r["accel_fsr"]}
    return si, prov, meta


def derived_report(si, tick_period_num_us, tick_period_den):
    """{sensor: (N, B, tau_star, K, sigma_d)} from the SI values (rules: module docstring)."""
    fs = 1e6 * tick_period_den / tick_period_num_us
    rep = {}
    for s in ("gyro", "accel"):
        n, b = si[f"{s}_noise_density"], si[f"{s}_bias_instability"]
        rep[s] = (n, b, (n / b) ** 2, (math.sqrt(6.0) / 2.0) * b * b / n, n * math.sqrt(fs / 2.0))
    return rep, fs


def _hex(x):
    return float.hex(float(x))


def render_header(si, prov, meta, profile_name):
    def lit(key):
        return f"{_hex(si[key])}"

    def comment(key, indent="  "):
        ps = "; ".join(f"profile {meta['profile']}, entry {n}, {p}" for n, p in prov[key])
        return f"{indent}// {key}: {si[key]!r}. {ps}" if key != "latency_samples" else f"{indent}// {key}: {ps}"

    lines = [
        f"// Generated by tools/card/gen_imu_config.py from {profile_name} (profile {meta['profile']}, "
        f"sha256 {meta['sha256']}).",
        "// Build directory only, never committed. Edit the profile, not this file.",
        "//",
        "// SI units, FRD axes. Floating values are C hexadecimal floating literals (exact); the decimal repr of the",
        "// double is in the comment. The accel noise density is the profile's value at the range accel_fsr "
        f"({meta['accel_range_g']!r} g).",
        "// K and sigma_d are not emitted: the plant's IMU model derives them from noise_density and bias_instability.",
        "// turn_on_bias is 0 (nominal) in imu_profile_config(); the per-axis bounds are separate constants and",
        "// imu_corner_config(s) puts s_i * bound_i in the six axes, s_i in {-1, 0, +1}, order gyro x y z, accel x y z.",
        "#pragma once",
        "",
        "#include <array>",
        "#include <optional>",
        "",
        '#include "marv_plant.h"',
        "",
        "namespace marv::sim {",
        "",
        "// Dimensionless fraction (the profile's ppm times 1e-6): the IMU's ODR error against the plant clock. It belongs",
        "// to the adapter's clock map (AdapterConfig::odr_error), not to the plant configuration.",
        comment("odr_error", ""),
        f"inline constexpr double kImuOdrError = {lit('odr_error')};",
        "",
        "// Turn-on bias bounds, one value per axis (a limit, not a sigma), rad/s and m/s^2.",
        comment("gyro_turn_on_bias_bound", ""),
        f"inline constexpr double kImuGyroTurnOnBiasBound = {lit('gyro_turn_on_bias_bound')};",
        comment("accel_turn_on_bias_bound", ""),
        f"inline constexpr double kImuAccelTurnOnBiasBound = {lit('accel_turn_on_bias_bound')};",
        "",
        "// The nominal configuration: every figure of the profile, turn_on_bias 0.",
        "inline constexpr marv_plant_imu_config imu_profile_config() {",
        "  marv_plant_imu_config c{};",
        "  c.struct_size = sizeof(c);",
        comment("latency_samples"),
        f"  c.latency_samples = {si['latency_samples']}U;",
    ]
    for s, field in (("gyro", "noise_density"), ("gyro", "bias_instability"), ("gyro", "lsb"), ("gyro", "full_scale"),
                     ("accel", "noise_density"), ("accel", "bias_instability"), ("accel", "lsb"),
                     ("accel", "full_scale")):
        key = f"{s}_{field}"
        lines += [comment(key), f"  c.{s}.{field} = {lit(key)};"]
    lines += [
        "  return c;",
        "}",
        "",
        "// The configuration at a corner of the turn-on bias box: axis i gets s[i] * bound, s in {-1, 0, +1}^6 (gyro x y z,",
        "// accel x y z). Any other s[i] gives no configuration (an empty optional).",
        "inline constexpr std::optional<marv_plant_imu_config> imu_corner_config(const std::array<int, 6>& s) {",
        "  for (const int v : s) {",
        "    if (v < -1 || v > 1) {",
        "      return std::nullopt;",
        "    }",
        "  }",
        "  marv_plant_imu_config c = imu_profile_config();",
        "  for (unsigned i = 0; i < 3; ++i) {",
        "    c.gyro.turn_on_bias[i] = static_cast<double>(s[i]) * kImuGyroTurnOnBiasBound;",
        "    c.accel.turn_on_bias[i] = static_cast<double>(s[3 + i]) * kImuAccelTurnOnBiasBound;",
        "  }",
        "  return c;",
        "}",
        "",
        "}  // namespace marv::sim",
        "",
    ]
    return "\n".join(lines)


BIAS_SIGNS = (-1, 0, 1)
SIGN_COUNT = 6  # gyro x y z, then accel x y z (imu_corner_config's order)


def imu_model_element(si, meta, signs):
    """The <imu_model> element (module docstring) of the SI values `si` and `meta` of imu_config(), at the turn-on bias
    corner `signs`. Raises GenError if `signs` is not six values in {-1, 0, +1}."""
    signs = tuple(signs)
    if len(signs) != SIGN_COUNT or not all(isinstance(s, int) and not isinstance(s, bool) and s in BIAS_SIGNS
                                           for s in signs):
        raise GenError([f"turn_on_bias_signs must be {SIGN_COUNT} integers in {list(BIAS_SIGNS)}, got {list(signs)!r}"])

    def sub(parent, tag, text):
        e = ET.SubElement(parent, tag)
        e.text = text
        return e

    imu = ET.Element("imu_model")
    sub(imu, "latency_samples", str(si["latency_samples"]))
    for s in ("gyro", "accel"):
        axis = ET.SubElement(imu, s)
        for field in ("noise_density", "bias_instability", "lsb", "full_scale"):
            sub(axis, field, repr(float(si[f"{s}_{field}"])))
        sub(axis, "turn_on_bias_bound", repr(float(si[f"{s}_turn_on_bias_bound"])))
    sub(imu, "turn_on_bias_signs", " ".join(str(v) for v in signs))
    sub(imu, "profile_sha256", meta["sha256"])
    return imu


def render_report(si, meta, tick):
    rep, fs = derived_report(si, *tick)
    out = [f"IMU noise model figures (profile {meta['profile']}, sha256 {meta['sha256']}); f_s = {fs!r} Hz",
           "sensor  N [unit/sqrt(Hz)]  B [unit]  tau* = (N/B)^2 [s]  K = (sqrt(6)/2) B^2/N [unit/s/sqrt(Hz)]  "
           "sigma_d = N sqrt(f_s/2) [unit]"]
    for s, (n, b, tau, k, sd) in rep.items():
        out.append(f"{s:<6}  {n!r}  {b!r}  {tau!r}  {k!r}  {sd!r}")
    return "\n".join(out) + "\n"


def _tick(scenario_path):
    doc = schema.load_yaml(scenario_path)
    try:
        num, den = doc["tick_period_num_us"]["value"], doc["tick_period_den"]["value"]
    except (KeyError, TypeError) as e:
        raise GenError([f"{scenario_path}: tick_period_num_us / tick_period_den: missing ({e})"]) from e
    if not (_is_num(num) and _is_num(den) and num > 0 and den > 0):
        raise GenError([f"{scenario_path}: tick_period_num_us / tick_period_den: must be numbers > 0"])
    return num, den


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--profile", required=True, metavar="FILE")
    ap.add_argument("--out", required=True, metavar="FILE")
    ap.add_argument("--report", action="store_true", help="print K, tau* and sigma_d of both sensors on stdout")
    ap.add_argument("--scenario", default=str(ROOT / SCENARIO_FILE), metavar="FILE",
                    help="scenario register that holds the tick period (report only)")
    args = ap.parse_args(argv)
    try:
        si, prov, meta = imu_config(args.profile)
        report = render_report(si, meta, _tick(args.scenario)) if args.report else None
    except GenError as e:
        for line in e.lines:
            print(line, file=sys.stderr)
        return 1
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(render_header(si, prov, meta, Path(args.profile).name), encoding="utf-8")
    if report:
        sys.stdout.write(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
