"""The gyro chain's derived parameters (quad spec L6 stage (b), decision 0013), by rule, for flatten.py --out-gyro-chain.

The rules are gyro_chain_design.py's (lowpass_cutoff_hz, notch_q_set, omega_threshold, chain_design); this module only
reads the register and profile entries, applies the refusals and writes the params_gen entries and the derivation report.

  gyro_lpf_cutoff_hz     Hz     the low-pass cutoff f_c
  gyro_notch_q_h1/h2/h3  1      the notch quality factor per harmonic
  gyro_notch_omega_min   rad/s  omega_th, the rotor speed below which a notch is bypassed

Inputs: design-budget gyro_chain_attenuation_min (a_min) and esc_clock_error_max; scenario tick_period_num_us,
tick_period_den and rate_loop_divisor; the vehicle card's mass, rotors.thrust_coeff and rotors.speed_range; the sensor
profile's imu odr_error and rotor_speed esc_clock_error (unit %, converted to a fraction by / 100). The firmware's other
chain inputs, the tick period and the divisor, are scenario parameters already.

The generator refuses (gpc.GenError, naming the entry) when an input is missing, UNKNOWN or out of its domain, when the
profile's esc_clock_error exceeds esc_clock_error_max, or when a rule refuses (rate-loop divisor below 2, a notch at or
above the tick's Nyquist).
"""

from __future__ import annotations

import math

import gen_imu_config as gic
import gen_plant_config as gpc
import gyro_chain_design as gcd
import schema

PERCENT = 100.0
ESC_ENTRY = "classes.rotor_speed.entries.esc_clock_error"
ESC_UNIT = "%"
MANTISSA_ENTRY = "classes.rotor_speed.entries.telemetry_mantissa_bits"
SIGMA = schema.UNKNOWN

LPF_METHOD = ("derived(f_c = f_s/pi * atan(tan(pi f_r/(2 f_s)) / (1/a_min^2 - 1)^(1/4)), f_r = f_s/D: the digital gain of "
              "the second-order Butterworth at the rate loop's Nyquist f_r/2 equals a_min; tools/card/gyro_chain_design.py "
              "lowpass_cutoff_hz)")
Q_METHOD = ("derived(Q_h = min over the edges f0 (1 -+ eps) of a x / (sqrt(1 - a^2) |1 - x^2|), a = a_min, x = tan(pi f_edge/f_s) "
            "/ tan(pi f0/f_s), f0 = h omega_max / (2 pi): the largest Q whose digital notch, the prewarped bilinear transform "
            "of (s^2 + 1)/(s^2 + s/Q + 1) at f0, attenuates by at least a_min at f0 (1 +- eps); eps = 2^-(m - 1) (m = telemetry_mantissa_bits) + odr_error + "
            "esc_clock_error; tools/card/gyro_chain_design.py notch_q)")
OMEGA_METHOD = ("derived(omega_th = omega_hover sqrt(a_min), omega_hover = sqrt(m g / (4 k)) at standard gravity g = 9.80665 "
                "m/s^2 (the plant uses the site's WGS 84 gravity, under 1 % apart); a notch is active only at omega >= "
                "omega_th; tools/card/gyro_chain_design.py omega_threshold)")


def _refuse(where, path, reason):
    raise gpc.GenError([f"{where}: {path}: {reason}"])


def _value(doc, name, path, where, kind):
    entry = doc.get(name) if isinstance(doc, dict) else None
    if not isinstance(entry, dict) or "value" not in entry:
        _refuse(where, path, f"{kind} entry missing (the gyro chain needs it)")
    v = entry["value"]
    if v == schema.UNKNOWN:
        _refuse(where, path, f"value is {schema.UNKNOWN}; the gyro chain needs it")
    if not schema.is_number(v):
        _refuse(where, path, f"value must be a finite number, got {v!r}")
    return v


def telemetry_step(entries, profile_path):
    """The telemetry word's worst-case relative period resolution, 2^-(m - 1) for m mantissa bits. Rule: the encoder keeps
    the top m bits of the period (m << e), so a normalised mantissa lies in [2^(m-1), 2^m) and one step is at most 2^-(m-1) of
    the value; the rule is the worst case (smallest normalised mantissa). Truncation makes the error one-sided, never larger
    than this step."""
    m = _value(entries, "telemetry_mantissa_bits", MANTISSA_ENTRY, profile_path, "profile")
    if not (isinstance(m, int) and m >= 2):
        _refuse(profile_path, MANTISSA_ENTRY, f"value must be an integer >= 2, got {m!r}")
    return 2.0 ** -(m - 1)


def design(card, budget, scenario, profile, profile_path, where):
    """The chain's design dict (raises gpc.GenError on refusal): gyro_chain_design.chain_design's, plus the inputs."""
    a_min = float(_value(budget, "gyro_chain_attenuation_min", "gyro_chain_attenuation_min", where, "budget"))
    esc_max = float(_value(budget, "esc_clock_error_max", "esc_clock_error_max", where, "budget"))
    entries = profile["classes"]["rotor_speed"]["entries"]
    esc_entry = entries.get("esc_clock_error")
    if not isinstance(esc_entry, dict):
        _refuse(profile_path, ESC_ENTRY, "missing (the gyro chain needs it)")
    if esc_entry.get("unit") != ESC_UNIT:
        _refuse(profile_path, ESC_ENTRY, f"unit is {esc_entry.get('unit')!r}, expected {ESC_UNIT!r}")
    esc_raw = _value(entries, "esc_clock_error", ESC_ENTRY, profile_path, "profile")
    esc = float(esc_raw) / PERCENT
    if esc < 0:
        _refuse(profile_path, ESC_ENTRY, f"value {esc_raw!r} is negative")
    if esc > esc_max:
        _refuse(profile_path, ESC_ENTRY, f"{esc_raw!r} {ESC_UNIT} (= {esc!r}) exceeds the requirement on any ESC, "
                                         f"esc_clock_error_max {esc_max!r} (design/budget.yaml)")
    step = telemetry_step(entries, profile_path)
    for name in ("tick_period_num_us", "tick_period_den", "rate_loop_divisor"):
        _value(scenario, name, name, where, "scenario")
    _value(card, "mass", "mass", where, "card")
    _value(card["rotors"], "thrust_coeff", "rotors.thrust_coeff", where, "card")
    rng = card["rotors"]["speed_range"]["value"]
    if rng == schema.UNKNOWN:
        _refuse(where, "rotors.speed_range", f"value is {schema.UNKNOWN}; the gyro chain needs it")
    try:
        si, _, _ = gic.imu_config(profile_path)
        d = gcd.chain_design(None, card, scenario, si, a_min, esc, step)
    except gic.GenError as e:
        raise gpc.GenError(e.lines) from e
    except gcd.DesignError as e:
        _refuse(where, "gyro chain", str(e))
    d.update(telemetry_step=step, a_min=a_min, esc_clock_error=esc, esc_clock_error_max=esc_max, odr_error=si["odr_error"],
             mass=card["mass"]["value"], thrust_coeff=card["rotors"]["thrust_coeff"]["value"])
    return d


def entries_from(d):
    inputs = (f"design-budget gyro_chain_attenuation_min (a_min {d['a_min']!r}); scenario register tick_period_num_us / "
              f"tick_period_den (f_s {d['fs']!r} Hz) and rate_loop_divisor (D {d['divisor']!r})")
    q_inputs = (f"design-budget gyro_chain_attenuation_min (a_min {d['a_min']!r}); vehicle card rotors.speed_range maximum "
                f"(omega_max {d['omega_max']!r} rad/s); scenario register tick_period_num_us / tick_period_den (f_s "
                f"{d['fs']!r} Hz); sensor profile imu odr_error ({d['odr_error']!r}) and rotor_speed esc_clock_error "
                f"({d['esc_clock_error']!r}, at most design-budget esc_clock_error_max {d['esc_clock_error_max']!r}); "
                f"sensor profile rotor_speed telemetry_mantissa_bits (step 2^-(m-1) = {d['telemetry_step']!r}); eps {d['eps']!r}")
    omega_inputs = (f"vehicle card mass ({d['mass']!r} kg) and rotors.thrust_coeff ({d['thrust_coeff']!r}); design-budget "
                    f"gyro_chain_attenuation_min (a_min {d['a_min']!r}); omega_hover {d['omega_hover']!r} rad/s")
    out = [("gyro_lpf_cutoff_hz", {"type": "f32", "value": d["f_c"], "unit": "Hz", "method": LPF_METHOD,
                                   "source": inputs, "sigma": SIGMA})]
    for h, q in zip(gcd.HARMONICS, d["q"]):
        out.append((f"gyro_notch_q_h{h}", {"type": "f32", "value": q, "unit": "1", "method": Q_METHOD,
                                          "source": q_inputs, "sigma": SIGMA}))
    out.append(("gyro_notch_omega_min", {"type": "f32", "value": d["omega_th"], "unit": "rad/s", "method": OMEGA_METHOD,
                                         "source": omega_inputs, "sigma": SIGMA}))
    return out


def report_text(d, where):
    return [
        "MARV L6 stage (b) gyro-chain parameter derivation (tools/card/gyro_chain_params.py, decision 0013)",
        f"card                       {where}",
        f"a_min                      {d['a_min']!r}",
        f"f_s (Hz), D                {d['fs']!r}  {d['divisor']!r}",
        f"omega_max (rad/s)          {d['omega_max']!r}",
        f"odr_error                  {d['odr_error']!r}",
        f"esc_clock_error            {d['esc_clock_error']!r}  (requirement esc_clock_error_max {d['esc_clock_error_max']!r})",
        f"telemetry step 2^-(m-1)   {d['telemetry_step']!r}",
        f"eps = step + odr + esc     {d['eps']!r}",
        f"omega_hover (rad/s)        {d['omega_hover']!r}  (standard gravity {gic.G!r} m/s^2)",
        f"gyro_lpf_cutoff_hz         {d['f_c']!r}",
        "gyro_notch_q_h1, _h2, _h3  " + "  ".join(f"{q!r}" for q in d["q"]),
        f"gyro_notch_omega_min       {d['omega_th']!r}  ({d['omega_th'] / (2 * math.pi)!r} Hz)",
    ]


def design_header_text(d, where):
    """The C++ header of the chain's relative tracking error eps and its three terms (for tests; no firmware parameter).
    Every figure is the design's own (design(): the same step, odr_error and esc_clock_error that set Q), written at
    full double precision, so a test checks the Q that ships against the figure that produced it."""
    rule = (f"epsilon (rule: 2^-(m - 1) + odr_error + esc_clock_error, gyro_chain_design.py epsilon; derivation of gyro_notch_q_h* "
            f"in {where})")
    return "\n".join([
        "// Generated by tools/card/gyro_chain_params.py through tools/card/flatten.py (decision 0013). Do not edit.",
        "// The gyro chain's relative notch-tracking error eps and its three terms, as the generator used them for",
        "// gyro_notch_q_h1..h3. Not firmware parameters: for tests that must use the figure that set Q.",
        "#pragma once",
        "",
        "namespace marv::gyro_chain_design {",
        "",
        "// Telemetry period resolution, 2^-(m - 1), m = sensor profile rotor_speed telemetry_mantissa_bits (rule: the worst-case",
        "// relative step of the encoder's m-bit mantissa; the encoder truncates, so the error is one-sided and at most this).",
        f"inline constexpr double kGyroChainTelemetryStep = {d['telemetry_step']!r};",
        "// Relative ODR error, sensor profile imu odr_error (a fraction).",
        f"inline constexpr double kGyroChainOdrError = {d['odr_error']!r};",
        "// ESC clock error, sensor profile rotor_speed esc_clock_error (the flown part's figure; unit % in the profile, / 100 here;",
        "// the profile lint requires it not to exceed design-budget esc_clock_error_max).",
        f"inline constexpr double kGyroChainEscClockError = {d['esc_clock_error']!r};",
        f"// {rule}",
        f"inline constexpr double kGyroChainEpsilon = {d['eps']!r};",
        "",
        "}  // namespace marv::gyro_chain_design",
        ""])


def gyro_chain_entries(card, budget, scenario, profile, profile_path, where):
    """(ordered (name, params_gen entry) list, report lines, design dict); raises gpc.GenError on refusal."""
    d = design(card, budget, scenario, profile, profile_path, where)
    return entries_from(d), report_text(d, where), d
