#!/usr/bin/env python3
"""Gazebo world generator for the quad L2 lockstep plugin (docs/decisions/0003 items 4, 5, 7, 9, 10).

  gen_world.py --card <vehicle>.yaml --scenario <scenario>.yaml --mode pilot|test --m <int>
               [--hover lo|hi] [--log-path <path>] [--attitude-source] --out-dir <dir> [--root <repo root>]

Writes <dir>/<vehicle>_<scenario>_<mode>_m<m>.sdf (build directory only, never committed) from the linted card
(gen_sdf.py, imported and reused, not copied) and a validated scenario (tools/sim/scenario.py). For a hover scenario
the scenario name in the file name is hover_lo or hover_hi, the bracket member of item 7. An unlinted card, an invalid
scenario, an m outside the scenario's m_sequence or an illegal hover bracket generates nothing (exit 1, findings on
stderr). The number of host steps to run is scenario.iterations(doc, m) = duration_ticks / m: pass it as
`gz sim -s -r --iterations N`.

Numbers are written round-trip exact: every floating value is Python repr(float), the shortest decimal that float()
reads back as the identical binary64 (not hex-float, which SDFormat cannot parse); integers are decimal. The step
size is the exact rational m * num_us / (den * 10^6) s rounded once to binary64. The vectors of the NED/FRD to
ENU/FLU pose map are signed permutations (unary negation only, so exact, -0.0 included); the attitude is not exact
(0003 item 6): q_enu_flu = q_ned2enu (x) q_ned_frd (x) q_frd2flu with q_ned2enu = (0, s, s, 0), s = sqrt(1/2)
(180 degrees about (1,1,0)/sqrt 2, v -> (y, x, -z)) and q_frd2flu = (0, 1, 0, 0) (180 degrees about x), evaluated in
closed form as (-s(w+z), -s(x+y), s(y-x), s(z-w)) for input (w, x, y, z), sign made canonical (w >= 0).

World (SDFormat 1.11, the specification version of sdformat14, the library of gz-sim 8):

  <world name="<file stem>">
    <physics name="marv_physics" type="dart">
      <max_step_size>            m * tick period, seconds
      <real_time_factor>         1 in pilot mode, 0 in test mode (uncapped)
    <gravity>0 0 0</gravity>     world gravity is zero in every host (core section 6); marv_plant applies g
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
      <engine><filename>gz-physics-dartsim-plugin</filename></engine>
    <model name="<vehicle>">     the gen_sdf.py model; <pose rotation_format="quat_xyzw"> added as its first child
    No ground plane and no contact geometry: no L2 scenario needs contact (free fall, rotation and hover are in air).

  Engine selection is third-party behaviour, INFERRED from strings in libgz-sim8-physics-system.so ("engine",
  "filename", "gz-physics-dartsim-plugin", GZ_SIM_PHYSICS_ENGINE_PATH) and ServerConfig.hh (PhysicsEngine is the
  physics engine plugin library to load); the type attribute of <physics> is not what selects it. Observed once, in
  marv-ci-gz (gz-sim 8.15.0, gz-physics 7.8.0, DART 6.13.2), not committed as a test: this <engine><filename> loads
  "gz::physics::dartsim::Plugin" from libgz-physics-dartsim-plugin.so, and the name gz-physics-bogus-plugin fails with
  "Failed to find plugin [gz-physics-bogus-plugin]".

Plugin <plugin filename="marv_gz_lockstep" name="marv::gz::Lockstep">, a child of the model. Children, in order.
Card children come from gen_plant_config.plugin_element() unchanged; scenario children follow.

  element                       type / unit    source     meaning
  ----------------------------  -------------  ---------  ---------------------------------------------------
  esc_map (attr model)          int            card       marv_plant_config.esc_map
  pole_count                    int            card       marv_plant_config.pole_count (0 = unknown)
  mass_kg, thrust_coeff,        float, units   card       the like-named marv_plant_config fields
    torque_ratio_m,             as attribute
    omega_min_rad_s,
    omega_max_rad_s, motor_tau_s
  rotor (attr motor 1..4)       -              card       children position_frd_m (3 floats, m) and yaw_sign (+1/-1)
  site_lat_rad                  float, rad     scenario   marv_plant_config.site_lat_rad
  site_height_m                 float, m       scenario   marv_plant_config.site_height_m (h0 of the NED origin)
  motor_substep_s               float, s       scenario   marv_plant_config.motor_substep_s = the tick period, exact
  ticks_per_step                int >= 1       scenario   m: ticks per host step (max_step_size = m * tick)
  tick_period_num_us            int > 0        scenario   tick period = num_us / den microseconds, exactly
  tick_period_den               int > 0        scenario
  seed                          int >= 0       scenario   marv_plant_config.rng_seed; reserved, unused at v0
  sil_override (attrs param,    text           scenario   one per motor: param ol_dshot_m1..m4, type i32, value
    type)                                                 the DShot command; applied through the SIL override before
                                                          the first tick
  initial_velocity_ned_m_s      3 floats, m/s  scenario   OPTIONAL, present iff nonzero; default 0 0 0
  initial_body_rates_frd        3 floats, rad/s scenario  OPTIONAL, present iff nonzero; default 0 0 0 (rotation only)
  initial_rotor_speed_rad_s     4 floats, rad/s scenario  OPTIONAL, present iff the scenario has initial_state.
                                                          rotor_speed_rad_s (decision 0007); logical motor order;
                                                          marv_plant_config.initial_omega_rad_s; default all 0
  attitude_source               text "truth"   --attitude-source OPTIONAL, present iff given (quad L5, decision 0006
                                                          section B); the plugin refuses it without <gyro_source>truth
                                                          </gyro_source> (which this generator does not write) and with
                                                          a SIL library that has no marv_truth_state_set
  gyro_source                   text "model"   sensors    OPTIONAL (decision 0019), present iff sensors.gyro == "model":
                                                          the SIL gets the plant IMU model's bytes, gyro and accel
  imu_model                     block          profile    present iff gyro_source model: gen_imu_config.py
                                                          imu_model_element() over the card's sensor profile, at the
                                                          turn-on bias corner sensors.bias_signs
  rotor_speed_model             block          profile    OPTIONAL, present iff sensors.rotor: latency_ticks (the
                                                          profile's latency_rate_periods x the register's
                                                          rate_loop_divisor), exponent_bits, mantissa_bits, period_unit_s
                                                          (the telemetry grid); the pole count is <pole_count>
  clock_corner, odr_error       int, float     sensors,   OPTIONAL, present iff sensors.clock_corner is not None: c in
                                               profile    {-1, 0, +1} and the profile's ODR error e; max_step_size is
                                                          then fl(n_true / 1e9) with n_true = outward(m tick_ns / (1 + c e))
                                                          ns, exact rationals (floor for c = +1, ceil for c = -1), the
                                                          plugin's realised host step (realised_host_step_ns)
  log_path                      text           --log-path OPTIONAL, present iff given; where the plugin writes its
                                                          binary log; no log if absent. Never written into the log.

  sensors (generate(..., sensors=SensorSet(...)), decision 0019): None writes none of the four sensor elements and leaves
  every other byte of the world as without the argument. With it, the world differs from that default world only by the
  inserted elements, and by max_step_size iff clock_corner is nonzero. The plugin refuses a clock corner without the
  model gyro (outside its test-only allow) and a rotor-speed model without the model gyro; the generator refuses the
  latter too.

  The initial position and attitude are the model <pose> (ENU / FLU, above); the plugin converts them back once.
  Motor numbers are logical, 1 to 4 (core section 3).
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import sys
import xml.etree.ElementTree as ET
from fractions import Fraction
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[0] / "sim"))
import gen_imu_config as gic  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import gen_sdf  # noqa: E402
import hover  # noqa: E402
import lint  # noqa: E402
import scenario as scn  # noqa: E402
import schema  # noqa: E402

SDF_VERSION = gen_sdf.SDF_VERSION
MODES = ("pilot", "test")
RTF = {"pilot": 1.0, "test": 0.0}
PHYSICS_SYSTEM_FILENAME = "gz-sim-physics-system"
PHYSICS_SYSTEM_NAME = "gz::sim::systems::Physics"
PHYSICS_ENGINE_FILENAME = "gz-physics-dartsim-plugin"
PHYSICS_NAME = "marv_physics"
PHYSICS_TYPE = "dart"
ZERO_GRAVITY = "0 0 0"
POSE_FORMAT = "quat_xyzw"
DSHOT_PARAM = "ol_dshot_m{}"
DSHOT_TYPE = "i32"
ATTITUDE_SOURCE_TRUTH = "truth"
GYRO_SOURCE_MODEL = "model"
CLOCK_CORNERS = (-1, 0, 1)
SCENARIO_REGISTER = Path("design") / "scenario_values.yaml"
# Nanoseconds per microsecond and per second: SI prefixes nano = 10^-9, micro = 10^-6 (BIPM, SI Brochure 9th ed. (2019),
# Table 7).
NS_PER_US = 1000
NS_PER_S = 10 ** 9
# The rotor-speed entries of the sensor profile the plugin's <rotor_speed_model> takes, with the unit each must have.
ROTOR_SPEED_ENTRIES = {"telemetry_exponent_bits": "1", "telemetry_mantissa_bits": "1", "telemetry_period_unit": "us",
                       "latency_rate_periods": "1"}
S = math.sqrt(0.5)


@dataclasses.dataclass(frozen=True)
class SensorSet:
    """The sensor elements of a world (decision 0019; module docstring). gyro: "model" or None (no <gyro_source> is
    written; the caller's own, if any, stands). rotor: write <rotor_speed_model> (needs the model gyro). bias_signs: the
    turn-on bias corner, six integers in {-1, 0, +1}, gyro x y z then accel x y z (with the model gyro only). clock_corner:
    None (no clock element) or -1, 0, +1."""
    gyro: str | None = None
    rotor: bool = False
    bias_signs: tuple | None = None
    clock_corner: int | None = None


def check_sensors(sensors):
    """The findings of a SensorSet, [] if it is valid."""
    bad = []
    if sensors.gyro not in (None, GYRO_SOURCE_MODEL):
        bad.append(f"sensors.gyro must be None or {GYRO_SOURCE_MODEL!r}, not {sensors.gyro!r}")
    if not isinstance(sensors.rotor, bool):
        bad.append(f"sensors.rotor must be a bool, not {sensors.rotor!r}")
    if sensors.rotor and sensors.gyro != GYRO_SOURCE_MODEL:
        bad.append("sensors.rotor needs sensors.gyro == 'model'")
    if (sensors.gyro == GYRO_SOURCE_MODEL) != (sensors.bias_signs is not None):
        bad.append("sensors.bias_signs is given iff sensors.gyro == 'model'")
    signs = sensors.bias_signs
    if signs is not None and (len(signs) != gic.SIGN_COUNT or not all(
            isinstance(v, int) and not isinstance(v, bool) and v in gic.BIAS_SIGNS for v in signs)):
        bad.append(f"sensors.bias_signs must be {gic.SIGN_COUNT} integers in {list(gic.BIAS_SIGNS)}, not {signs!r}")
    c = sensors.clock_corner
    if c is not None and (isinstance(c, bool) or c not in CLOCK_CORNERS):
        bad.append(f"sensors.clock_corner must be None or one of {list(CLOCK_CORNERS)}, not {c!r}")
    return bad


def realised_host_step_ns(doc, m, corner, odr_error):
    """n_true = outward(m tick_ns / (1 + corner e)) ns in exact rationals, e the binary64 odr_error exactly: floor for
    corner +1, ceil for corner -1, m tick_ns for corner 0. tick_ns = num_us 1000 / den must be an integer."""
    vals = scn.values(doc)
    tick_ns = Fraction(vals["tick_period_num_us"] * NS_PER_US, vals["tick_period_den"])
    if tick_ns.denominator != 1:
        raise gpc.GenError([f"the tick period {tick_ns} ns is not a whole number of nanoseconds"])
    n = m * tick_ns.numerator
    exact = n / (1 + corner * Fraction(odr_error))
    return n if corner == 0 else (math.floor(exact) if corner > 0 else math.ceil(exact))


def rotor_speed_values(profile, card_path, root):
    """{latency_ticks, exponent_bits, mantissa_bits, period_unit_s} of <rotor_speed_model> from the card's sensor profile
    and the scenario register's rate_loop_divisor. Raises gpc.GenError on a missing, UNKNOWN, mis-united or non-integer
    entry."""
    entries = profile["classes"]["rotor_speed"]["entries"]
    raw, bad = {}, []
    for name, unit in ROTOR_SPEED_ENTRIES.items():
        e = entries.get(name)
        v = e.get("value") if isinstance(e, dict) else None
        if not isinstance(e, dict) or e.get("unit") != unit or isinstance(v, bool) or not isinstance(v, int) or v <= 0:
            bad.append(f"{card_path}: profile rotor_speed.entries.{name}: must be a whole number > 0 in unit {unit!r} "
                       f"(the plugin's <rotor_speed_model> needs it), got {e!r}")
        raw[name] = v
    reg = schema.load_yaml(Path(root) / SCENARIO_REGISTER).get("rate_loop_divisor", {})
    div = reg.get("value") if isinstance(reg, dict) else None
    if isinstance(div, bool) or not isinstance(div, int) or div <= 0:
        bad.append(f"{SCENARIO_REGISTER}: rate_loop_divisor: must be a whole number > 0, got {div!r}")
    if bad:
        raise gpc.GenError(bad)
    return {"latency_ticks": raw["latency_rate_periods"] * div, "exponent_bits": raw["telemetry_exponent_bits"],
            "mantissa_bits": raw["telemetry_mantissa_bits"], "period_unit_s": raw["telemetry_period_unit"] * gic.MICRO}


def ned_to_enu(v):
    """(x, y, z) NED -> (y, x, -z) ENU. Exact (a signed permutation)."""
    return (v[1], v[0], -v[2])


def quat_ned_frd_to_enu_flu(q):
    """(w, x, y, z) body FRD -> NED to body FLU -> ENU, closed form of the module docstring, canonical sign w >= 0."""
    w, x, y, z = q
    out = (-(S * (w + z)), -(S * (x + y)), S * (y - x), S * (z - w))
    if out[0] < 0:
        out = tuple(-c for c in out)
    return out


def pose_text(pos_ned, q_wxyz):
    e = ned_to_enu(pos_ned)
    w, x, y, z = quat_ned_frd_to_enu_flu(q_wxyz)
    return " ".join(repr(float(c)) for c in (*e, x, y, z, w))


def _fmt3(v):
    return " ".join(repr(float(c)) for c in v)


def _world_stem(vehicle, name, mode, m):
    return f"{vehicle}_{name}_{mode}_m{m}"


def world_element(card, cfg, units, doc, mode, m, dshot, name, log_path, attitude_source=False, sensors=None,
                  sensor_values=None):
    vals = scn.values(doc)
    tick = scn.tick_period_s(doc)
    stem = _world_stem(card["vehicle"], name, mode, m)
    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    world = ET.SubElement(sdf, "world", {"name": stem})
    world.append(ET.Comment(
        f" Generated by tools/card/gen_world.py from the vehicle card {card['vehicle']} and the scenario {name}, "
        f"mode {mode}, m = {m}. SDFormat {SDF_VERSION}. Gravity is zero (core section 6). No ground plane: no L2 "
        "scenario needs contact. "))
    physics = ET.SubElement(world, "physics", {"name": PHYSICS_NAME, "type": PHYSICS_TYPE})
    if sensors is not None and sensors.clock_corner is not None:
        n_true = realised_host_step_ns(doc, m, sensors.clock_corner, sensor_values["imu"][0]["odr_error"])
        gpc.text_element(physics, "max_step_size", repr(float(Fraction(n_true, NS_PER_S))))
    else:
        gpc.text_element(physics, "max_step_size", repr(float(m * tick)))
    gpc.text_element(physics, "real_time_factor", repr(RTF[mode]))
    gpc.text_element(world, "gravity", ZERO_GRAVITY)
    system = ET.SubElement(world, "plugin", {"filename": PHYSICS_SYSTEM_FILENAME, "name": PHYSICS_SYSTEM_NAME})
    engine = ET.SubElement(system, "engine")
    gpc.text_element(engine, "filename", PHYSICS_ENGINE_FILENAME)

    model = gen_sdf.sdf_element(card, cfg, units).find("model")
    st = vals["initial_state"]
    pose = ET.Element("pose", {"rotation_format": POSE_FORMAT})
    pose.text = pose_text(st["position_ned_m"], st["attitude_q_wxyz"])
    model.insert(0, pose)
    world.append(model)

    plugin = model.find("plugin")
    gpc.text_element(plugin, "site_lat_rad", repr(float(vals["site_latitude_rad"])), "rad")
    gpc.text_element(plugin, "site_height_m", repr(float(vals["site_height_m"])), "m")
    gpc.text_element(plugin, "motor_substep_s", repr(float(tick)), "s")
    gpc.text_element(plugin, "ticks_per_step", str(m))
    gpc.text_element(plugin, "tick_period_num_us", str(vals["tick_period_num_us"]), "us")
    gpc.text_element(plugin, "tick_period_den", str(vals["tick_period_den"]))
    gpc.text_element(plugin, "seed", str(vals["seed"]))
    for i, d in enumerate(dshot):
        gpc.text_element(plugin, "sil_override", str(d), param=DSHOT_PARAM.format(i + 1), type=DSHOT_TYPE)
    if any(c != 0 for c in st["velocity_ned_m_s"]):
        gpc.text_element(plugin, "initial_velocity_ned_m_s", _fmt3(st["velocity_ned_m_s"]), "m/s")
    if any(c != 0 for c in st["body_rates_frd_rad_s"]):
        gpc.text_element(plugin, "initial_body_rates_frd", _fmt3(st["body_rates_frd_rad_s"]), "rad/s")
    if "rotor_speed_rad_s" in st:
        gpc.text_element(plugin, "initial_rotor_speed_rad_s", " ".join(repr(float(c)) for c in st["rotor_speed_rad_s"]),
                         "rad/s")
    if sensors is not None:
        si, meta = sensor_values["imu"]
        if sensors.gyro == GYRO_SOURCE_MODEL:
            gpc.text_element(plugin, "gyro_source", GYRO_SOURCE_MODEL)
            plugin.append(gic.imu_model_element(si, meta, sensors.bias_signs))
        if sensors.rotor:
            rotor = ET.SubElement(plugin, "rotor_speed_model")
            rv = sensor_values["rotor"]
            for k in ("latency_ticks", "exponent_bits", "mantissa_bits"):
                gpc.text_element(rotor, k, str(rv[k]))
            gpc.text_element(rotor, "period_unit_s", repr(float(rv["period_unit_s"])))
        if sensors.clock_corner is not None:
            gpc.text_element(plugin, "clock_corner", str(sensors.clock_corner))
            gpc.text_element(plugin, "odr_error", repr(float(si["odr_error"])))
    if attitude_source:
        gpc.text_element(plugin, "attitude_source", ATTITUDE_SOURCE_TRUTH)
    if log_path is not None:
        gpc.text_element(plugin, "log_path", log_path)
    return sdf


def generate(card_path, scenario_path, mode, m, hover_member=None, log_path=None, root=gpc.ROOT,
             attitude_source=False, sensors=None):
    """Return (file name, text). Raises gpc.GenError, scn.ScenarioError or hover.HoverError. `sensors` is a SensorSet or
    None (module docstring)."""
    if mode not in MODES:
        raise gpc.GenError([f"mode {mode!r} is not one of {MODES}"])
    if sensors is not None and check_sensors(sensors):
        raise gpc.GenError(check_sensors(sensors))
    card, profile = gpc.load_linted(card_path, root)
    if card["inertia_diag"]["value"] == schema.UNKNOWN:
        gpc.refuse(card_path, "inertia_diag", f"value is {schema.UNKNOWN}; the SDF needs it")
    cfg, units = gpc.config_from(card, profile, card_path, root)
    doc = scn.load(scenario_path, card_path)
    vals = scn.values(doc)
    if not scn.is_int(m) or m not in vals["m_sequence"]:
        raise gpc.GenError([f"{scenario_path}: m = {m!r} is not an integer in the scenario's m_sequence "
                            f"{vals['m_sequence']}"])
    name = vals["scenario"]
    cmd = vals["command"]
    if "dshot" in cmd:
        if hover_member is not None:
            raise gpc.GenError([f"{scenario_path}: --hover given, but the scenario has a dshot command"])
        dshot = cmd["dshot"]
    else:
        members = cmd["hover"]
        if hover_member is None and len(members) == 1:
            hover_member = members[0]
        if hover_member not in members:
            raise gpc.GenError([f"{scenario_path}: --hover must be one of {members}, not {hover_member!r}"])
        _, _, d_lo, d_hi = hover.bracket_for_card(cfg, vals["site_latitude_rad"], vals["site_height_m"])
        d = d_lo if hover_member == "lo" else d_hi
        dshot = [d] * scn.MOTORS
        name = f"{name}_{hover_member}"
    sensor_values = None
    if sensors is not None:
        try:
            imu = gic.imu_config(Path(root) / lint.PROFILE_DIR / f"{card['sensor_profile']}.yaml")[0::2]
        except gic.GenError as e:
            raise gpc.GenError(e.lines) from e
        sensor_values = {"imu": imu, "rotor": rotor_speed_values(profile, card_path, root) if sensors.rotor else None}
    sdf = world_element(card, cfg, units, doc, mode, m, dshot, name, log_path, attitude_source, sensors, sensor_values)
    text = '<?xml version="1.0"?>\n' + gpc.to_text(sdf)
    return f"{_world_stem(card['vehicle'], name, mode, m)}.sdf", text


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", required=True, metavar="FILE")
    ap.add_argument("--scenario", required=True, metavar="FILE")
    ap.add_argument("--mode", required=True, choices=MODES)
    ap.add_argument("--m", required=True, type=int, help="ticks per host step")
    ap.add_argument("--hover", choices=scn.HOVER_MEMBERS)
    ap.add_argument("--log-path", metavar="PATH", help="written into the plugin's <log_path>; omitted if not given")
    ap.add_argument("--attitude-source", action="store_true",
                    help="write <attitude_source>truth</attitude_source> into the plugin element; omitted if not given")
    ap.add_argument("--out-dir", required=True, metavar="DIR")
    ap.add_argument("--root", default=str(gpc.ROOT), help="repository root for resolving sensor_profile")
    args = ap.parse_args(argv)
    try:
        name, text = generate(args.card, args.scenario, args.mode, args.m, args.hover, args.log_path, args.root,
                              args.attitude_source)
    except (gpc.GenError, scn.ScenarioError) as e:
        for line in e.lines:
            print(line, file=sys.stderr)
        return 1
    except hover.HoverError as e:
        print(f"{args.card}: {e}", file=sys.stderr)
        return 1
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / name).write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
