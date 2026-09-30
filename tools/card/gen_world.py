#!/usr/bin/env python3
"""Gazebo world generator for the quad L2 lockstep plugin (docs/decisions/0003 items 4, 5, 7, 9, 10).

  gen_world.py --card <vehicle>.yaml --scenario <scenario>.yaml --mode pilot|test --m <int>
               [--hover lo|hi] [--log-path <path>] --out-dir <dir> [--root <repo root>]

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
  log_path                      text           --log-path OPTIONAL, present iff given; where the plugin writes its
                                                          binary log; no log if absent. Never written into the log.

  The initial position and attitude are the model <pose> (ENU / FLU, above); the plugin converts them back once.
  Motor numbers are logical, 1 to 4 (core section 3).
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[0] / "sim"))
import gen_plant_config as gpc  # noqa: E402
import gen_sdf  # noqa: E402
import hover  # noqa: E402
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
S = math.sqrt(0.5)


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


def world_element(card, cfg, units, doc, mode, m, dshot, name, log_path):
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
    if log_path is not None:
        gpc.text_element(plugin, "log_path", log_path)
    return sdf


def generate(card_path, scenario_path, mode, m, hover_member=None, log_path=None, root=gpc.ROOT):
    """Return (file name, text). Raises gpc.GenError, scn.ScenarioError or hover.HoverError."""
    if mode not in MODES:
        raise gpc.GenError([f"mode {mode!r} is not one of {MODES}"])
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
    sdf = world_element(card, cfg, units, doc, mode, m, dshot, name, log_path)
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
    ap.add_argument("--out-dir", required=True, metavar="DIR")
    ap.add_argument("--root", default=str(gpc.ROOT), help="repository root for resolving sensor_profile")
    args = ap.parse_args(argv)
    try:
        name, text = generate(args.card, args.scenario, args.mode, args.m, args.hover, args.log_path, args.root)
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
