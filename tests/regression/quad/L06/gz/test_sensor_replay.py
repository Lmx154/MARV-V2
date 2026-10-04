"""SIL-2 with noise on the plugin's sensor path (decision 0019, W1 C2, test b; quad spec section 4 L6 pass bar (a), T1:
"SIL-2 with noise ... The adapter's sensor bytes equal a direct marv_plant call").

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz build holds libmarv_gz_lockstep.so and the replay
tool sim/gz/sensor_replay/sensor_replay. Every gz process gets its own GZ_PARTITION and GZ_IP=127.0.0.1.

The runs are model-gyro worlds from tools/card/gen_world.py with SensorSet(gyro="model", rotor=True, signs,
clock_corner=c), at a turn-on bias corner with every axis nonzero:
- the L2 hover world (member lo): the rotors spin, so the rotor-speed samples, the plant outputs and the accel truth
  move; at (m = 1, c = 0) and (m = 2, c = +1);
- the L2 rotation world: the body turns from its initial rates, so the gyro truth moves (with decision 0016's step-0
  rates); at (m = 1, c = -1) and (m = 2, c = 0).
The tool (sim/gz/sensor_replay/sensor_replay.cpp) builds one plant from the world's plant elements and record 6, and
calls marv_plant directly, in the adapter's order, on the logged bodies and DShot.

What is checked, per run: the tool reports 0 mismatches over every tick. That covers the IMU bytes passed to the SIL, the
rotor-speed sample and the plant outputs, all bitwise. It also covers record 6's configuration against the generated
header's imu_corner_config(signs), its bounds and kImuOdrError, and record 6's tick_s against t_true. It also covers
record 6's rotor-speed configuration against the sensor profile's: the tool is given the profile's values on its plant
file's rotor_speed_model line (gen_world.rotor_speed_values over the card's profile: latency_ticks = the profile's
latency_rate_periods x the register's rate_loop_divisor, and the telemetry grid), not the world's, and checks record 6's
latency_ticks, exponent_bits, mantissa_bits and period_unit_s against them (the pole count against the world's
pole_count). The header's seed is record 6's.

Negative controls, each on a copy of every run's log, replayed by the same tool:
- one bit of one logged IMU byte flipped (the lowest bit of tick K's gyro x): exactly one mismatch, the IMU at tick K;
- record 6's seed + 1: the IMU bytes mismatch on most ticks and nothing else does (marv_plant_step ignores the seed, and
  the rotor-speed sensor draws no noise);
- record 6's rotor latency_ticks + 1, and separately its mantissa_bits + 1: exactly one config mismatch, naming that
  field (the rotor bytes and plant outputs of such a replay are not constrained);
- the samples one tick late (tick j's IMU and rotor bytes are tick j - 1's, tick 0's zeroed): the IMU bytes mismatch on
  most ticks and the rotor bytes at tick 0 at least; the plant outputs do not.
"""

import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_world  # noqa: E402
import lockstep_log  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCENARIOS = ROOT / "scenarios" / "quad" / "L02"
BUILD = ROOT / "build" / "host-gz"
PLUGIN_DIR = BUILD / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"
TOOL = BUILD / "sim" / "gz" / "sensor_replay" / "sensor_replay"
ITERATIONS = 512  # labelled harness value: host steps per run
TIMEOUT_S = 180  # labelled harness value: the wall-time bound of one gz process (the other L06/gz tests' bound)
TOOL_TIMEOUT_S = 60  # labelled harness value: the wall-time bound of one replay (it takes well under a second)
SEED = 1  # the scenarios' committed seed
# Labelled harness values: two turn-on bias corners with every axis nonzero (gyro x y z, accel x y z).
SIGNS_A = (1, -1, 1, -1, 1, -1)
SIGNS_B = (-1, 1, 1, 1, -1, -1)
# (scenario, hover member, m, clock corner, signs)
RUNS = {
    "hover_m1_c0": ("hover", "lo", 1, 0, SIGNS_A),
    "hover_m2_c+1": ("hover", "lo", 2, 1, SIGNS_B),
    "rotation_m1_c-1": ("rotation", None, 1, -1, SIGNS_B),
    "rotation_m2_c0": ("rotation", None, 2, 0, SIGNS_A),
}
# The plant elements of the plugin element the tool reads (the plugin's own names; rotor and the initial rotor speed
# below).
PLANT_SCALARS = ("esc_map", "pole_count", "mass_kg", "thrust_coeff", "torque_ratio_m", "omega_min_rad_s",
                 "omega_max_rad_s", "motor_tau_s", "site_lat_rad", "site_height_m", "motor_substep_s")
# Byte ranges inside a record's payload (after the type byte), from lockstep_log's layouts: the TICK record's IMU bytes
# after tick and sil_t_us; the ROTOR record's sample after its tick; record 6's seed after four u8, odr_error,
# host_step_ns and tick_s.
TICK_IMU = (struct.calcsize("<QQ"), struct.calcsize("<QQ32s"))
ROTOR_SAMPLE = (struct.calcsize("<Q"), lockstep_log._ROTOR.size)
SENSORS_SEED = (struct.calcsize("<BBBbdQd"), struct.calcsize("<BBBbdQdQ"))
# Record 6's rotor-speed configuration after the IMU part: four u32 (pole_count, latency_ticks, exponent_bits,
# mantissa_bits), then f64 period_unit_s.
SENSORS_ROTOR = struct.calcsize("<BBBbdQdQIQI7d7d2d6b32s")
SENSORS_LATENCY = SENSORS_ROTOR + struct.calcsize("<I")
SENSORS_MANTISSA = SENSORS_ROTOR + struct.calcsize("<III")
SENSORS_FIELD = {"latency_ticks": 35, "mantissa_bits": 37}  # their indices in lockstep_log._SENSORS's unpacked tuple
COUNT_KINDS = ("config", "imu", "rotor", "plant")

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and TOOL.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8, or the host-gz build of libmarv_gz_lockstep.so or sensor_replay, is "
                                       "absent")


def profile_rotor_grid():
    """The sensor profile's rotor-speed values, from the card's profile and the scenario register (not the world)."""
    _, profile = gen_world.gpc.load_linted(CARD, ROOT)
    return gen_world.rotor_speed_values(profile, CARD, ROOT)


def plant_input(world_text):
    """The tool's <plant> text: the plant elements of the world's lockstep plugin element, their text verbatim, and the
    profile's rotor-speed values."""
    plugins = ET.fromstring(world_text).findall(".//plugin[@filename='marv_gz_lockstep']")
    assert len(plugins) == 1
    plugin = plugins[0]

    def one(parent, name):
        found = parent.findall(name)
        assert len(found) == 1, name
        return found[0].text.strip()

    lines = [f"{name} {one(plugin, name)}" for name in PLANT_SCALARS]
    for r in plugin.findall("rotor"):
        lines.append(f"rotor {r.get('motor')} {one(r, 'position_frd_m')} {one(r, 'yaw_sign')}")
    if plugin.findall("initial_rotor_speed_rad_s"):
        lines.append(f"initial_rotor_speed_rad_s {one(plugin, 'initial_rotor_speed_rad_s')}")
    g = profile_rotor_grid()
    lines.append(f"rotor_speed_model {g['latency_ticks']} {g['exponent_bits']} {g['mantissa_bits']} "
                 f"{g['period_unit_s']!r}")
    return "\n".join(lines) + "\n"


def payloads(data, kind):
    """The payload offset of every record of type `kind`, in file order."""
    pos = lockstep_log._header(data)["header_size"]
    out = []
    while pos < len(data):
        k = data[pos]
        if k == kind:
            out.append(pos + 1)
        pos += 1 + lockstep_log._SIZES[k]
    return out


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    out = {}
    for key, (scenario, member, m, corner, signs) in RUNS.items():
        tmp = tmp_path_factory.mktemp(key)
        log = tmp / "run.bin"
        sensors = gen_world.SensorSet(gyro="model", rotor=True, bias_signs=signs, clock_corner=corner)
        text = gen_world.generate(CARD, SCENARIOS / f"{scenario}.yaml", "test", m, member, str(log),
                                  sensors=sensors)[1]
        sdf = tmp / "world.sdf"
        sdf.write_text(text, encoding="utf-8")
        r = run_scenario.run_gz_process(sdf, ITERATIONS, SEED, PLUGIN_DIR, None, TIMEOUT_S)
        assert r.returncode == 0, run_scenario.stderr_tail(r.stderr)
        out[key] = {"m": m, "corner": corner, "signs": signs, "plant": plant_input(text), "data": log.read_bytes(),
                    "log": lockstep_log.read(log), "tmp": tmp}
    return out


def replay(run, data, tag):
    """The tool's (counts, mismatches) on `data` as the run's log: counts maps ticks and each kind to its number;
    mismatches is the list of (kind, tick, what)."""
    plant = run["tmp"] / f"{tag}.plant.txt"
    plant.write_text(run["plant"], encoding="utf-8")
    log = run["tmp"] / f"{tag}.bin"
    log.write_bytes(data)
    r = subprocess.run([str(TOOL), str(plant), str(log)], capture_output=True, text=True, timeout=TOOL_TIMEOUT_S)
    assert r.returncode in (0, 1), r.stderr
    *lines, summary = r.stdout.splitlines()
    words = summary.split()
    counts = dict(zip(words[0::2], (int(w) for w in words[1::2])))
    assert list(counts) == ["ticks", *COUNT_KINDS], summary
    mismatches = [tuple(line.split()[1:4]) for line in lines]
    assert all(line.startswith("mismatch ") for line in lines)
    assert len(mismatches) == sum(counts[k] for k in COUNT_KINDS)
    assert (r.returncode == 0) == (not mismatches)
    return counts, mismatches


@pytest.mark.parametrize("key", sorted(RUNS))
def test_the_replay_equals_the_log(runs, key):
    run = runs[key]
    d = run["log"]
    s6 = d["sensors"]
    assert s6["gyro_source"] == "model" and s6["rotor_speed"] == 1 and s6["clock"] == 1
    assert s6["clock_corner"] == run["corner"] and tuple(s6["imu"]["turn_on_bias_signs"]) == run["signs"]
    assert s6["seed"] == d["header"]["seed"] == SEED
    assert len(d["ticks"]) == len(d["rotors"]) == ITERATIONS * run["m"]
    counts, mismatches = replay(run, run["data"], "equal")
    assert mismatches == []
    assert counts == {"ticks": ITERATIONS * run["m"], "config": 0, "imu": 0, "rotor": 0, "plant": 0}


@pytest.mark.parametrize("key", sorted(RUNS))
def test_control_one_flipped_imu_bit_is_one_imu_mismatch(runs, key):
    run = runs[key]
    ticks = payloads(run["data"], lockstep_log.TICK)
    k = len(ticks) // 2
    data = bytearray(run["data"])
    data[ticks[k] + TICK_IMU[0]] ^= 1
    counts, mismatches = replay(run, bytes(data), "bit")
    assert mismatches == [("imu", str(k), "bytes")]


@pytest.mark.parametrize("key", sorted(RUNS))
def test_control_a_wrong_record_6_seed_breaks_only_the_imu_bytes(runs, key):
    run = runs[key]
    (s6,) = payloads(run["data"], lockstep_log.SENSORS)
    data = bytearray(run["data"])
    a, b = s6 + SENSORS_SEED[0], s6 + SENSORS_SEED[1]
    data[a:b] = struct.pack("<Q", struct.unpack("<Q", data[a:b])[0] + 1)
    assert lockstep_log._SENSORS.unpack_from(data, s6)[7] == SEED + 1
    counts, _ = replay(run, bytes(data), "seed")
    assert counts["imu"] > counts["ticks"] // 2, counts
    assert counts["config"] == counts["rotor"] == counts["plant"] == 0, counts


@pytest.mark.parametrize("key", sorted(RUNS))
def test_control_samples_one_tick_late_break_the_sensor_bytes(runs, key):
    run = runs[key]
    data = bytearray(run["data"])
    for kind, (a, b) in ((lockstep_log.TICK, TICK_IMU), (lockstep_log.ROTOR, ROTOR_SAMPLE)):
        offsets = payloads(run["data"], kind)
        for j in reversed(range(len(offsets))):
            src = run["data"][offsets[j - 1] + a:offsets[j - 1] + b] if j > 0 else bytes(b - a)
            data[offsets[j] + a:offsets[j] + b] = src
    counts, mismatches = replay(run, bytes(data), "late")
    assert counts["imu"] > counts["ticks"] // 2, counts
    assert counts["rotor"] >= 1 and ("rotor", "0", "bytes") in mismatches, counts
    assert counts["config"] == counts["plant"] == 0, counts


@pytest.mark.parametrize("key", sorted(RUNS))
def test_the_rotor_config_is_the_profiles(runs, key):
    s6 = runs[key]["log"]["sensors"]["rotor"]
    g = profile_rotor_grid()
    assert (s6["latency_ticks"], s6["exponent_bits"], s6["mantissa_bits"], s6["period_unit_s"]) == (
        g["latency_ticks"], g["exponent_bits"], g["mantissa_bits"], g["period_unit_s"])


@pytest.mark.parametrize("field,offset", (("latency_ticks", SENSORS_LATENCY), ("mantissa_bits", SENSORS_MANTISSA)))
@pytest.mark.parametrize("key", sorted(RUNS))
def test_control_a_wrong_record_6_rotor_config_is_a_config_mismatch(runs, key, field, offset):
    run = runs[key]
    (s6,) = payloads(run["data"], lockstep_log.SENSORS)
    data = bytearray(run["data"])
    a = s6 + offset
    data[a:a + 4] = struct.pack("<I", struct.unpack("<I", data[a:a + 4])[0] + 1)
    changed = lockstep_log._SENSORS.unpack_from(data, s6)[SENSORS_FIELD[field]]
    assert changed == run["log"]["sensors"]["rotor"][field] + 1
    counts, mismatches = replay(run, bytes(data), f"rotor_{field}")
    assert [m for m in mismatches if m[0] == "config"] == [("config", "-", f"rotor_{field}")], mismatches
    assert counts["config"] == 1, counts
