"""The sim channel of the lockstep plugin (ground-segment unit GX-8, decision 0056; invariants SC-1 and S-1's producer half).

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz-quadfc build holds libmarv_gz_lockstep.so (the gz CI step fails on a
skip). The run is the HIL S2e flight script of tools/sim/run_quad_fc.py on the FC3v2 card (model IMU, launch pad, the N-tick command
delay, <ground/>) to the end of the hover hold plus one second of the roll step, not real time, twice: without the sim channel
elements, and with <sim_telemetry_udp_port> and <sim_telemetry_period_us> (tools/sim/sim_channel.py) pointing at a socket of this test.

What is checked:
  - SC-1: the lockstep log's SHA-256 is the same with and without the elements (the channel does not perturb the simulation), and the
    two worlds differ by the two elements and the log's own file name only (the control for the byte-identity of the world is in
    tests/tools/test_sim_run.py: without a port the elements are not written).
  - The records: version 1, sequence 0, 1, 2 ... with no gap, one per period of SIM time (the first at the first host step, each later
    one at the first host step at or after the next multiple of the period), and every field equals what the lockstep log holds for
    that host step: the plant body (position, velocity, attitude, rates) and the sim time of its STEP record, the TICK record's DShot
    commanded and applied, rotor speeds and IMU sample bytes, the specific force the formula -k sum(w^2) / m of the card's constants,
    the pad flags against the PAD and RELEASE records, the touchdown count and the last impact speed against the TOUCHDOWN records. A flight that lifts off (the pad released, DShot above the minimum) is part of
    what is compared, so that the flags and the DShot are not all zero.
  - Absent elements: the world has none and no datagram is sent (the socket stays empty).
  - Refusals: one element without the other, a port or a period out of range, and a repeated element are plugin REFUSED lines (abort),
    not a silent run; the same world with both elements in range is not refused (the control).
"""

import math
import re
import socket
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
for sub in ("sim", "card"):
    sys.path.insert(0, str(ROOT / "tools" / sub))
import gen_world  # noqa: E402
import run_quad_fc  # noqa: E402
import run_scenario  # noqa: E402
import sim_channel  # noqa: E402

BUILD = ROOT / "build" / "host-gz-quadfc"
PLUGIN_DIR = BUILD / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz-quadfc build of libmarv_gz_lockstep.so is absent")

TIMEOUT_S = 600  # labelled harness value: bounds a hung gz process only
AFTER_HOLD_S = 1.0  # labelled harness value: sim time after the hover hold, inside the roll step
RECEIVE_BUFFER_BYTES = 4 << 20  # labelled harness value: the test socket holds the whole run's records (not real time)
DSHOT_THROTTLE_MIN = 48  # DShot protocol: 0 is stop, 1..47 are commands, 48..2047 are throttle values


def flight_us():
    phases = run_quad_fc.build_phases(run_quad_fc.derive_hover(), run_quad_fc.STEP_STICK)
    hold = next(p for p in phases if p.name == "hold")
    return phases, hold.end_us + run_quad_fc.us(AFTER_HOLD_S)


def listener():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER_BYTES)
    s.bind(("127.0.0.1", 0))
    s.setblocking(False)
    return s


def drain(s):
    out = []
    while True:
        try:
            out.append(s.recv(65535))
        except BlockingIOError:
            return out


def run(out_dir, sim_xml):
    """One run; returns the run_scenario result."""
    phases, total_us = flight_us()
    scenario, num, den = run_quad_fc.fc3_scenario(out_dir)
    iterations = -(-total_us * den // num)
    return run_scenario.run(run_quad_fc.CARD, scenario, run_quad_fc.SEED, run_quad_fc.M, "test", "lo", out_dir, PLUGIN_DIR,
                            run_quad_fc.sdf_edit_for(phases, [], 0.0, sim_xml, ground=True), iterations=iterations, timeout_s=TIMEOUT_S,
                            sensors=gen_world.SensorSet(gyro="model", bias_signs=(0,) * 6),
                            command_delay_ticks=run_quad_fc.COMMAND_DELAY_TICKS)


@pytest.fixture(scope="module")
def pair(tmp_path_factory):
    base = tmp_path_factory.mktemp("sim_channel")
    plain = run(base / "plain", "")
    s = listener()
    try:
        port = s.getsockname()[1]
        with_channel = run(base / "channel", sim_channel.sim_telemetry_xml(port))
        records = drain(s)
    finally:
        s.close()
    return plain, with_channel, records, port


def test_sc1_the_lockstep_log_is_the_same_with_and_without_the_sim_channel(pair):
    plain, with_channel, _, port = pair
    print(f"SC-1 lockstep log sha256 without the sim channel {plain.log_sha256}, with it {with_channel.log_sha256}")
    assert plain.log_sha256 == with_channel.log_sha256
    a = Path(plain.log_path.replace(".bin", ".sdf")).read_text(encoding="utf-8").replace(str(Path(plain.log_path).parent), "X")
    b = Path(with_channel.log_path.replace(".bin", ".sdf")).read_text(encoding="utf-8").replace(str(Path(with_channel.log_path).parent), "X")
    added = (f"<sim_telemetry_udp_port>{port}</sim_telemetry_udp_port>\n"
             f"<sim_telemetry_period_us>{sim_channel.period_us(sim_channel.DEFAULT_RATE_HZ)}</sim_telemetry_period_us>\n")
    assert b.replace(added, "", 1) == a


def test_without_the_elements_nothing_is_sent(tmp_path):
    s = listener()
    try:
        run(tmp_path, "")
        assert drain(s) == []  # the control: the socket was never told about this run, and the world has no element
    finally:
        s.close()


def test_records_are_sequential_periodic_and_equal_to_the_lockstep_log(pair):
    plain, with_channel, datagrams, _ = pair
    log = with_channel.log
    period_ns = sim_channel.period_us(sim_channel.DEFAULT_RATE_HZ) * 1000
    records = [sim_channel.decode(d) for d in datagrams]
    assert len(records) > 100
    assert [r["seq"] for r in records] == list(range(len(records)))  # no gap, none out of order
    step_ns = log["steps"][1]["sim_time_ns"] - log["steps"][0]["sim_time_ns"]
    assert len(records) == math.ceil(log["steps"][-1]["sim_time_ns"] / period_ns)  # one per period, the first at the first step
    world = Path(with_channel.log_path.replace(".bin", ".sdf")).read_text(encoding="utf-8")
    k = float(re.search(r"<thrust_coeff[^>]*>([^<]+)<", world).group(1))
    mass = float(re.search(r"<mass_kg[^>]*>([^<]+)<", world).group(1))
    released = log["pad"]["release_tick"]
    assert released is not None, "the flight must lift off for the pad flags to be compared"
    lifted = False
    for n, r in enumerate(records):
        due = n * period_ns
        i = r["tick"]  # m = 1: the host step i holds tick i
        step, tick = log["steps"][i], log["ticks"][i]
        assert due <= r["sim_time_ns"] < due + period_ns + step_ns  # the first host step at or after the multiple of the period
        assert r["sim_time_ns"] == step["sim_time_ns"] and r["tick"] == tick["tick"]
        assert tuple(r["pos_ned_m"]) == tuple(step["body_pos_ned"]) and tuple(r["vel_ned_m_s"]) == tuple(step["body_vel_ned"])
        assert tuple(r["q_wxyz"]) == tuple(step["body_q_wxyz"]) and tuple(r["omega_frd_rad_s"]) == tuple(step["body_omega_frd"])
        assert tuple(r["rotor_speed_rad_s"]) == tuple(tick["rotor_speed"])
        assert tuple(r["dshot_commanded"]) == tuple(tick["dshot"]) and tuple(r["dshot_applied"]) == tuple(tick["dshot_applied"])
        gyro_accel = struct.unpack("<6f", tick["imu"][:24])
        assert tuple(r["imu_gyro_rad_s"]) + tuple(r["imu_accel_m_s2"]) == gyro_accel
        assert r["imu_flags"] == struct.unpack("<I", tick["imu"][28:32])[0]
        thrust = k * sum(w * w for w in tick["rotor_speed"])
        assert r["specific_force_frd_m_s2"][:2] == (0.0, 0.0)
        assert r["specific_force_frd_m_s2"][2] == pytest.approx(-thrust / mass, rel=1e-12, abs=0.0)
        f = r["flags"]
        assert f["imu_model"] and f["launch_pad"] and f["ground"] and not f["hil"]
        assert f["pad_held"] == tick["pad_held"] and f["pad_released"] == (released <= i)
        done = [d for d in log["touchdowns"] if d["tick"] <= i]  # the plugin counts a touchdown in the host step that detects it
        assert r["touchdowns"] == len(done)
        assert r["last_touchdown_down_m_s"] == (struct.unpack("<f", struct.pack("<f", done[-1]["down_speed_m_s"]))[0] if done else 0.0)
        lifted = lifted or (f["pad_released"] and max(r["dshot_commanded"]) > DSHOT_THROTTLE_MIN)
        assert r["hil"]["status_records"] == 0 and r["hil"]["sticks"] == (0.0, 0.0, 0.0, 0.0)  # a SIL run: the HIL block is zero
        assert r["rtf"] >= 0.0
    assert lifted
    assert records[0]["rtf"] == 0.0 and all(r["rtf"] > 0.0 for r in records[1:])


PORT_ELEMENT, PERIOD_ELEMENT = "<sim_telemetry_udp_port>40000</sim_telemetry_udp_port>\n", "<sim_telemetry_period_us>50000</sim_telemetry_period_us>\n"
REFUSALS = [
    ("port_only", PORT_ELEMENT, "<sim_telemetry_udp_port> and <sim_telemetry_period_us> go together; one of them is missing"),
    ("period_only", PERIOD_ELEMENT, "<sim_telemetry_udp_port> and <sim_telemetry_period_us> go together; one of them is missing"),
    ("port_zero", "<sim_telemetry_udp_port>0</sim_telemetry_udp_port>\n" + PERIOD_ELEMENT, "<sim_telemetry_udp_port> is not a UDP port 1..65535"),
    ("port_too_large", "<sim_telemetry_udp_port>65536</sim_telemetry_udp_port>\n" + PERIOD_ELEMENT,
     "<sim_telemetry_udp_port> is not a UDP port 1..65535"),
    ("period_zero", PORT_ELEMENT + "<sim_telemetry_period_us>0</sim_telemetry_period_us>\n", "<sim_telemetry_period_us> is 0"),
    ("port_twice", PORT_ELEMENT + PORT_ELEMENT + PERIOD_ELEMENT, "plugin element <sim_telemetry_udp_port> appears twice"),
]


@pytest.mark.parametrize("name,elements,reason", REFUSALS, ids=[r[0] for r in REFUSALS])
def test_the_plugin_refuses_an_incomplete_or_out_of_range_sim_channel(tmp_path, name, elements, reason):
    _, text = run_quad_fc.interactive_world(tmp_path, elements)
    sdf = tmp_path / "world.sdf"
    sdf.write_text(text, encoding="utf-8")
    r = run_scenario.run_gz_process(sdf, 8, run_quad_fc.SEED, PLUGIN_DIR, None, TIMEOUT_S)
    assert r.returncode != 0, "the run was not refused"
    assert f"marv_gz_lockstep: REFUSED: {reason}" in r.stderr, run_scenario.stderr_tail(r.stderr)


def test_control_both_elements_in_range_are_not_refused(tmp_path):
    _, good = run_quad_fc.interactive_world(tmp_path, PORT_ELEMENT + PERIOD_ELEMENT)
    sdf = tmp_path / "world.sdf"
    sdf.write_text(good, encoding="utf-8")
    ok = run_scenario.run_gz_process(sdf, 8, run_quad_fc.SEED, PLUGIN_DIR, None, TIMEOUT_S)
    assert ok.returncode == 0 and "REFUSED" not in ok.stderr, run_scenario.stderr_tail(ok.stderr)
