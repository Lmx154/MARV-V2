"""The SIL pilot input of the lockstep plugin (ground-segment unit GX-9, the SIL pilot bridge; invariants PB-1, PB-2 and R-2 end to end).

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz-quadfc build holds libmarv_gz_lockstep.so (the gz CI step fails on
a skip). The world is the interactive SIL of tools/sim/run_quad_fc.py (FC3v2 card, model IMU, launch pad, the N-tick command delay,
<ground/>), in the pilot mode (real time) with <pilot_udp_port> in place of the stick script, and the sim channel (<sim_telemetry_udp_port>)
read by this test. The pilot is a script of this test: it sends the helper's datagrams (tools/gs/pilot.py encode_frame) to the pilot port
every PILOT_PERIOD_S of wall time, and moves from one phase to the next on the SIM time of the sim records.

What is checked (behaviour predicates on the sim channel, never the design model's envelope):
  - the flight: the vehicle stays disarmed (every DShot command 0) with the arm channel (5) low through the calibration and the link
    recovery; with ch5 high and the throttle low it arms (a DShot throttle command, >= 48); with the throttle at the derived hover stick the
    pad is released and the vehicle rises; with the sticks centred and the throttle at minimum it comes down (a touchdown, the pad held again);
    with ch5 low it disarms (every DShot command 0).
  - the control: the same script with ch5 held low never arms (every DShot command 0 for the whole run) and the pad is never released.
  - refusals: <pilot_udp_port> together with <manual_control_script>, with <hil>, and in the test mode (real_time_factor 0) is a plugin REFUSED line;
    the control is a pilot-only pilot-mode world, which is accepted.
"""

import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
for sub in ("sim", "card", "gs"):
    sys.path.insert(0, str(ROOT / "tools" / sub))
import gen_world  # noqa: E402
import pilot  # noqa: E402
import run_quad_fc  # noqa: E402
import run_scenario  # noqa: E402
import sim_channel  # noqa: E402

BUILD = ROOT / "build" / "host-gz-quadfc"
PLUGIN_DIR = BUILD / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz-quadfc build of libmarv_gz_lockstep.so is absent")

TIMEOUT_S = 600  # labelled harness value: bounds a hung gz process only
WALL_LIMIT_S = 300  # labelled harness value: wall time of one flight before the test gives up
PILOT_PERIOD_S = 0.02  # labelled harness value: the pilot script's frame period (the helper's, tools/gs/pilot.py FRAME_PERIOD_S)
RECORD_WAIT_S = 1.0  # labelled harness value: wall time waited for a sim record
SIM_RATE_HZ = 50.0  # labelled harness value: sim record rate of the test
ARM_IDLE_S = 0.5  # labelled harness value: armed at the minimum throttle before the raise
LIFT_BUDGET_S = 10.0  # labelled harness value: sim time allowed for the pad to release after the throttle is raised
LIFT_MIN_M = 0.02  # labelled harness value: height gained over the start for "lifts off the pad" (with the pad released)
AIRBORNE_S = 1.0  # labelled harness value: sim time held at the hover stick after lift-off
LAND_BUDGET_S = 15.0  # labelled harness value: sim time allowed to come down after the cut
DISARM_BUDGET_S = 1.0  # labelled harness value: sim time allowed for the disarm to reach the output
CONTROL_EXTRA_S = LIFT_BUDGET_S  # the control runs ready + arm-idle + this, with the throttle at the hover stick
DSHOT_THROTTLE_MIN = run_quad_fc.DSHOT_MIN


def ticks(x):
    """The raw channel value of x in [-1, 1] (-1 = -100 %), as the helper maps a joystick axis."""
    return max(0, min(pilot.CONSTANTS["raw_max"], round(pilot.CONSTANTS["centre"] + x * pilot.CONSTANTS["half_span_ticks"])))


def stick_ticks(s):
    """The raw throttle channel of a throttle stick s in [0, 1]: throttle = (us - 988) / 1024, us = 1500 + 512 x."""
    return ticks(2.0 * s - 1.0)


class Pilot:
    """The pilot script's frames: 16 channels (AETR: roll 1, pitch 2, throttle 3, yaw 4; arm 5; angle 8), sent every PILOT_PERIOD_S."""

    def __init__(self, port):
        self.port = port
        self.channels = [ticks(0.0)] * pilot.CHANNELS
        self.sent = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def set(self, *, arm, throttle):
        ch = [ticks(0.0)] * pilot.CHANNELS
        ch[2] = stick_ticks(throttle)
        ch[4] = ticks(1.0 if arm else -1.0)
        ch[7] = ticks(1.0)  # angle mode, as the interactive script
        self.channels = ch

    def _run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while not self._stop.is_set():
            s.sendto(pilot.encode_frame(self.channels), ("127.0.0.1", self.port))
            self.sent += 1
            self._stop.wait(PILOT_PERIOD_S)
        s.close()

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join()


def free_port():
    return pilot.free_udp_port()


def start_gz(tmp_path, pilot_port, sim_port):
    """The pilot-mode SIL world (module docstring) running in a process group of its own."""
    link = sim_channel.sim_telemetry_xml(sim_port, SIM_RATE_HZ)
    name, text = run_quad_fc.interactive_world(tmp_path, link, pilot_port=pilot_port)
    world = tmp_path / name
    world.write_text(text, encoding="utf-8")
    _, num, den = run_quad_fc.fc3_scenario(tmp_path)
    iterations = -(-run_quad_fc.us(WALL_LIMIT_S * 2) * den // num)
    cmd = [*run_scenario.GZ_ARGS, "-s", "-r", "--iterations", str(iterations), "--seed", str(run_quad_fc.SEED), str(world)]
    log = open(tmp_path / "gz.log", "wb")
    proc = subprocess.Popen(cmd, env=run_scenario.gz_env(PLUGIN_DIR), stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return proc, log


def stop_gz(proc, log):
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(30)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    log.close()


class Flight:
    """The records of a run and the clock they carry."""

    def __init__(self, sock):
        self.sock, self.records, self.deadline = sock, [], time.monotonic() + WALL_LIMIT_S

    def next(self):
        while True:
            assert time.monotonic() < self.deadline, "the flight did not finish within the wall limit"
            try:
                data = self.sock.recv(65535)
            except socket.timeout:
                continue
            r = sim_channel.decode(data)
            self.records.append(r)
            return r

    def until(self, predicate, budget_s, what):
        """Reads records until predicate(record) holds; the sim time it may take is budget_s."""
        start = self.records[-1]["sim_time_ns"] if self.records else 0
        while True:
            r = self.next()
            if predicate(r):
                return r
            assert (r["sim_time_ns"] - start) / 1e9 <= budget_s, f"{what}: not within {budget_s} s of sim time"


def run_flight(tmp_path, arm_channel_high):
    """The pilot script against a running SIL; returns (records, the pilot, gz's log text)."""
    pilot_port, sock = free_port(), socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(RECORD_WAIT_S)
    sim_port = sock.getsockname()[1]
    hover = run_quad_fc.derive_hover().stick_f32
    ready_s = run_quad_fc.build_phases(run_quad_fc.derive_hover(), run_quad_fc.STEP_STICK)[0].duration_us / 1e6
    proc, log = start_gz(tmp_path, pilot_port, sim_port)
    p = Pilot(pilot_port)
    p.set(arm=False, throttle=0.0)
    p.start()
    f = Flight(sock)
    try:
        r0 = f.next()
        start_z = r0["pos_ned_m"][2]
        f.until(lambda r: r["sim_time_ns"] >= ready_s * 1e9, ready_s + 5.0, "the calibration and the link recovery")
        if arm_channel_high:
            p.set(arm=True, throttle=0.0)
            f.until(lambda r: max(r["dshot_commanded"]) >= DSHOT_THROTTLE_MIN, 5.0, "arming on ch5 high")
            t_armed = f.records[-1]["sim_time_ns"]
            f.until(lambda r: r["sim_time_ns"] >= t_armed + ARM_IDLE_S * 1e9, ARM_IDLE_S + 1.0, "armed idle")
        else:
            p.set(arm=False, throttle=0.0)
            t = f.records[-1]["sim_time_ns"]
            f.until(lambda r: r["sim_time_ns"] >= t + ARM_IDLE_S * 1e9, ARM_IDLE_S + 1.0, "the idle with ch5 low")
        p.set(arm=arm_channel_high, throttle=hover)
        if arm_channel_high:
            lifted = f.until(lambda r: r["flags"]["pad_released"] and start_z - r["pos_ned_m"][2] >= LIFT_MIN_M, LIFT_BUDGET_S,
                             "lift-off from the pad")
            t_up = lifted["sim_time_ns"]
            f.until(lambda r: r["sim_time_ns"] >= t_up + AIRBORNE_S * 1e9, AIRBORNE_S + 1.0, "airborne hold")
            p.set(arm=True, throttle=0.0)  # the sticks centred (they are), the throttle at minimum
            f.until(lambda r: r["touchdowns"] >= 1 and r["flags"]["pad_held"], LAND_BUDGET_S, "coming down")
            p.set(arm=False, throttle=0.0)
            f.until(lambda r: max(r["dshot_commanded"]) == 0, DISARM_BUDGET_S, "disarming on ch5 low")
        else:
            t = f.records[-1]["sim_time_ns"]
            f.until(lambda r: r["sim_time_ns"] >= t + CONTROL_EXTRA_S * 1e9, CONTROL_EXTRA_S + 1.0, "the control run")
    finally:
        p.stop()
        sock.close()
        stop_gz(proc, log)
    return f.records, p, (tmp_path / "gz.log").read_text(errors="replace")


def test_a_pilot_script_arms_on_ch5_lifts_off_the_pad_comes_down_and_disarms(tmp_path):
    records, p, log = run_flight(tmp_path, True)
    assert "REFUSED" not in log, log[-2000:]
    assert p.sent > 100
    first_armed = next(i for i, r in enumerate(records) if max(r["dshot_commanded"]) >= DSHOT_THROTTLE_MIN)
    assert all(max(r["dshot_commanded"]) == 0 for r in records[:first_armed]), "disarmed (stop) before ch5 went high"
    start_z = records[0]["pos_ned_m"][2]
    assert any(r["flags"]["pad_released"] for r in records), "the pad was released"
    assert any(start_z - r["pos_ned_m"][2] >= LIFT_MIN_M for r in records), "the vehicle rose above the pad"
    assert records[-1]["touchdowns"] >= 1 and records[-1]["flags"]["pad_held"], "it came down onto the pad"
    assert max(records[-1]["dshot_commanded"]) == 0, "ch5 low disarmed it"
    print(f"pilot flight: {len(records)} records, {p.sent} frames, armed from record {first_armed}")


def test_control_with_the_arm_channel_low_the_vehicle_never_arms(tmp_path):
    records, p, log = run_flight(tmp_path, False)
    assert "REFUSED" not in log, log[-2000:]
    assert p.sent > 100 and len(records) > 100
    assert all(max(r["dshot_commanded"]) == 0 for r in records), "an unarmed vehicle commands no motor"
    assert not any(r["flags"]["pad_released"] for r in records)


# ---- refusals ---------------------------------------------------------------------------------------------------------------

PILOT = f"<pilot_udp_port>{40100}</pilot_udp_port>\n"
ITERATIONS = 8  # host steps of a refusal run


def run_world(tmp_path, text):
    sdf = tmp_path / "world.sdf"
    sdf.write_text(text, encoding="utf-8")
    return run_scenario.run_gz_process(sdf, ITERATIONS, run_quad_fc.SEED, PLUGIN_DIR, None, TIMEOUT_S)


def pilot_only_world(tmp_path, extra=""):
    return run_quad_fc.interactive_world(tmp_path, "", pilot_port=free_port())[1].replace("<gyro_source>", extra + "<gyro_source>", 1)


def test_the_plugin_refuses_the_pilot_port_with_a_script(tmp_path):
    _, text = run_quad_fc.interactive_world(tmp_path, PILOT)  # the interactive world has the stick script
    assert "<manual_control_script>" in text
    r = run_world(tmp_path, text)
    assert r.returncode != 0
    assert ("marv_gz_lockstep: REFUSED: <pilot_udp_port> and <manual_control_script> are exclusive: the pilot's frames are the only sticks"
            in r.stderr), run_scenario.stderr_tail(r.stderr)


def test_the_plugin_refuses_the_pilot_port_with_hil(tmp_path):
    hil = "<hil><device>/dev/null</device><samples_per_record>1</samples_per_record><delay_ticks>0</delay_ticks></hil>\n"
    _, text = run_quad_fc.interactive_world(tmp_path, "", pilot_port=free_port())
    text = text.replace(f"<command_delay_ticks>{run_quad_fc.COMMAND_DELAY_TICKS}</command_delay_ticks>", "")
    text = text.replace("<gyro_source>", hil + "<gyro_source>", 1)
    assert "<hil>" in text and "<pilot_udp_port>" in text
    r = run_world(tmp_path, text)
    assert r.returncode != 0
    assert "marv_gz_lockstep: REFUSED: <hil> takes the sticks from the board's receiver; <pilot_udp_port> is for the SIL" in r.stderr, \
        run_scenario.stderr_tail(r.stderr)


def test_the_plugin_refuses_the_pilot_port_in_test_mode(tmp_path):
    scenario, _, _ = run_quad_fc.fc3_scenario(tmp_path)
    _, text = gen_world.generate(run_quad_fc.CARD, scenario, "test", run_quad_fc.M, "lo", None, ROOT,
                                 sensors=gen_world.SensorSet(gyro="model", bias_signs=(0,) * 6),
                                 command_delay_ticks=run_quad_fc.COMMAND_DELAY_TICKS)
    text = text.replace("<gyro_source>", f"<launch_pad/>\n<pilot_udp_port>{free_port()}</pilot_udp_port>\n<gyro_source>", 1)
    assert "<real_time_factor>0" in text
    r = run_world(tmp_path, text)
    assert r.returncode != 0
    assert "marv_gz_lockstep: REFUSED: <pilot_udp_port> needs a real-time world" in r.stderr, run_scenario.stderr_tail(r.stderr)


def test_control_a_pilot_only_pilot_mode_world_is_accepted(tmp_path):
    r = run_world(tmp_path, pilot_only_world(tmp_path))
    assert r.returncode == 0 and "REFUSED" not in r.stderr, run_scenario.stderr_tail(r.stderr)
