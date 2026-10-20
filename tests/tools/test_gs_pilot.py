"""The SIL pilot bridge of the helper (tools/gs/pilot.py, ground-segment unit GX-9; invariants PB-1 and PB-2).

A fake joystick device is a FIFO named js0 in a directory of the test, written with struct js_event records as the kernel's joydev
does (including the INIT burst at open). The plugin's port is a UDP socket of the test.

  - the frame: axis to 11-bit mapping at the ends and the centre, the datagram's round trip, the INIT bit of an event;
  - PB-2: frames arrive while the joystick is open, and none after close() (the control is the frames before it); a joystick that
    goes away (end of file) is state "lost" and the frames stop too;
  - the pilot_state events are held to 30 Hz while a change of state is always told;
  - the device path is refused unless it is js<N> of the joystick directory;
  - through the helper: pilot_open is refused without a run started with the pilot option (the control: it is accepted with one), the
    run's plan carries the pilot port only with the option, and stopping the run stops the frames;
  - PB-1: nothing the bridge sends goes to the link's socket (the SIL's UDP link port).
"""

import asyncio
import json
import os
import socket
import struct
import sys
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "tools" / "gs"))
for sub in ("sim", "card"):
    sys.path.insert(0, str(ROOT / "tools" / sub))
import pilot as pilot_mod  # noqa: E402
import run_quad_fc  # noqa: E402
import test_gs_helper as gh  # noqa: E402
import runs as runs_mod  # noqa: E402

PERIOD_S = 0.02  # the bridge's frame period under test (pilot.FRAME_PERIOD_S)
SETTLE_PERIODS = 5  # labelled harness value: periods waited for frames to arrive or to be absent
AXIS_ARM = 4  # channel 5 under the default layout (channel n = axis n - 1)


def js_event(kind, number, value, init=False):
    return struct.pack("<IhBB", 0, value, kind | (pilot_mod.JS_EVENT_INIT if init else 0), number)


def init_burst(values):
    return b"".join(js_event(pilot_mod.JS_EVENT_AXIS, n, v, init=True) for n, v in enumerate(values))


def fake_js(tmp_path):
    """(the directory of joysticks, the FIFO path, a writer end that keeps the FIFO from reporting end of file)."""
    d = tmp_path / "input"
    d.mkdir()
    path = d / "js0"
    os.mkfifo(path)
    keep = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    return d, path, keep


def listener():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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


def test_axis_mapping_datagram_and_events():
    c = pilot_mod.CONSTANTS
    assert c == {"centre": 992, "half_span_ticks": pytest.approx(819.2), "raw_max": 2047}
    assert pilot_mod.axis_to_ticks(0) == c["centre"]
    assert pilot_mod.axis_to_ticks(32767) == 1811 and pilot_mod.axis_to_ticks(-32767) == 173  # CRSF +100 % tick; -100 % is 172 (ELRS) / 173 here
    assert pilot_mod.axis_to_ticks(-32768) == pilot_mod.axis_to_ticks(-32767)  # clamped
    ch = list(range(0, 2048, 128))[:16]
    assert pilot_mod.decode_frame(pilot_mod.encode_frame(ch)) == ch
    for bad in (ch[:15], [*ch[:15], 2048], [*ch[:15], -1]):
        with pytest.raises(pilot_mod.PilotError):
            pilot_mod.encode_frame(bad)
    with pytest.raises(pilot_mod.PilotError):
        pilot_mod.decode_frame(b"XXXX" + pilot_mod.encode_frame(ch)[4:])
    assert pilot_mod.parse_events(init_burst([5, -5]) + js_event(pilot_mod.JS_EVENT_BUTTON, 3, 1) + b"\x00") == [
        (pilot_mod.JS_EVENT_AXIS, 0, 5), (pilot_mod.JS_EVENT_AXIS, 1, -5), (pilot_mod.JS_EVENT_BUTTON, 3, 1)]


def test_frames_arrive_while_open_and_stop_at_once_on_close(tmp_path):
    d, path, keep = fake_js(tmp_path)
    s = listener()
    events = []

    async def main():
        p = pilot_mod.Pilot(events.append, d, frame_period_s=PERIOD_S)
        await p.open(str(path), s.getsockname()[1])
        os.write(keep, init_burst([0, 0, 0, 0, 32767, 0, 0, 0]))
        await asyncio.sleep(PERIOD_S * SETTLE_PERIODS)
        before = [pilot_mod.decode_frame(x) for x in drain(s)]
        assert len(before) >= 3, "control: frames are sent while the joystick is open"
        assert before[-1][AXIS_ARM] == 1811 and before[-1][0] == 992 and before[-1][8:] == [992] * 8
        p.close()
        assert p.state == "closed"
        drain(s)  # whatever was in flight when close() returned
        await asyncio.sleep(PERIOD_S * SETTLE_PERIODS)
        assert drain(s) == [], "PB-2: no frame after close"
        p.close()  # idempotent
        assert [e["state"] for e in events if e["state"] != "open"] == ["closed"]

    try:
        asyncio.run(main())
    finally:
        os.close(keep)
        s.close()


def test_a_joystick_that_goes_away_is_lost_and_the_frames_stop(tmp_path):
    d, path, keep = fake_js(tmp_path)
    s = listener()

    async def main():
        events = []
        p = pilot_mod.Pilot(events.append, d, frame_period_s=PERIOD_S)
        await p.open(str(path), s.getsockname()[1])
        await asyncio.sleep(PERIOD_S * SETTLE_PERIODS)
        assert drain(s)
        os.close(keep)  # the last writer closes: the reader sees end of file
        await asyncio.sleep(PERIOD_S * SETTLE_PERIODS)
        assert p.state == "lost" and p.reason == "end of file"
        drain(s)
        await asyncio.sleep(PERIOD_S * SETTLE_PERIODS)
        assert drain(s) == []
        assert events[-1]["state"] == "lost"

    try:
        asyncio.run(main())
    finally:
        s.close()


def test_state_events_are_held_to_30_hz_and_a_change_of_state_is_always_told(tmp_path):
    d, path, keep = fake_js(tmp_path)
    s = listener()
    window_s = 1.0  # labelled harness value: the counting window

    async def main():
        events = []
        p = pilot_mod.Pilot(events.append, d, frame_period_s=PERIOD_S / 4)  # frames faster than the cap, to exercise it
        await p.open(str(path), s.getsockname()[1])
        await asyncio.sleep(window_s)
        periodic = [e for e in events if e["state"] == "open"]
        assert 5 < len(periodic) <= pilot_mod.STATE_HZ_MAX * window_s + 1  # the first (state change) plus at most the cap
        assert p.frames_sent > len(periodic) * 2, "frames were not what limited the events"
        p.close()
        assert events[-1]["state"] == "closed" and events[-1]["channels"] is None

    try:
        asyncio.run(main())
    finally:
        os.close(keep)
        s.close()


def test_a_device_is_a_js_file_of_the_joystick_directory(tmp_path):
    d, path, keep = fake_js(tmp_path)
    (tmp_path / "elsewhere").write_text("")
    os.symlink(tmp_path / "elsewhere", d / "js1")
    p = pilot_mod.Pilot(lambda e: None, d)
    assert p.resolve(str(path)) == str(path) and p.devices() == [str(path), str(d / "js1")]
    for bad in (str(d / "js"), str(d / "event0"), str(d / ".." / "input" / "js0"), str(tmp_path / "elsewhere"), str(d / "js1"), "js0", "", None, 5,
                str(d / "js0\0")):
        with pytest.raises(pilot_mod.PilotError):
            p.resolve(bad)
    with pytest.raises(pilot_mod.PilotError):
        pilot_mod.check_axis_map([0] * 15)
    assert pilot_mod.check_axis_map(None) == list(range(16))
    os.close(keep)


def test_axis_map_replaces_the_default_layout(tmp_path):
    d, path, keep = fake_js(tmp_path)
    s = listener()

    async def main():
        p = pilot_mod.Pilot(lambda e: None, d, frame_period_s=PERIOD_S)
        await p.open(str(path), s.getsockname()[1], [None, 7, *range(14)])
        os.write(keep, init_burst([0, 0, 0, 0, 0, 0, 0, 32767]))
        await asyncio.sleep(PERIOD_S * SETTLE_PERIODS)
        ch = pilot_mod.decode_frame(drain(s)[-1])
        assert ch[0] == 992 and ch[1] == 1811 and ch[2] == 992  # channel 2 reads axis 7; channel 1 has none
        p.close()

    try:
        asyncio.run(main())
    finally:
        os.close(keep)
        s.close()


def helper_run(tmp_path, body, **kw):
    d, path, keep = fake_js(tmp_path)
    try:
        return gh.run_with_helper(tmp_path, lambda h: body(h, path, keep), sil_command=["sleep", "600"], pilot_dir=d, **kw)
    finally:
        os.close(keep)


def test_pilot_open_needs_a_run_with_the_pilot_option_and_stopping_the_run_stops_the_frames(tmp_path):
    link = listener()  # PB-1: the SIL link's port (the helper's --sil-udp-port): nothing of the pilot is ever sent to it
    sink = listener()

    async def body(h, path, keep):
        h.pilot_port = sink.getsockname()[1]
        h.pilot.frame_period_s = PERIOD_S
        async with gh.ws_connect(h) as ws:
            s = gh.Session(ws)
            plan = (await s.cmd(cmd="run_plan", kind="sil"))["plan"]
            assert plan["pilot_port"] is None and "--pilot-udp-port" not in plan["argv"]
            plan = (await s.cmd(cmd="run_plan", kind="sil", pilot=True))["plan"]
            # the stand-in command replaces the argv; the plan still names the port the real command would be given
            assert plan["pilot_port"] == h.pilot_port
            assert (await s.cmd(cmd="pilot_devices"))["devices"] == [str(path)]
            # no run
            refused = await s.cmd(cmd="pilot_open", device=str(path))
            assert not refused["ok"] and "pilot option" in refused["error"]
            # a run without the option
            assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]
            refused = await s.cmd(cmd="pilot_open", device=str(path))
            assert not refused["ok"] and "pilot option" in refused["error"]
            assert (await s.cmd(cmd="run_stop"))["ok"]
            # the control: a run with the option
            started = await s.cmd(cmd="run_start", kind="sil", pilot=True)
            assert started["ok"] and started["run"]["pilot_port"] == h.pilot_port and started["run"]["options"]["pilot"] is True
            bad = await s.cmd(cmd="pilot_open", device="/dev/input/js0")
            assert not bad["ok"] and "not a joystick device" in bad["error"]
            opened = await s.cmd(cmd="pilot_open", device=str(path))
            assert opened["ok"] and opened["pilot"]["state"] == "open"
            again = await s.cmd(cmd="pilot_open", device=str(path))
            assert not again["ok"] and "close it first" in again["error"]
            os.write(keep, init_burst([0, 0, 0, 0, 32767]))
            ev = await s.wait_event("pilot_state", state="open", timeout=gh.TIMEOUT_S)
            assert ev["port"] == h.pilot_port and ev["device"] == str(path)
            await asyncio.sleep(PERIOD_S * SETTLE_PERIODS)
            frames = [pilot_mod.decode_frame(x) for x in drain(sink)]
            assert frames and frames[-1][4] == 1811
            assert drain(link) == [], "PB-1: not on the link's port"
            status = (await s.cmd(cmd="status"))["status"]["pilot"]
            assert status["state"] == "open" and status["frames_sent"] > 0
            # stopping the run closes the pilot
            assert (await s.cmd(cmd="run_stop"))["ok"]
            assert h.pilot.state == "closed"
            await s.wait_event("pilot_state", state="closed", timeout=gh.TIMEOUT_S)
            drain(sink)
            await asyncio.sleep(PERIOD_S * SETTLE_PERIODS)
            assert drain(sink) == []
            closed = await s.cmd(cmd="pilot_close")
            assert closed["ok"] and closed["pilot"]["state"] == "closed"
            s.close()

    helper_run(tmp_path, body, sil_udp_port=link.getsockname()[1])
    link.close()
    sink.close()


def test_pilot_close_command_stops_the_frames(tmp_path):
    sink = listener()

    async def body(h, path, keep):
        h.pilot_port = sink.getsockname()[1]
        h.pilot.frame_period_s = PERIOD_S
        async with gh.ws_connect(h) as ws:
            s = gh.Session(ws)
            assert (await s.cmd(cmd="run_start", kind="sil", pilot=True))["ok"]
            assert (await s.cmd(cmd="pilot_open", device=str(path)))["ok"]
            await asyncio.sleep(PERIOD_S * SETTLE_PERIODS)
            assert drain(sink), "control: frames while open"
            t0 = time.monotonic()
            assert (await s.cmd(cmd="pilot_close"))["ok"]
            drain(sink)
            await asyncio.sleep(PERIOD_S * SETTLE_PERIODS)
            assert drain(sink) == [], f"PB-2: frames after pilot_close ({time.monotonic() - t0:.2f} s)"
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()

    helper_run(tmp_path, body)
    sink.close()


def test_the_pilot_world_has_the_port_and_no_stick_script_and_the_default_world_has_neither_pilot_element(tmp_path):
    _, plain = run_quad_fc.interactive_world(tmp_path, "")
    _, piloted = run_quad_fc.interactive_world(tmp_path, "", pilot_port=40123)
    assert "<pilot_udp_port>" not in plain and "<manual_control_script>" in plain
    assert "<pilot_udp_port>40123</pilot_udp_port>" in piloted and "<manual_control_script>" not in piloted
    swapped = piloted.replace("<pilot_udp_port>40123</pilot_udp_port>", run_quad_fc.script_xml(
        [run_quad_fc.Phase("idle", 0, run_quad_fc.us(run_quad_fc.INTERACTIVE_S), run_quad_fc.SWITCH_LOW)]), 1)
    assert swapped == plain, "the two worlds differ by the stick source only"
    assert runs_mod.sil_args({"pilot": True}, None, 40123) == ["--pilot-udp-port", "40123"]
    assert runs_mod.sil_args({"pilot": False}, None, 40123) == [] and runs_mod.sil_args({}, None, 40123) == []
