"""The console relay (ground-segment unit GX-16, tools/gs/console.py, decision 0038): /ws/console has the helper's gate and one client; text
goes both ways in order against a pty standing for the board's console; the console log; close stops traffic; the console is refused
while a HIL run, the HIL hold or a flash owns the board (each with a control: the same request passes when it does not); the device
path is restricted to the by-id directory; one opener per port; and H-2 (nothing outlives the close). A pty and fakes only: no real
port is opened.
"""

import asyncio
import json
import os
import sys
import time
from pathlib import Path

import pytest
from websockets.exceptions import ConnectionClosed, InvalidStatus

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "gs"))

import console as console_mod  # noqa: E402
import device  # noqa: E402
import groundlog  # noqa: E402
import helper as helper_mod  # noqa: E402
from test_gs_helper import (  # noqa: E402  (module-level helpers of the sibling test file)
    POLL_S, TIMEOUT_S, Session, _read_all, _write_all, control, fake_board, http_get, run_with_helper, sim_connect, wait_for, ws_connect)

NAME = "usb-Raspberry_Pi_Pico_TESTCONSOLE-if00"
QUIET_S = 0.4  # labelled harness value: how long "nothing arrives" is watched for (many relay round trips on a loopback pty)
BANNER = "marv_fc3v2 console ready\r\n"
LINES = ["motor_test 1 100 500 PROPS-OFF\n", "stop\n", "x\r\n", "café µs\n"]


class Pty:
    """A pty standing for the board's console: the helper opens the slave through a by-id link, the test holds the master."""

    def __init__(self, tmp_path, name=NAME):
        self.master, self.slave = os.openpty()
        os.set_blocking(self.master, False)
        by_id = tmp_path / "by-id"
        by_id.mkdir(exist_ok=True)
        self.link = str(by_id / name)
        (by_id / name).symlink_to(os.ttyname(self.slave))
        self.got = bytearray()

    def pump(self):
        self.got.extend(_read_all(self.master))
        return bytes(self.got)

    async def read_until(self, n):
        deadline = time.monotonic() + TIMEOUT_S
        while len(self.pump()) < n and time.monotonic() < deadline:
            await asyncio.sleep(0.01)
        return bytes(self.got)

    async def write(self, data):
        await _write_all(self.master, data)

    def close(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


def run_console(tmp_path, body, *, pty=True, **kwargs):
    board = Pty(tmp_path) if pty else None
    kwargs.setdefault("autoconnect", False)
    try:
        return run_with_helper(tmp_path, lambda h: body(h, board), **kwargs)
    finally:
        if board:
            board.close()


async def recv_text(ws, n):
    """Text frames until n characters have arrived."""
    got = ""
    while len(got) < n:
        m = await asyncio.wait_for(ws.recv(), TIMEOUT_S)
        assert isinstance(m, str), "the console carries text frames only"
        got += m
    return got


async def silence(ws):
    """True when no frame arrives within QUIET_S."""
    try:
        await asyncio.wait_for(ws.recv(), QUIET_S)
    except asyncio.TimeoutError:
        return True
    return False


async def opened(s, pty):
    reply = await s.cmd(cmd="console_open", device=pty.link)
    assert reply["ok"], reply
    return reply


# ---- the gate ---------------------------------------------------------------------------------------------------------------------

def test_ws_console_has_the_gate_of_ws_and_accepts_one_client(tmp_path):
    async def body(h, board):
        for token in ("", "wrong"):
            with pytest.raises(InvalidStatus) as e:
                async with sim_connect(h, token=token, path="/ws/console"):
                    pass
            assert e.value.response.status_code == 401
        for origin in ("http://evil.example", None):
            with pytest.raises(InvalidStatus) as e:
                async with sim_connect(h, origin=origin, path="/ws/console"):
                    pass
            assert e.value.response.status_code == 403
        with pytest.raises(InvalidStatus) as e:
            async with sim_connect(h, path="/ws/consol"):
                pass
        assert e.value.response.status_code == 404
        assert (await http_get(h, "/ws/console", token="own"))[0] == 426  # a plain GET: WebSocket only
        assert (await http_get(h, "/ws/console", token=None))[0] == 401
        assert (await http_get(h, "/ws/console", host="evil.example"))[0] == 403
        async with sim_connect(h, path="/ws/console") as first:  # control: the same request with the right token and origin is served
            async with ws_connect(h) as link:  # the link relay at /ws is a client of its own
                s = Session(link)
                assert (await s.cmd(cmd="console_status"))["console"]["client"] is True
                await opened(s, board)
                async with sim_connect(h, path="/ws/console") as second:
                    with pytest.raises(ConnectionClosed) as e:
                        await asyncio.wait_for(second.recv(), TIMEOUT_S)
                    assert e.value.rcvd.code == helper_mod.WS_BUSY
                await board.write(b"still here\n")  # the first client is untouched
                assert await recv_text(first, 11) == "still here\n"
                s.close()
        assert await asyncio.to_thread(wait_for, lambda: h.console.client is None)
    run_console(tmp_path, body)


def test_a_binary_frame_closes_the_console_client(tmp_path):
    async def body(h, board):
        async with sim_connect(h, path="/ws/console") as ws:
            await ws.send(b"\x00\x01")
            with pytest.raises(ConnectionClosed) as e:
                await asyncio.wait_for(ws.recv(), TIMEOUT_S)
            assert e.value.rcvd.code == console_mod.WS_UNSUPPORTED
        assert await asyncio.to_thread(wait_for, lambda: h.console.client is None)
    run_console(tmp_path, body)


# ---- the relay --------------------------------------------------------------------------------------------------------------------

def test_text_goes_both_ways_in_order_and_unchanged_against_a_pty(tmp_path):
    async def body(h, board):
        async with ws_connect(h) as link, sim_connect(h, path="/ws/console") as ws:
            s = Session(link)
            reply = await opened(s, board)
            assert reply["console"]["state"] == "open" and reply["console"]["device"] == board.link
            await s.wait_event("console_state", state="open")
            # down: chunks in order, a multi-byte character split across two writes arrives whole
            raw = (BANNER + "café µs\r\n").encode("utf-8")
            cut = raw.index("é".encode("utf-8")) + 1
            await board.write(raw[:cut])
            await asyncio.sleep(0.05)
            await board.write(raw[cut:])
            await board.write(b"bad \xff byte\n")
            expected = raw.decode("utf-8") + "bad � byte\n"
            assert await recv_text(ws, len(expected)) == expected
            # up: the lines reach the board as their UTF-8 bytes, in order, with nothing added or taken away
            sent = b""
            for line in LINES:
                await ws.send(line)
                sent += line.encode("utf-8")
            assert await board.read_until(len(sent)) == sent
            # and the board answers an echo of what it got
            await board.write(sent)
            assert await recv_text(ws, len(sent.decode("utf-8"))) == sent.decode("utf-8")
            status = (await s.cmd(cmd="console_status"))["console"]
            assert status["rx_bytes"] == len(raw) + len(b"bad \xff byte\n") + len(sent) and status["tx_bytes"] == len(sent)
            s.close()
    run_console(tmp_path, body)


def test_the_link_relay_and_the_ground_log_do_not_see_the_console(tmp_path):
    async def body(h, board):
        async with ws_connect(h) as link, sim_connect(h, path="/ws/console") as ws:
            assert (await control(link, cmd="console_open", device=board.link))["ok"]
            await board.write(b"console only\n")
            await recv_text(ws, 13)
            await ws.send("typed\n")
            await board.read_until(6)
            frames = []
            while True:  # whatever else /ws carries in this time is an event, never a binary frame of console bytes
                try:
                    frames.append(await asyncio.wait_for(link.recv(), QUIET_S))
                except asyncio.TimeoutError:
                    break
            assert all(isinstance(f, str) and "event" in json.loads(f) for f in frames), frames
    run_console(tmp_path, body)
    log = next((tmp_path / "runs").glob("*/ground.log"))
    assert b"".join(groundlog.byte_stream(log, groundlog.RX)) == b"" and b"".join(groundlog.byte_stream(log, groundlog.TX)) == b""


# ---- the log ----------------------------------------------------------------------------------------------------------------------

def test_console_log_is_written_only_while_a_console_is_open_in_ground_log_format(tmp_path):
    seen = {}

    async def body(h, board):
        runs = lambda: sorted(p.name for p in h.run_dir.glob("console*.log"))  # noqa: E731
        async with ws_connect(h) as link, sim_connect(h, path="/ws/console") as ws:
            s = Session(link)
            assert runs() == []  # nothing before the first open
            await board.write(b"before open\n")
            await asyncio.sleep(0.1)
            await opened(s, board)
            assert runs() == ["console.log"] and (await s.cmd(cmd="console_status"))["console"]["log"] == str(h.run_dir / "console.log")
            await board.write(b"hello\n")
            await recv_text(ws, 6)
            await ws.send("stop\n")
            await board.read_until(5)
            assert (await s.cmd(cmd="console_close"))["ok"]
            seen["size"] = (h.run_dir / "console.log").stat().st_size
            await board.write(b"after close\n")
            await ws.send("ignored\n")
            await asyncio.sleep(QUIET_S)
            assert (h.run_dir / "console.log").stat().st_size == seen["size"]  # nothing written while closed
            assert (await s.cmd(cmd="console_status"))["console"]["log"] is None
            await opened(s, board)  # a second open writes a file of its own
            assert runs() == ["console-1.log", "console.log"]
            assert (await s.cmd(cmd="console_close"))["ok"]
            s.close()
    run_console(tmp_path, body)
    run_dir = next((tmp_path / "runs").iterdir())
    first = run_dir / "console.log"
    assert b"".join(groundlog.byte_stream(first, groundlog.RX)) == b"hello\n"
    assert b"".join(groundlog.byte_stream(first, groundlog.TX)) == b"stop\n"
    events = [r.data.decode() for r in groundlog.read_records(first) if r.kind == groundlog.EVENT]
    assert len(events) == 2 and events[0].startswith("console opened: ") and events[1] == "console closed: closed by request"
    second = [r for r in groundlog.read_records(run_dir / "console-1.log")]
    assert [r.kind for r in second] == [groundlog.EVENT, groundlog.EVENT]
    assert b"before open" not in first.read_bytes() and b"after close" not in first.read_bytes() and b"ignored" not in first.read_bytes()
    assert b"hello" not in (run_dir / "ground.log").read_bytes()  # the ground log is the link's


def test_helper_stop_closes_an_open_console_and_its_log(tmp_path):
    async def body(h, board):
        async with ws_connect(h) as link:
            s = Session(link)
            await opened(s, board)
            s.close()
    run_console(tmp_path, body)
    log = next((tmp_path / "runs").glob("*/console.log"))
    events = [r.data.decode() for r in groundlog.read_records(log) if r.kind == groundlog.EVENT]
    assert events[-1] == "console closed: helper stopping"


# ---- close stops traffic ----------------------------------------------------------------------------------------------------------

def test_close_stops_traffic_in_both_directions_and_open_starts_it_again(tmp_path):
    async def body(h, board):
        async with ws_connect(h) as link, sim_connect(h, path="/ws/console") as ws:
            s = Session(link)
            await opened(s, board)
            await board.write(b"one\n")  # control: traffic flows while open
            assert await recv_text(ws, 4) == "one\n"
            await ws.send("two\n")
            assert await board.read_until(4) == b"two\n"
            closed = await s.cmd(cmd="console_close")
            assert closed["ok"] and closed["console"]["state"] == "closed" and closed["console"]["device"] is None
            await s.wait_event("console_state", state="closed", reason="closed by request")
            await board.write(b"three\n")
            await ws.send("four\n")
            assert await silence(ws)
            assert board.pump() == b"two\n"
            assert (await s.cmd(cmd="console_close"))["ok"]  # idempotent
            await opened(s, board)  # control: traffic flows again
            await board.write(b"five\n")
            assert await recv_text(ws, 5) == "five\n"
            s.close()
    run_console(tmp_path, body)


def test_a_console_the_board_drops_is_lost_and_can_be_opened_again(tmp_path):
    async def body(h, board):
        async with ws_connect(h) as link:
            s = Session(link)
            await opened(s, board)
            os.close(board.master)  # the board went away (reset, USB unplugged)
            board.master = -1
            lost = await s.wait_event("console_state", state="lost")
            assert lost["reason"] and h.console.device is None
            assert (await s.cmd(cmd="console_status"))["console"]["state"] == "lost"
            s.close()
    run_console(tmp_path, body)


# ---- refusal while a HIL run, the hold or a flash owns the board --------------------------------------------------------------------

def test_console_is_refused_while_a_hil_run_is_going_and_opens_when_it_is_not(tmp_path):
    port, keep = fake_board(tmp_path, "usb-Fake_Board_1-if00")

    async def body(h, board):
        async with ws_connect(h) as link:
            s = Session(link)
            assert (await s.cmd(cmd="run_start", kind="hil", port=port))["ok"]
            refused = await s.cmd(cmd="console_open", device=board.link)
            assert not refused["ok"] and "HIL run" in refused["error"] and refused["refusals"]
            assert h.console.device is None and (await s.cmd(cmd="console_status"))["console"]["blocked"]
            assert (await s.cmd(cmd="run_stop"))["ok"]
            assert (await s.cmd(cmd="console_open", device=board.link))["ok"]  # control: the same request, no run
            s.close()
    try:
        run_console(tmp_path, body, hil_command=["sleep", "600"])
    finally:
        for fd in keep:
            os.close(fd)


def test_console_is_refused_while_the_hil_hold_is_taken_and_opens_when_it_is_released(tmp_path):
    async def body(h, board):
        async with ws_connect(h) as link:
            s = Session(link)
            assert (await s.cmd(cmd="hil_hold", on=True))["hold"] is True
            refused = await s.cmd(cmd="console_open", device=board.link)
            assert not refused["ok"] and "hold" in refused["error"]
            assert (await s.cmd(cmd="hil_hold", on=False))["hold"] is False
            assert (await s.cmd(cmd="console_open", device=board.link))["ok"]  # control
            s.close()
    run_console(tmp_path, body)


def test_console_is_refused_while_a_flash_runs_and_opens_when_it_does_not(tmp_path):
    async def body(h, board):
        async with ws_connect(h) as link:
            s = Session(link)
            h.hil._swd_task = asyncio.create_task(asyncio.sleep(600))  # an SWD flash.sh that is still going
            refused = await s.cmd(cmd="console_open", device=board.link)
            assert not refused["ok"] and "flash is running" in refused["error"]
            h.hil._swd_task.cancel()
            await asyncio.gather(h.hil._swd_task, return_exceptions=True)
            h._flash_task = asyncio.create_task(asyncio.sleep(600))  # and a picotool flash
            refused = await s.cmd(cmd="console_open", device=board.link)
            assert not refused["ok"] and "flash is running" in refused["error"]
            h._flash_task.cancel()
            await asyncio.gather(h._flash_task, return_exceptions=True)
            assert (await s.cmd(cmd="console_open", device=board.link))["ok"]  # control
            s.close()
    run_console(tmp_path, body)


def test_an_open_console_is_closed_when_the_hold_is_taken_and_a_hil_launch_or_flash_is_refused_while_it_is_open(tmp_path, monkeypatch):
    port, keep = fake_board(tmp_path, "usb-Fake_Board_1-if00")

    async def body(h, board):
        async with ws_connect(h) as link:
            s = Session(link)
            await opened(s, board)
            refused = await s.cmd(cmd="run_start", kind="hil", port=port)
            assert not refused["ok"] and any("console is open" in r for r in refused["refusals"])
            pre = (await s.cmd(cmd="hil_preflight", esc_battery_unplugged=True))["preflight"]
            assert any("console is open" in r for r in pre["refusals"])
            swd = await s.cmd(cmd="flash_swd", kind="hil")
            assert not swd["ok"]
            assert (await s.cmd(cmd="hil_hold", on=True))["hold"] is True
            closed = await s.wait_event("console_state", state="closed")
            assert "hold" in closed["reason"] and h.console.device is None
            assert (await s.cmd(cmd="hil_hold", on=False))["ok"]
            started = await s.cmd(cmd="run_start", kind="hil", port=port)  # control: with the console closed the same request starts
            assert started["ok"], started
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    try:
        run_console(tmp_path, body, hil_command=["sleep", "600"], poll_s=POLL_S)
    finally:
        for fd in keep:
            os.close(fd)


# ---- the path ---------------------------------------------------------------------------------------------------------------------

def test_the_device_path_is_restricted_to_the_by_id_directory_and_a_console_interface(tmp_path):
    outside = tmp_path / "outside"
    outside.mkdir()
    regular = outside / "file"
    regular.write_text("not a device")

    async def body(h, board):
        by_id = tmp_path / "by-id"
        (by_id / "sub").mkdir()
        (by_id / "regular-if00").symlink_to(regular)  # a symlink that leads to a file, not a device
        (by_id / "null-if00").symlink_to("/dev/null")  # a character device under /dev, but not a tty
        (by_id / "dir-if00").symlink_to(outside)
        (outside / "usb-Elsewhere-if00").symlink_to(os.path.realpath(board.link))
        (by_id / "usb-link-if02").symlink_to(os.path.realpath(board.link))
        refused = {
            "traversal": f"{by_id}/../by-id/{NAME}",
            "dotdot out": f"{by_id}/../outside/usb-Elsewhere-if00",
            "relative": f"by-id/{NAME}",
            "bare name": NAME,
            "subdirectory": f"{by_id}/sub/{NAME}",
            "double slash": f"{by_id}//{NAME}",
            "other directory": f"{outside}/usb-Elsewhere-if00",
            "the link interface": f"{by_id}/usb-link-if02",
            "symlink to a file": f"{by_id}/regular-if00",
            "symlink to a directory": f"{by_id}/dir-if00",
            "a device that is no tty": f"{by_id}/null-if00",
            "a raw tty": os.path.realpath(board.link),
            "absent": f"{by_id}/usb-Absent-if00",
            "empty": "",
            "nul": f"{by_id}/{NAME}\0",
        }
        async with ws_connect(h) as link:
            s = Session(link)
            for why, path in refused.items():
                reply = await s.cmd(cmd="console_open", device=path)
                assert not reply["ok"], why
                assert h.console.device is None, why
                if why in ("symlink to a file", "symlink to a directory"):  # refused by the path rule itself, not by a later failure to open
                    assert "character device under /dev" in reply["error"], (why, reply)
            for bad in (None, 7, ["a"]):
                assert not (await s.cmd(cmd="console_open", device=bad))["ok"]
            assert not (await s.cmd(cmd="console_open"))["ok"]
            assert not (await s.cmd(cmd="console_open", device=board.link, baud=9600))["ok"]  # a closed list of fields
            assert (await s.cmd(cmd="console_open", device=board.link))["ok"]  # control: the by-id entry itself opens
            assert (await s.cmd(cmd="console_status"))["console"]["devices"] == sorted(
                str(p) for p in by_id.iterdir() if p.name.endswith("-if00"))
            s.close()
    run_console(tmp_path, body)


def test_one_opener_per_port_the_console_and_the_link_cannot_both_hold_it(tmp_path):
    async def body(h, board):
        async with ws_connect(h) as link:
            s = Session(link)
            tty = os.path.realpath(board.link)
            await opened(s, board)
            with pytest.raises(device.DeviceError):  # the link cannot take the console's port
                await h.open_serial(tty, 115200, from_browser=False)
            assert h.device is None
            second = await s.cmd(cmd="console_open", device=board.link)  # nor a second console
            assert not second["ok"] and "close it first" in second["error"]
            assert (await s.cmd(cmd="console_close"))["ok"]
            await h.open_serial(tty, 115200, from_browser=False)  # control: free once the console has closed
            refused = await s.cmd(cmd="console_open", device=board.link)  # and the console cannot take the link's port
            assert not refused["ok"] and h.console.device is None and h.device is not None
            s.close()
    run_console(tmp_path, body)


def test_the_console_never_autoconnects(tmp_path):
    async def body(h, board):
        await asyncio.sleep(POLL_S * 6)
        assert h.console.device is None and h.console.state == "closed" and not list(h.run_dir.glob("console*.log"))
    run_console(tmp_path, body, autoconnect=True, poll_s=POLL_S)


# ---- H-2 --------------------------------------------------------------------------------------------------------------------------

def test_nothing_outlives_the_close_no_task_and_no_file_descriptor(tmp_path):
    def fds():
        out = {}
        for fd in os.listdir("/proc/self/fd"):
            try:
                out[int(fd)] = os.readlink(f"/proc/self/fd/{fd}")
            except OSError:
                pass
        return out

    async def body(h, board):
        await asyncio.sleep(0.1)
        tasks_before = set(asyncio.all_tasks())
        fds_before = fds()
        async with ws_connect(h) as link:
            s = Session(link)
            async with sim_connect(h, path="/ws/console") as ws:
                await opened(s, board)
                await board.write(b"x\n")
                await recv_text(ws, 2)
                await ws.send("y\n")
                await board.read_until(2)
                during = fds()
                assert any(path.endswith("console.log") for path in during.values())  # the log is open now (the control of the check below)
                tty = os.path.realpath(board.link)  # the test holds the slave too: the helper's descriptor is the second one
                assert list(during.values()).count(tty) == list(fds_before.values()).count(tty) + 1
                assert (await s.cmd(cmd="console_close"))["ok"]
                after_close = fds()
                assert not any(path.endswith("console.log") for path in after_close.values())
                assert list(after_close.values()).count(tty) == list(fds_before.values()).count(tty)
                assert (await s.cmd(cmd="console_open", device=board.link))["ok"]  # open and close once more
                assert (await s.cmd(cmd="console_close"))["ok"]
            s.close()
            await asyncio.gather(s._pump_task, return_exceptions=True)
        assert await asyncio.to_thread(wait_for, lambda: h.console.client is None and h.client is None)
        await asyncio.sleep(0.1)
        leftover = [t for t in asyncio.all_tasks() if t not in tasks_before and not t.done()]
        assert leftover == [], leftover
        assert sorted(fds().values()) == sorted(fds_before.values())
    run_console(tmp_path, body)
