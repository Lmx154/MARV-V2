"""HIL from the GCS (ground-segment unit GX-10, tools/gs/hil.py): the preflight refusals of an S4b launch, each with a control (the
same setup with that condition satisfied passes), the launch refused while the heavy lock is held, the verdict form, the bench-image
kinds of the SWD flash (a fake flash.sh) and H-2 (nothing outlives a stopped run or a stopped helper). Fakes only: a pty for the board's
port, a fake sysfs for the probe and the board, a lock file for the heavy lock, a stand-in for the run and for flash.sh. No board, no
probe, no real port is opened.
"""

import asyncio
import fcntl
import json
import os
import stat
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "gs"))

import device  # noqa: E402
import hil as hil_mod  # noqa: E402
import runs as runs_mod  # noqa: E402
from test_gs_helper import (  # noqa: E402  (module-level helpers of the sibling test file)
    POLL_S, Session, alive, fake_board, hold_heavy, release_heavy, repo_state, run_with_helper, wait_for, ws_connect)

PROBE_SERIAL = "TESTPROBE0001"  # a fake probe serial
BOARD_SERIAL = "TESTBOARD0001"  # a fake board serial
PORT_NAME = f"usb-Raspberry_Pi_Pico_{BOARD_SERIAL}-if00"
S4B_EXPECT = ["--expect", "none", "--gui"]


def usb_device(tmp_path, name, vendor, product, serial):
    d = tmp_path / "sys" / "bus" / "usb" / "devices" / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "idVendor").write_text(vendor + "\n")
    (d / "idProduct").write_text(product + "\n")
    (d / "serial").write_text(serial + "\n")


class Host:
    """The fake bench: probe and board on the fake sysfs, the board's port a pty under the fake by-id directory."""

    def __init__(self, tmp_path, monkeypatch, *, probe=True, board=True, interface="Board CDC"):
        self.keep = []
        self.port = None
        if probe:
            monkeypatch.setenv("MARV_PROBE_SERIAL", PROBE_SERIAL)
            usb_device(tmp_path, "1-3.3", "2e8a", "000c", PROBE_SERIAL)
        else:
            monkeypatch.delenv("MARV_PROBE_SERIAL", raising=False)
        if board:
            usb_device(tmp_path, "1-1", "2e8a", "0009", BOARD_SERIAL)
            self.port, k = fake_board(tmp_path, PORT_NAME, interface=interface)
            self.keep.extend(k)
        (tmp_path / "by-id").mkdir(exist_ok=True)

    def close(self):
        for fd in self.keep:
            try:
                os.close(fd)
            except OSError:
                pass


def run_host(tmp_path, monkeypatch, body, *, host_kwargs=None, **kwargs):
    host = Host(tmp_path, monkeypatch, **(host_kwargs or {}))
    kwargs.setdefault("hil_command", ["sleep", "600"])
    kwargs.setdefault("poll_s", POLL_S)
    try:
        return run_with_helper(tmp_path, lambda h: body(h, host), **kwargs)
    finally:
        host.close()


async def preflight(s, host, **over):
    msg = {"cmd": "hil_preflight", "esc_battery_unplugged": True, **over}
    reply = await s.cmd(**msg)
    assert reply["ok"], reply
    return reply["preflight"]


def states(pre):
    return {c["id"]: c["state"] for c in pre["checks"]}


def detail(pre, check_id):
    return next(c["detail"] for c in pre["checks"] if c["id"] == check_id)


async def hold(s):
    assert (await s.cmd(cmd="hil_hold", on=True))["hold"] is True


ALL = ("probe", "board", "hil_image", "esc_battery", "host_idle", "ports")


# ---- preflight: the control, then every refusal -------------------------------------------------------------------------------------

def test_preflight_passes_when_every_condition_holds_names_the_s4b_command_and_starts_nothing(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            pre = await preflight(s, host)
            assert pre["allowed"] and pre["refusals"] == []
            assert states(pre) == {c: "ok" for c in ALL}
            assert pre["port"] == host.port and pre["seconds"] == hil_mod.S4B_SECONDS
            assert h.runs.active() is None and not (h.run_dir / "runs").exists()
            s.close()
    run_host(tmp_path, monkeypatch, body, hil_command=None)


def test_preflight_argv_is_run_hil_s4_expect_none_gui_on_the_boards_port(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            argv = (await preflight(s, host, seconds=45))["argv"]
            assert argv[1].endswith("tools/sim/run_hil_s4.py")
            assert argv[argv.index("--port") + 1] == host.port
            assert argv[argv.index("--expect") + 1] == "none" and "--gui" in argv
            assert argv[argv.index("--seconds") + 1] == "45.0"
            assert not any("cmake" in a or "ninja" in a for a in argv)
            s.close()
    run_host(tmp_path, monkeypatch, body, hil_command=None)


def refused_only(pre, check_id, text):
    assert not pre["allowed"]
    assert states(pre)[check_id] == "refused", pre["checks"]
    assert {i for i, st in states(pre).items() if st == "refused"} == {check_id}, pre["checks"]
    assert any(text in r for r in pre["refusals"]), pre["refusals"]
    assert text in detail(pre, check_id)


def test_no_debug_probe_is_refused_by_name_and_the_same_setup_with_the_probe_passes(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            # the probe's serial is not set in the helper's environment
            monkeypatch.delenv("MARV_PROBE_SERIAL")
            refused_only(await preflight(s, host), "probe", "MARV_PROBE_SERIAL is not set")
            # set, but no probe with that serial on USB
            monkeypatch.setenv("MARV_PROBE_SERIAL", "OTHERPROBE")
            refused_only(await preflight(s, host), "probe", "no debug probe: no USB device 2e8a:000c with serial OTHERPROBE")
            # the serial of a probe that is a board (right serial, wrong product): not a probe
            usb_device(tmp_path, "1-9", "2e8a", "0009", "OTHERPROBE")
            refused_only(await preflight(s, host), "probe", "no debug probe")
            # control: the probe is there
            monkeypatch.setenv("MARV_PROBE_SERIAL", PROBE_SERIAL)
            assert (await preflight(s, host))["allowed"]
            s.close()
    run_host(tmp_path, monkeypatch, body)


def test_no_board_is_refused_and_the_same_setup_with_the_board_passes(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            pre = await preflight(s, host)
            refused_only(pre, "board", "no board: no USB device 2e8a:0009")
            assert states(pre)["hil_image"] == "skipped"
            # control: the board appears with the HIL image's port
            usb_device(tmp_path, "1-1", "2e8a", "0009", BOARD_SERIAL)
            host.port, k = fake_board(tmp_path, PORT_NAME, interface="Board CDC")
            host.keep.extend(k)
            assert (await preflight(s, host))["allowed"]
            s.close()
    run_host(tmp_path, monkeypatch, body, host_kwargs=dict(board=False))


def test_a_board_not_running_the_hil_image_is_refused_and_the_hil_image_passes(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            pre = await preflight(s, host)
            refused_only(pre, "hil_image", "the board is not running the HIL image")
            assert "'MARV console'" in detail(pre, "hil_image") and pre["port"] is None
            # the board's other interface does not make it the HIL image either
            _, k = fake_board(tmp_path, f"usb-Raspberry_Pi_Pico_{BOARD_SERIAL}-if02", interface="MARV link")
            host.keep.extend(k)
            refused_only(await preflight(s, host), "hil_image", "the board is not running the HIL image")
            # control: the same board with the HIL image's descriptors
            os.unlink(tmp_path / "by-id" / PORT_NAME)
            os.unlink(tmp_path / "by-id" / f"usb-Raspberry_Pi_Pico_{BOARD_SERIAL}-if02")
            host.port, k = fake_board(tmp_path, PORT_NAME, interface="Board CDC")
            host.keep.extend(k)
            assert (await preflight(s, host))["allowed"]
            s.close()
    run_host(tmp_path, monkeypatch, body, host_kwargs=dict(interface="MARV console"))


def test_the_esc_battery_acknowledgement_must_be_ticked_and_the_same_request_with_it_passes(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            no_ack = (await s.cmd(cmd="hil_preflight"))["preflight"]
            refused_only(no_ack, "esc_battery", "the ESC battery acknowledgement is not ticked")
            refused_only(await preflight(s, host, esc_battery_unplugged=False), "esc_battery", "not ticked")
            assert not (await s.cmd(cmd="hil_preflight", esc_battery_unplugged="yes"))["ok"]  # not a boolean
            assert (await preflight(s, host, esc_battery_unplugged=True))["allowed"]  # control
            # and hil_start refuses it too, starting nothing
            refused = await s.cmd(cmd="hil_start")
            assert not refused["ok"] and "ESC battery" in refused["error"] and h.runs.active() is None
            s.close()
    run_host(tmp_path, monkeypatch, body)


def test_a_host_that_is_not_idle_is_refused_heavy_lock_or_another_sim_and_the_idle_host_passes(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            proc = hold_heavy(tmp_path)
            try:
                pre = await preflight(s, host)
                refused_only(pre, "host_idle", "the heavy lock is held")
                assert "heavy.sh" in detail(pre, "host_idle") or "sleep" in detail(pre, "host_idle")  # the holder is named
            finally:
                release_heavy(proc)
            assert (await preflight(s, host))["allowed"]  # control: lock free
            # another simulation is running (H-1)
            assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]
            refused_only(await preflight(s, host), "host_idle", "one simulation at a time")
            assert (await s.cmd(cmd="run_stop"))["ok"]
            assert (await preflight(s, host))["allowed"]  # control: the sim is gone
            s.close()
    run_host(tmp_path, monkeypatch, body, sil_command=["sleep", "600"])


def test_the_boards_ports_must_be_closed_and_autoconnect_suspended_each_reason_named_and_the_held_board_passes(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            # autoconnect enabled and not suspended
            pre = await preflight(s, host)
            refused_only(pre, "ports", "autoconnect is not suspended")
            # control: the hold suspends it
            await hold(s)
            assert (await preflight(s, host))["allowed"]
            # the helper holds a serial port
            link, k = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_OTHER-if02")
            host.keep.extend(k)
            await h.open_serial(link, 115200, from_browser=False)
            refused_only(await preflight(s, host), "ports", f"the helper has a serial port open ({link})")
            # control: hold closes the helper's serial port
            await hold(s)
            assert h.device is None and (await preflight(s, host))["allowed"]
            # another process holds the board's port
            fd = os.open(os.path.realpath(host.port), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                refused_only(await preflight(s, host), "ports", f"another process holds the board's port {host.port}")
            finally:
                os.close(fd)
            assert (await preflight(s, host))["allowed"]  # control: released
            # taking the hold off makes autoconnect count again
            assert (await s.cmd(cmd="hil_hold", on=False))["hold"] is False
            refused_only(await preflight(s, host), "ports", "autoconnect is not suspended")
            # autoconnect switched off by the user is as good as suspended
            assert (await s.cmd(cmd="autoconnect", enabled=False))["ok"]
            assert (await preflight(s, host))["allowed"]
            s.close()
    run_host(tmp_path, monkeypatch, body)


def test_every_refusal_is_listed_at_once_with_its_own_reason(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            pre = await preflight(s, host, esc_battery_unplugged=False)
            assert states(pre) == {"probe": "refused", "board": "refused", "hil_image": "skipped", "esc_battery": "refused",
                                   "host_idle": "ok", "ports": "refused"}
            assert len(pre["refusals"]) == 4 and not pre["allowed"]
            s.close()
    run_host(tmp_path, monkeypatch, body, host_kwargs=dict(probe=False, board=False))


def test_preflight_options_are_a_closed_list(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for bad in ({"gui": False}, {"expect": "s4a"}, {"seconds": 0}, {"seconds": True}, {"port": 5}, {"seconds": "9"}):
                for cmd in ("hil_preflight", "hil_start"):
                    reply = await s.cmd(cmd=cmd, esc_battery_unplugged=True, **bad)
                    assert not reply["ok"], (cmd, bad)
            assert h.runs.active() is None
            # a port of the by-id directory that is not the HIL image's is refused by the hil_image check
            pre = await preflight(s, host, port=str(tmp_path / "by-id" / "usb-Nope-if00"))
            assert states(pre)["hil_image"] == "refused" and "not a board port running the HIL image" in detail(pre, "hil_image")
            s.close()
    run_host(tmp_path, monkeypatch, body)


# ---- the launch ---------------------------------------------------------------------------------------------------------------------

def test_a_launch_with_the_heavy_lock_held_is_refused_and_starts_nothing_and_the_same_launch_starts_when_it_is_free(tmp_path, monkeypatch):
    ran = tmp_path / "hil.ran"

    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            proc = hold_heavy(tmp_path)
            try:
                refused = await s.cmd(cmd="hil_start", esc_battery_unplugged=True)
                assert not refused["ok"] and "heavy lock is held" in refused["error"] and refused["refusals"]
                assert refused["preflight"]["allowed"] is False
                assert not ran.exists() and h.runs.active() is None and (await s.cmd(cmd="run_list"))["runs"] == []
            finally:
                release_heavy(proc)
            started = await s.cmd(cmd="hil_start", esc_battery_unplugged=True)  # control: the lock is free
            assert started["ok"] and started["run"]["kind"] == "hil" and started["preflight"]["allowed"]
            assert await asyncio.to_thread(wait_for, ran.exists)
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    run_host(tmp_path, monkeypatch, body, hil_command=["sh", "-c", f"touch {ran}; sleep 600"])


def test_hil_start_launches_a_hil_run_with_expect_none_gui_and_the_s4b_seconds_after_the_preflight(tmp_path, monkeypatch):
    seen = []
    original = runs_mod.RunSupervisor.start

    def spy(self, kind, opts):
        seen.append((kind, dict(opts)))
        return original(self, kind, opts)
    monkeypatch.setattr(runs_mod.RunSupervisor, "start", spy)

    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            started = await s.cmd(cmd="hil_start", esc_battery_unplugged=True)
            assert started["ok"]
            assert seen == [("hil", {"port": host.port, "seconds": hil_mod.S4B_SECONDS, "expect": "none", "gui": True})]
            assert started["run"]["options"]["expect"] == "none" and started["run"]["options"]["gui"] is True
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    run_host(tmp_path, monkeypatch, body)


def test_a_launch_never_builds(tmp_path, monkeypatch):
    """H-3: every process a launch spawns is recorded; there is exactly one, the run, and none is a build tool."""
    spawned = []
    real_popen = subprocess.Popen

    class Recording(real_popen):
        def __init__(self, args, *a, **kw):
            spawned.append(list(args))
            super().__init__(args, *a, **kw)
    monkeypatch.setattr(subprocess, "Popen", Recording)

    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            await preflight(s, host)
            assert spawned == []  # the preflight spawns nothing
            started = await s.cmd(cmd="hil_start", esc_battery_unplugged=True, seconds=5)
            assert started["ok"]
            assert spawned == [["sleep", "600"]]
            assert not any(any(t in arg for t in ("cmake", "ninja", "make")) for argv in spawned for arg in argv)
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    run_host(tmp_path, monkeypatch, body)


def test_autoconnect_is_suspended_for_the_run_and_restored_after_it_without_changing_the_users_setting(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            await s.wait_event("autoconnect", state="suspended")
            assert (await s.cmd(cmd="hil_start", esc_battery_unplugged=True))["ok"]
            await s.wait_event("autoconnect", state="suspended", reason="a HIL run owns the board")
            during = (await s.cmd(cmd="status"))["status"]["autoconnect"]
            assert during["enabled"] is True and during["suspended"] == "a HIL run owns the board"
            assert (await s.cmd(cmd="run_stop"))["ok"]
            await s.wait_event("autoconnect", state="resumed")
            after = (await s.cmd(cmd="status"))["status"]
            assert after["autoconnect"]["enabled"] is True and after["autoconnect"]["suspended"] is None
            assert after["hil"]["hold"] is False and after["hil"]["hold_run"] is None
            s.close()
    run_host(tmp_path, monkeypatch, body)


def test_a_hold_with_no_run_stays_until_released_and_a_serial_port_the_helper_holds_is_closed_by_it(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            link, k = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_OTHER-if02")
            host.keep.extend(k)
            await h.open_serial(link, 115200, from_browser=False)
            await hold(s)
            assert h.device is None and (await s.wait_event("device_closed"))["reason"] == "held for a HIL run"
            await s.wait_event("autoconnect", state="suspended")
            await asyncio.sleep(POLL_S * 5)
            assert h.hil.hold and h.device is None
            assert (await s.cmd(cmd="hil_hold", on=False))["hold"] is False
            await s.wait_event("autoconnect", state="resumed")
            assert not (await s.cmd(cmd="hil_hold", on="yes"))["ok"]
            s.close()
    run_host(tmp_path, monkeypatch, body)


# ---- H-2 ----------------------------------------------------------------------------------------------------------------------------

def test_a_stopped_hil_run_leaves_no_process_and_the_run_is_its_own_process_group(tmp_path, monkeypatch):
    marker = tmp_path / "child.pid"
    pids = {}

    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            run = (await s.cmd(cmd="hil_start", esc_battery_unplugged=True))["run"]
            assert await asyncio.to_thread(wait_for, lambda: marker.exists() and marker.read_text().strip())
            pids["child"], pids["leader"] = int(marker.read_text()), run["pid"]
            assert os.getpgid(run["pid"]) == run["pid"] != os.getpgid(os.getpid())
            assert os.getpgid(pids["child"]) == run["pid"]  # the child is in the run's group
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    run_host(tmp_path, monkeypatch, body, hil_command=["sh", "-c", f"sleep 600 & echo $! > {marker}; wait"])
    for pid in pids.values():
        assert wait_for(lambda: not alive(pid)), f"process {pid} outlived the stopped run"


def test_a_helper_that_stops_during_a_hil_run_leaves_no_process(tmp_path, monkeypatch):
    marker = tmp_path / "child.pid"
    pids = {}

    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            run = (await s.cmd(cmd="hil_start", esc_battery_unplugged=True))["run"]
            assert await asyncio.to_thread(wait_for, lambda: marker.exists() and marker.read_text().strip())
            pids["child"], pids["leader"] = int(marker.read_text()), run["pid"]
            s.close()
    run_host(tmp_path, monkeypatch, body, hil_command=["sh", "-c", f"sleep 600 & echo $! > {marker}; wait"])
    for pid in pids.values():
        assert wait_for(lambda: not alive(pid)), f"process {pid} outlived the helper"


# ---- the verdict --------------------------------------------------------------------------------------------------------------------

def test_the_verdict_lands_in_the_run_directory_next_to_the_report_and_is_not_overwritten(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await hold(s)
            run = (await s.cmd(cmd="hil_start", esc_battery_unplugged=True))["run"]
            run_id = run["run_id"]
            early = await s.cmd(cmd="hil_verdict", run_id=run_id, mode="ANGLE", text="x")
            assert not early["ok"] and "still going" in early["error"]
            assert (await s.cmd(cmd="run_stop"))["ok"]
            out = Path(run["out_dir"])
            assert out == h.run_dir / "runs" / run_id / "out"
            (out / "marv_x_hil_n25.hil_report.txt").write_text("the report\n")  # what run_hil_s4 writes
            for bad in ({"mode": "ACRO "}, {"mode": "angle"}, {"mode": None}, {"text": ""}, {"text": "   "}, {"text": 7},
                        {"text": "x" * (hil_mod.VERDICT_TEXT_MAX + 1)}, {"flown": True}):
                msg = {"run_id": run_id, "mode": "ANGLE", "text": "flew it", **bad}
                assert not (await s.cmd(cmd="hil_verdict", **msg))["ok"], bad
            assert not (out / "verdict.json").exists()  # nothing written by a refused form
            saved = await s.cmd(cmd="hil_verdict", run_id=run_id, mode="ANGLE", text="Took off, held a hover, landed.\nSecond line.")
            assert saved["ok"] and saved["path"] == str(out / "verdict.json")
            on_disk = json.loads((out / "verdict.json").read_text())
            assert on_disk["mode"] == "ANGLE" and on_disk["text"] == "Took off, held a hover, landed.\nSecond line."
            assert on_disk["run_id"] == run_id and on_disk["report"] == "marv_x_hil_n25.hil_report.txt" and on_disk["schema"] == 1
            assert (out / "marv_x_hil_n25.hil_report.txt").exists()  # beside the report
            again = await s.cmd(cmd="hil_verdict", run_id=run_id, mode="ACRO", text="changed my mind")
            assert not again["ok"] and "already has a verdict" in again["error"]
            assert json.loads((out / "verdict.json").read_text()) == on_disk
            assert (await s.cmd(cmd="status"))["status"]["hil"]["verdict"] == on_disk
            assert h.token not in (out / "verdict.json").read_text()
            s.close()
    run_host(tmp_path, monkeypatch, body)


def test_a_verdict_names_a_hil_run_under_the_runs_directory_only(tmp_path, monkeypatch):
    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            sil = (await s.cmd(cmd="run_start", kind="sil"))["run"]["run_id"]
            assert (await s.cmd(cmd="run_stop"))["ok"]
            not_hil = await s.cmd(cmd="hil_verdict", run_id=sil, mode="ANGLE", text="x")
            assert not not_hil["ok"] and "not a HIL run" in not_hil["error"]
            for run_id in ("../../..", "nope", None, 5, "20200101T000000Z-hil-1"):
                assert not (await s.cmd(cmd="hil_verdict", run_id=run_id, mode="ANGLE", text="x"))["ok"], run_id
            assert not list((h.run_dir / "runs").glob("**/verdict.json"))
            s.close()
    run_host(tmp_path, monkeypatch, body, sil_command=["sleep", "600"])


# ---- bench images over SWD ----------------------------------------------------------------------------------------------------------

def fake_flash_script(tmp_path, *, status=0, sleep=None):
    """A flash.sh stand-in: records its arguments and MARV_PROBE_SERIAL, prints lines like the real one, exits `status`."""
    script = tmp_path / "fake_flash.sh"
    record = tmp_path / "flash.record"
    body = [f'echo "$@" > {record}', f'echo "serial=$MARV_PROBE_SERIAL" >> {record}', 'echo "flash: image $1"', 'echo "** Verified OK **"']
    if sleep:
        body += [f"sleep {sleep} &", f"echo $! > {tmp_path / 'flash.child'}", "wait"]
    script.write_text("#!/usr/bin/env bash\n" + "\n".join(body) + f"\nexit {status}\n")
    script.chmod(script.stat().st_mode | stat.S_IXUSR)
    return script, record


def hil_tree(tmp_path, *, cache=("MARV_FC3V2_HIL:BOOL=ON", "MARV_FC3V2_HIL_RECEIVER:BOOL=ON"), elf=True):
    tree = tmp_path / "build" / "m33-hil"
    (tree / "fw" / "hal" / "target").mkdir(parents=True)
    (tree / "CMakeCache.txt").write_text("\n".join(["CMAKE_BUILD_TYPE:STRING=Release", *cache]) + "\n")
    if elf:
        (tree / "fw" / "hal" / "target" / "marv_fc3v2.elf").write_bytes(b"\x7fELF-fake-hil")
    return tree


def test_the_motor_capable_bench_kinds_are_refused_naming_the_condition_and_the_hil_kind_flashes(tmp_path, monkeypatch):
    script, record = fake_flash_script(tmp_path)
    hil_tree(tmp_path)

    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for kind in ("bench-profile", "bench-log"):
                refused = await s.cmd(cmd="flash_swd", kind=kind)
                assert not refused["ok"], kind
                assert "only after its props-off TB on the exact image and Luis's presence acknowledgement" in refused["error"], refused
                assert kind in refused["error"] and not record.exists()  # the script never ran
            for kind in ("hil ", "HIL", "flight", "", None, 5, ["hil"]):
                assert not (await s.cmd(cmd="flash_swd", kind=kind))["ok"], kind
            assert not (await s.cmd(cmd="flash_swd"))["ok"]
            assert not (await s.cmd(cmd="flash_swd", kind="hil", elf="/etc/passwd"))["ok"]  # the browser names no path
            assert not record.exists() and h.hil.swd is None
            # control: the hil kind flashes, outside G7 (no image library, no report)
            started = await s.cmd(cmd="flash_swd", kind="hil")
            assert started["ok"] and started["started"] and started["kind"] == "hil"
            done = await s.wait_event("flash_swd_done")
            assert done["ok"] is True and done["exit_status"] == 0 and done["error"] is None
            elf = tmp_path / "build" / "m33-hil" / "fw" / "hal" / "target" / "marv_fc3v2.elf"
            assert done["elf"] == str(elf) and len(done["elf_sha256"]) == 64
            assert record.read_text().splitlines() == [str(elf), f"serial={PROBE_SERIAL}"]
            assert (await s.cmd(cmd="status"))["status"]["hil"]["swd_flash"]["ok"] is True
            assert any(e["line"] == "** Verified OK **" for e in s.seen("flash_swd_progress"))
            s.close()
    run_host(tmp_path, monkeypatch, body, build_root=tmp_path / "build", swd_flash_script=script)


@pytest.mark.parametrize("what", ["no_probe", "no_elf", "not_hil_tree", "no_tree", "serial_open", "run_going", "flash_going"])
def test_an_swd_flash_of_the_hil_kind_is_refused_when_its_precondition_fails_and_the_script_never_runs(tmp_path, monkeypatch, what):
    script, record = fake_flash_script(tmp_path)
    if what == "no_elf":
        hil_tree(tmp_path, elf=False)
    elif what == "not_hil_tree":
        hil_tree(tmp_path, cache=("MARV_FC3V2_HIL:BOOL=ON", "MARV_FC3V2_HIL_RECEIVER:BOOL=OFF"))
    elif what != "no_tree":
        hil_tree(tmp_path)
    expected = {"no_probe": "MARV_PROBE_SERIAL is not set", "no_elf": "no image at", "not_hil_tree": "MARV_FC3V2_HIL_RECEIVER=OFF",
                "no_tree": "absent", "serial_open": "serial port open", "run_going": "one simulation at a time",
                "flash_going": "a flash is running"}[what]

    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            if what == "serial_open":
                link, k = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_OTHER-if02")
                host.keep.extend(k)
                await h.open_serial(link, 115200, from_browser=False)
            if what == "run_going":
                assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]
            if what == "flash_going":
                h._flash_task = asyncio.create_task(asyncio.sleep(600))
            refused = await s.cmd(cmd="flash_swd", kind="hil")
            assert not refused["ok"] and expected in refused["error"], refused
            assert not record.exists() and h.hil.swd is None
            if what == "flash_going":
                h._flash_task.cancel()
            s.close()
    run_host(tmp_path, monkeypatch, body, host_kwargs=dict(probe=(what != "no_probe"), board=False), build_root=tmp_path / "build", swd_flash_script=script, sil_command=["sleep", "600"])


def test_a_failing_flash_script_is_reported_not_ok_and_a_hung_one_is_killed_with_its_group(tmp_path, monkeypatch):
    hil_tree(tmp_path)
    failing, _ = fake_flash_script(tmp_path, status=3)

    async def fail_body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="flash_swd", kind="hil"))["ok"]
            done = await s.wait_event("flash_swd_done")
            assert done["ok"] is False and done["exit_status"] == 3 and "status 3" in done["error"]
            s.close()
    run_host(tmp_path, monkeypatch, fail_body, host_kwargs=dict(board=False), build_root=tmp_path / "build", swd_flash_script=failing)

    hung, _ = fake_flash_script(tmp_path, sleep=600)
    monkeypatch.setattr(hil_mod, "SWD_TIMEOUT_S", 1.0)
    monkeypatch.setattr(hil_mod, "SWD_STOP_S", 2.0)

    async def hung_body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="flash_swd", kind="hil"))["ok"]
            done = await s.wait_event("flash_swd_done", timeout=30)
            assert done["ok"] is False and "did not end within" in done["error"]
            s.close()
    run_host(tmp_path, monkeypatch, hung_body, host_kwargs=dict(board=False), build_root=tmp_path / "build", swd_flash_script=hung)
    child = int((tmp_path / "flash.child").read_text())
    assert wait_for(lambda: not alive(child)), "the hung flash.sh's child outlived the timeout (H-2)"


def test_a_helper_that_stops_during_an_swd_flash_leaves_no_process(tmp_path, monkeypatch):
    hil_tree(tmp_path)
    hung, _ = fake_flash_script(tmp_path, sleep=600)

    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="flash_swd", kind="hil"))["ok"]
            assert await asyncio.to_thread(wait_for, (tmp_path / "flash.child").exists)
            s.close()
    run_host(tmp_path, monkeypatch, body, host_kwargs=dict(board=False), build_root=tmp_path / "build", swd_flash_script=hung)
    child = int((tmp_path / "flash.child").read_text())
    assert wait_for(lambda: not alive(child)), "flash.sh's child outlived the helper (H-2)"


def test_an_swd_flash_suspends_autoconnect_and_blocks_a_second_flash_and_a_run(tmp_path, monkeypatch):
    hil_tree(tmp_path)
    slow, record = fake_flash_script(tmp_path, sleep=2)

    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="flash_swd", kind="hil"))["ok"]
            await s.wait_event("autoconnect", state="suspended")
            again = await s.cmd(cmd="flash_swd", kind="hil")
            assert not again["ok"] and "a flash is running" in again["error"]
            run = await s.cmd(cmd="run_start", kind="sil")
            assert not run["ok"] and "a flash is running" in run["refusals"][0]
            assert not (await s.cmd(cmd="flash", image_hash="x", board={}))["ok"]
            await s.wait_event("flash_swd_done", timeout=30)
            await s.wait_event("autoconnect", state="resumed")
            s.close()
    run_host(tmp_path, monkeypatch, body, host_kwargs=dict(board=False), build_root=tmp_path / "build", swd_flash_script=slow, sil_command=["sleep", "600"])


# ---- W-1 and the unit's pieces ----------------------------------------------------------------------------------------------------

def test_no_hil_command_writes_into_the_repository(tmp_path, monkeypatch):
    """W-1: git status and the diff against HEAD are the same after every new command as before it."""
    script, _ = fake_flash_script(tmp_path)
    hil_tree(tmp_path)
    before = repo_state()
    seen = []

    async def body(h, host):
        async with ws_connect(h) as ws:
            s = Session(ws)

            async def cmd(**msg):
                reply = await s.cmd(**msg)
                seen.append((msg["cmd"], repo_state()))
                return reply
            await cmd(cmd="hil_preflight", esc_battery_unplugged=True)
            await cmd(cmd="hil_hold", on=True)
            started = await cmd(cmd="hil_start", esc_battery_unplugged=True)
            assert started["ok"]
            await cmd(cmd="run_stop")
            await cmd(cmd="hil_verdict", run_id=started["run"]["run_id"], mode="ACRO", text="x")
            await cmd(cmd="flash_swd", kind="bench-profile")
            await cmd(cmd="flash_swd", kind="hil")
            await s.wait_event("flash_swd_done")
            await cmd(cmd="hil_hold", on=False)
            s.close()
    run_host(tmp_path, monkeypatch, body, build_root=tmp_path / "build", swd_flash_script=script)
    assert len(seen) == 8
    for name, state in seen:
        assert state == before, f"{name} changed the repository"


def test_flock_holders_reads_the_kernels_lock_table_without_opening_the_port(tmp_path):
    master, slave = os.openpty()
    path = os.ttyname(slave)
    try:
        assert hil_mod.flock_holders(path) == []
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            assert hil_mod.flock_holders(path) == [os.getpid()]
        finally:
            os.close(fd)
        assert hil_mod.flock_holders(path) == []
    finally:
        os.close(master)
        os.close(slave)


def test_the_helper_and_a_second_opener_exclude_each_other_on_a_port_in_both_orders(tmp_path):
    """The helper's half of the port lock (tools/gs/device.py); the plugin's half is tests/regression/quad/L10/gz/test_hil_port_lock.py."""
    master, slave = os.openpty()
    path = os.ttyname(slave)

    async def main():
        dev = device.SerialDevice(path, 115200, lambda b: None, lambda r: None)
        try:
            other = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            with pytest.raises(BlockingIOError):
                fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)  # the helper holds it: the second opener is refused
            os.close(other)
            with pytest.raises(device.DeviceError, match="cannot configure"):
                device.SerialDevice(path, 115200, lambda b: None, lambda r: None)
        finally:
            dev.close()
        other = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)  # the second opener holds it: the helper is refused
        try:
            with pytest.raises(device.DeviceError, match="cannot configure"):
                device.SerialDevice(path, 115200, lambda b: None, lambda r: None)
        finally:
            os.close(other)
        device.SerialDevice(path, 115200, lambda b: None, lambda r: None).close()  # control: free again
    try:
        asyncio.run(main())
    finally:
        os.close(master)
        os.close(slave)
