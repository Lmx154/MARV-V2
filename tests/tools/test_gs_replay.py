"""The replay of a ground log (tools/gs/replay.py, ground-segment unit GX-11; ground spec GS-8).

The claim: a recorded session replayed through the helper decodes to the values the live session decoded. The fixture is a ground log of a
real Gazebo SIL session (tests/tools/data/gx11/sil_session/ground.log, recorded by
tests/regression/quad/L10/results/gcs/gx11/record_sil_session.py) with, beside it, the frames the live client decoded from the bytes it
received over the WebSocket (live.decoded.jsonl.xz, decoded by tests/tools/replay_decode.py). The test replays the log through a real helper
over a real WebSocket, decodes what the browser's side receives with the same decoder, and compares frame by frame. The control: the same log
with one byte corrupted, and with two records swapped, replays and does NOT decode to the live values (and a rewritten copy with no change does).

Around it: the pace (recorded and as fast as possible), the listing and its refusals, and "never to a vehicle": a replay is refused while a
device is open or a run is going, opens no device, autoconnect is suspended while it plays, the browser's bytes meanwhile reach no device and
not the ground log, and the replayed bytes are not written to the running session's ground log.
"""

import asyncio
import json
import lzma
import shutil
import socket
import sys
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "tools" / "gs"))
import groundlog  # noqa: E402
import replay as replay_mod  # noqa: E402
import replay_decode  # noqa: E402
import test_gs_helper as gh  # noqa: E402

FIXTURE_DIR = HERE / "data" / "gx11"
SIL_LOG = FIXTURE_DIR / "sil_session" / "ground.log"
LIVE_DECODED = FIXTURE_DIR / "sil_session" / "live.decoded.jsonl.xz"
TIMEOUT_S = gh.TIMEOUT_S
SLACK_S = 0.1  # labelled harness value: scheduling slack allowed on a lower bound of elapsed time
SLOW_READ_S = 0.002  # labelled harness value: the slow browser's pause after each frame


@pytest.fixture(scope="module")
def dialect(tmp_path_factory):
    return replay_decode.load_dialect(tmp_path_factory.mktemp("defs"))


def live_decoded():
    with lzma.open(LIVE_DECODED, "rt", encoding="utf-8") as f:
        return [json.loads(line) for line in f]


def write_log(path: Path, records, *, wall_ns=1, mono_ns=0):
    """A ground log of (kind, t_ns, data) records, written byte for byte in the layout of tools/gs/groundlog.py."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(groundlog.HEADER.pack(groundlog.MAGIC, wall_ns, mono_ns))
        for kind, t_ns, data in records:
            f.write(groundlog.RECORD.pack(kind, t_ns, len(data)) + data)


def run_dir_of(tmp_path, name):
    return tmp_path / "replays" / name


async def play(ws, log: Path, mode: str, *, until=("finished", "stopped", "failed")):
    """Starts a replay and collects what the browser's side receives until it ends: (the binary frames, the replay_state events, the reply)."""
    await ws.send(json.dumps({"cmd": "replay_start", "log": str(log), "mode": mode, "id": 7}))
    frames, events, reply = [], [], None
    while True:
        m = await asyncio.wait_for(ws.recv(), TIMEOUT_S)
        if isinstance(m, bytes):
            frames.append((time.monotonic(), m))
            continue
        msg = json.loads(m)
        if msg.get("event") == "replay_state":
            events.append(msg)
            if msg["state"] in until:
                return frames, events, reply
        elif msg.get("id") == 7:
            reply = msg
            if not msg["ok"]:
                return frames, events, reply


def with_replays(tmp_path, body, **kw):
    return gh.run_with_helper(tmp_path, body, replay_dirs=[tmp_path / "replays"], **kw)


# ---- the claim --------------------------------------------------------------------------------------------------------------

def test_replayed_decoded_values_equal_the_live_recorded_ones_for_a_sil_session(tmp_path, dialect):
    live = live_decoded()
    recorded = b"".join(groundlog.byte_stream(SIL_LOG, groundlog.RX))
    # The fixture is a session of a real vehicle: every stream of the live view and the identity are in it (a fixture that lost them proves nothing).
    kinds = {m["type"] for m in live}
    assert {"HEARTBEAT", "MARV_IDENTITY", "SYS_STATUS", "ATTITUDE", "ATTITUDE_QUATERNION", "RC_CHANNELS", "ATTITUDE_TARGET", "HIGHRES_IMU",
            "ACTUATOR_OUTPUT_STATUS", "MARV_FC_STATUS", "MARV_ESTIMATOR"} <= kinds
    assert replay_decode.decode(dialect, recorded) == live, "the fixture's own bytes must decode to the live-recorded values"
    shutil.copytree(FIXTURE_DIR / "sil_session", run_dir_of(tmp_path, "sil_session"))
    log = run_dir_of(tmp_path, "sil_session") / "ground.log"

    async def body(h):
        async with gh.ws_connect(h) as ws:
            frames, events, reply = await play(ws, log, "fast")
            assert reply is None or reply["ok"]
            replayed = b"".join(f for _, f in frames)
            assert replayed == recorded
            assert replay_decode.decode(dialect, replayed) == live
            assert [e["state"] for e in events][0] == "playing" and events[-1]["state"] == "finished"
            assert events[-1]["records_sent"] == events[-1]["records_total"] > 0 and events[-1]["bytes_sent"] == len(recorded)
        return h.run_dir
    run_dir = with_replays(tmp_path, body)
    # The replay left the running session's ground log without a received or a sent byte: only events name it.
    kinds = [r.kind for r in groundlog.read_records(run_dir / "ground.log")]
    assert groundlog.RX not in kinds and groundlog.TX not in kinds
    texts = [r.data.decode() for r in groundlog.read_records(run_dir / "ground.log") if r.kind == groundlog.EVENT]
    assert any(t.startswith("replay started:") for t in texts) and any(t.startswith("replay finished:") for t in texts)


def corrupted_copy(tmp_path, name, change):
    """The fixture's log rewritten with `change(records)` applied; change=None rewrites it unchanged (the control of the control)."""
    records = [(r.kind, r.t_ns, r.data) for r in groundlog.read_records(SIL_LOG)]
    if change is not None:
        change(records)
    write_log(run_dir_of(tmp_path, name) / "ground.log", records)
    return run_dir_of(tmp_path, name) / "ground.log"


def flip_a_byte(records):
    rx = [i for i, r in enumerate(records) if r[0] == groundlog.RX and len(r[2]) > 40]
    i = rx[len(rx) // 2]
    kind, t, data = records[i]
    records[i] = (kind, t, data[:20] + bytes([data[20] ^ 0xFF]) + data[21:])


def swap_two_records(records):
    rx = [i for i, r in enumerate(records) if r[0] == groundlog.RX and len(r[2]) > 40]
    a, b = rx[len(rx) // 3], rx[len(rx) // 3 + 1]
    (ka, ta, da), (kb, tb, db) = records[a], records[b]
    records[a], records[b] = (ka, ta, db), (kb, tb, da)


@pytest.mark.parametrize("name,change,equal", [("rewritten", None, True), ("one-byte-corrupted", flip_a_byte, False), ("two-records-swapped", swap_two_records, False)])
def test_the_equality_fails_for_a_corrupted_or_reordered_log_and_holds_for_an_unchanged_copy(tmp_path, dialect, name, change, equal):
    live = live_decoded()
    log = corrupted_copy(tmp_path, name, change)

    async def body(h):
        async with gh.ws_connect(h) as ws:
            frames, _, _ = await play(ws, log, "fast")
            return replay_decode.decode(dialect, b"".join(f for _, f in frames))
    replayed = with_replays(tmp_path, body)
    assert (replayed == live) is equal
    if not equal:
        assert replayed != live


# ---- the pace ---------------------------------------------------------------------------------------------------------------

GAP_S = 0.3  # labelled harness value: the recorded gap between the chunks of the synthetic log
CHUNKS = [b"first", b"second", b"third"]


def synthetic(tmp_path, name="synthetic", gap_ns=int(GAP_S * 1e9)):
    # TX and EVENT records between the RX ones: not replayed.
    records = [(groundlog.EVENT, 100, b"opened"), (groundlog.RX, 1_000, CHUNKS[0]), (groundlog.TX, 1_500, b"request"),
               (groundlog.RX, 1_000 + gap_ns, CHUNKS[1]), (groundlog.RX, 1_000 + 2 * gap_ns, CHUNKS[2])]
    write_log(run_dir_of(tmp_path, name) / "ground.log", records)
    return run_dir_of(tmp_path, name) / "ground.log"


def test_paced_replay_keeps_the_recorded_gaps_and_fast_replay_does_not(tmp_path):
    log = synthetic(tmp_path)

    async def body(h):
        async with gh.ws_connect(h) as ws:
            t0 = time.monotonic()
            fast, _, _ = await play(ws, log, "fast")
            fast_s = time.monotonic() - t0
            t0 = time.monotonic()
            paced, events, _ = await play(ws, log, "paced")
            paced_s = time.monotonic() - t0
            return fast, fast_s, paced, paced_s, events
    fast, fast_s, paced, paced_s, events = with_replays(tmp_path, body)
    for frames in (fast, paced):
        assert [f for _, f in frames] == CHUNKS  # every received chunk, in order, unchanged; no TX, no EVENT
    assert fast_s < GAP_S
    assert paced_s >= 2 * GAP_S - SLACK_S
    times = [t for t, _ in paced]
    assert times[1] - times[0] >= GAP_S - SLACK_S and times[2] - times[1] >= GAP_S - SLACK_S
    assert events[-1]["position_s"] == pytest.approx(2 * GAP_S) and events[-1]["duration_s"] == pytest.approx(2 * GAP_S)


def test_a_slow_browser_loses_no_byte_in_a_fast_replay(tmp_path):
    chunk = bytes(range(256)) * 256  # 64 KiB
    count = 3 * helper_queue_limit() // len(chunk)  # three times what the helper queues for a live device before it drops chunks
    write_log(run_dir_of(tmp_path, "big") / "ground.log", [(groundlog.RX, i, chunk) for i in range(count)])
    log = run_dir_of(tmp_path, "big") / "ground.log"

    async def body(h):
        async with gh.ws_connect(h) as ws:
            await ws.send(json.dumps({"cmd": "replay_start", "log": str(log), "mode": "fast", "id": 1}))
            got, queued_max = 0, 0
            while got < count * len(chunk):
                m = await asyncio.wait_for(ws.recv(), TIMEOUT_S)
                if isinstance(m, bytes):
                    got += len(m)
                    queued_max = max(queued_max, h._queued_bytes)
                    await asyncio.sleep(SLOW_READ_S)
            return got, h.dropped_bytes, queued_max
    got, dropped, queued_max = with_replays(tmp_path, body)
    assert got == count * len(chunk) and dropped == 0
    assert queued_max > helper_queue_limit() // 2, "the replay never filled the browser's queue: the test did not reach the wait"
    assert queued_max <= helper_queue_limit()  # and it waited for room instead of queueing the whole log


def helper_queue_limit():
    return gh.helper_mod.CLIENT_QUEUE_MAX_BYTES


# ---- the listing ------------------------------------------------------------------------------------------------------------

def test_the_listing_names_the_logs_with_their_summaries_and_refuses_what_it_does_not_list(tmp_path):
    synthetic(tmp_path, "good")
    bad = run_dir_of(tmp_path, "bad") / "ground.log"
    bad.parent.mkdir(parents=True)
    bad.write_bytes(b"not a ground log at all, but longer than the header")
    (run_dir_of(tmp_path, "no-log")).mkdir()
    outside = tmp_path / "outside" / "ground.log"
    write_log(outside, [(groundlog.RX, 1, b"x")])

    async def body(h):
        async with gh.ws_connect(h) as ws:
            listed = await gh.control(ws, cmd="replay_list")
            refusals = {}
            for what in (str(outside), str(run_dir_of(tmp_path, "good") / ".." / "good" / "ground.log"), "../../etc/passwd", "ground.log", 5, None):
                refusals[str(what)] = await gh.control(ws, cmd="replay_start", log=what, mode="fast")
            refusals["mode"] = await gh.control(ws, cmd="replay_start", log=str(run_dir_of(tmp_path, "good") / "ground.log"), mode="slow")
            status = await gh.control(ws, cmd="replay_status")
            return listed, refusals, status, str(h.run_dir / "ground.log")
    listed, refusals, status, current = with_replays(tmp_path, body)
    by_name = {e["name"]: e for e in listed["logs"]}
    assert set(by_name) == {Path(current).parent.name, "good", "bad"}  # no-log has no ground.log; outside is not under a listed directory
    good = by_name["good"]
    assert (good["rx_records"], good["rx_bytes"], good["tx_records"], good["events"]) == (3, sum(map(len, CHUNKS)), 1, 1)
    assert good["duration_s"] == pytest.approx(2 * GAP_S) and good["wall_utc"] == "1970-01-01T00:00:00Z" and good["current"] is False
    assert "error" in by_name["bad"] and "rx_records" not in by_name["bad"]
    assert by_name[Path(current).parent.name]["current"] is True
    for key, reply in refusals.items():
        assert reply["ok"] is False, key
    assert "not a ground log the helper lists" in refusals[str(outside)]["error"]
    assert status["replay"]["state"] == "idle" and status["replay"]["records_sent"] == 0


def test_a_log_with_no_received_byte_is_refused_and_a_replay_of_the_running_sessions_own_log_is_allowed(tmp_path):
    write_log(run_dir_of(tmp_path, "tx-only") / "ground.log", [(groundlog.TX, 1, b"request"), (groundlog.EVENT, 2, b"x")])

    async def body(h):
        async with gh.ws_connect(h) as ws:
            empty = await gh.control(ws, cmd="replay_start", log=str(run_dir_of(tmp_path, "tx-only") / "ground.log"), mode="fast")
            h.log.write(groundlog.RX, b"recorded by this session")  # as a live device's chunk would be
            own = await gh.control(ws, cmd="replay_start", log=str(h.run_dir / "ground.log"), mode="fast")
            return empty, own
    empty, own = with_replays(tmp_path, body)
    assert empty["ok"] is False and "nothing to replay" in empty["error"]
    assert own["ok"] is True


# ---- never to a vehicle -----------------------------------------------------------------------------------------------------

def test_a_replay_is_refused_while_a_device_is_open_and_while_a_run_is_going(tmp_path):
    log = synthetic(tmp_path)
    vehicle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vehicle.bind(("127.0.0.1", 0))
    vehicle.setblocking(False)

    async def body(h):
        async with gh.ws_connect(h) as ws:
            assert (await gh.control(ws, cmd="open", udp_port=vehicle.getsockname()[1]))["ok"]
            with_device = await gh.control(ws, cmd="replay_start", log=str(log), mode="fast")
            assert (await gh.control(ws, cmd="close"))["ok"]
            started = await gh.control(ws, cmd="run_start", kind="sil")
            with_run = await gh.control(ws, cmd="replay_start", log=str(log), mode="fast")
            stopped = await gh.control(ws, cmd="run_stop")
            after = await gh.control(ws, cmd="replay_start", log=str(log), mode="fast")  # the control: both closed, it plays
            return with_device, started, with_run, stopped, after
    try:
        with_device, started, with_run, stopped, after = with_replays(tmp_path, body, sil_command=[sys.executable, "-c", "import time; time.sleep(60)"])
    finally:
        vehicle.close()
    assert with_device["ok"] is False and "device open" in with_device["error"] and with_device["refusals"]
    assert started["ok"] is True
    assert with_run["ok"] is False and "run" in with_run["error"] and "going" in with_run["error"]
    assert stopped["ok"] is True
    assert after["ok"] is True


def test_while_it_plays_the_replay_opens_no_device_starts_no_run_suspends_autoconnect_and_sends_nothing_to_a_vehicle(tmp_path):
    log = synthetic(tmp_path, gap_ns=int(2e9))  # long gaps: it is still playing while the commands below are answered
    vehicle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vehicle.bind(("127.0.0.1", 0))
    vehicle.setblocking(False)
    port = vehicle.getsockname()[1]
    sent = b"a request the GCS would send to a vehicle"

    async def body(h):
        async with gh.ws_connect(h) as ws:
            assert (await gh.control(ws, cmd="replay_start", log=str(log), mode="paced"))["ok"]
            answers = {
                "again": await gh.control(ws, cmd="replay_start", log=str(log), mode="paced"),
                "open_udp": await gh.control(ws, cmd="open", udp_port=port),
                "open_serial": await gh.control(ws, cmd="open", serial="/dev/ttyACM0"),
                "sil": await gh.control(ws, cmd="run_start", kind="sil"),
            }
            await ws.send(sent)
            await asyncio.sleep(3 * h.poll_s + 0.1)
            status = (await gh.control(ws, cmd="status"))["status"]
            stopped = await gh.control(ws, cmd="replay_stop")
            return answers, status, stopped
    answers, status, stopped = with_replays(tmp_path, body, poll_s=0.05, autoconnect=True)
    assert all(a["ok"] is False for a in answers.values())
    assert "replay is going" in answers["again"]["error"] and "replay is going" in answers["open_udp"]["error"]
    assert "replay is going" in answers["sil"]["error"]
    assert status["device"] is None and status["runs"]["active"] is None
    assert status["autoconnect"]["suspended"] == "a replay is going"
    assert status["replay"]["tx_dropped_bytes"] == len(sent)
    assert stopped["replay"]["state"] == "stopped"
    with pytest.raises(BlockingIOError):
        vehicle.recvfrom(4096)  # nothing reached the vehicle's socket
    vehicle.close()
    run_dirs = list((tmp_path / "runs").iterdir())
    kinds = [r.kind for r in groundlog.read_records(run_dirs[0] / "ground.log")]
    assert groundlog.TX not in kinds  # and the bytes the browser sent are not in the ground log either


def test_stopping_and_a_browser_that_goes_away_end_the_replay(tmp_path):
    log = synthetic(tmp_path, gap_ns=int(5e9))

    async def body(h):
        async with gh.ws_connect(h) as ws:
            assert (await gh.control(ws, cmd="replay_start", log=str(log), mode="paced"))["ok"]
            await asyncio.sleep(0.1)
            stopped = (await gh.control(ws, cmd="replay_stop"))["replay"]
        async with gh.ws_connect(h) as ws:
            assert (await gh.control(ws, cmd="replay_start", log=str(log), mode="paced"))["ok"]
            await asyncio.sleep(0.1)
        await asyncio.sleep(0.2)  # the browser has gone
        return stopped, h.replay.status()
    stopped, after_close = with_replays(tmp_path, body)
    assert stopped["state"] == "stopped" and stopped["records_sent"] < stopped["records_total"] == 3
    assert after_close["state"] == "stopped" and "browser went away" in after_close["reason"]


def test_replay_commands_need_the_gate_and_replay_dir_is_a_helper_option(tmp_path):
    async def body(h):
        for token in ("", "wrong"):
            with pytest.raises(gh.InvalidStatus):
                async with gh.ws_connect(h, token=token):
                    pass
    with_replays(tmp_path, body)
    assert gh.helper_mod.parse_args(["--replay-dir", "a", "--replay-dir", "b"]).replay_dir == [Path("a"), Path("b")]
    assert gh.helper_mod.parse_args([]).replay_dir is None


def test_replay_module_reads_only_received_bytes(tmp_path):
    log = synthetic(tmp_path)
    assert [d for _, d in replay_mod.load_rx(log)] == CHUNKS
    assert replay_mod.summarise(log)["rx_records"] == 3


def test_a_device_open_that_yields_while_a_replay_starts_is_refused_and_leaves_no_device(tmp_path, monkeypatch):
    """The window of autoconnect's open_udp: the check passes, the open yields, a replay starts, the open returns."""
    log = synthetic(tmp_path, gap_ns=int(5e9))
    vehicle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vehicle.bind(("127.0.0.1", 0))
    real_open = gh.helper_mod.UdpDevice.open
    state = {}

    async def open_then_replay(port, on_rx, on_close):
        device = await real_open(port, on_rx, on_close)
        state["start"] = await state["helper"].replay.start({"log": str(log), "mode": "paced"})  # the replay lands in the window
        return device
    monkeypatch.setattr(gh.helper_mod.UdpDevice, "open", staticmethod(open_then_replay))

    async def body(h):
        state["helper"] = h
        try:
            await h.open_udp(vehicle.getsockname()[1])
            refused = None
        except gh.helper_mod.DeviceError as e:
            refused = str(e)
        playing = h.replay.status()["state"]
        await h.replay.stop()
        return refused, h.device, playing
    refused, device, playing = with_replays(tmp_path, body)
    vehicle.close()
    assert state["start"]["replay"]["state"] == "playing" and playing == "playing"
    assert refused is not None and "replay is going" in refused
    assert device is None
