"""The ground-segment helper (tools/gs, ground spec section 3, decision 0053 unit 7): the gate (token, Origin, Host), the byte
relay both ways over a loopback UDP echo and a pty serial pair, the ground log round trip (GS-8), the image library and the
SIL launcher's start and stop."""

import asyncio
import hashlib
import http.client
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed, InvalidStatus

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "gs"))

import groundlog  # noqa: E402
import helper as helper_mod  # noqa: E402
import sil  # noqa: E402

TIMEOUT_S = 10  # labelled harness value: bounds a hung relay only


def run_with_helper(tmp_path, body, *, build_root=None, gcs_dir=None, sil_command=None, sil_udp_port=None):
    """Runs `await body(helper)` with a started helper on a free port; always stops the helper."""
    async def main():
        h = helper_mod.Helper(runs_dir=tmp_path / "runs", build_root=build_root or tmp_path / "build",
                              gcs_dir=gcs_dir or tmp_path / "gcs", sil_udp_port=sil_udp_port, sil_command=sil_command)
        await h.start(0)
        try:
            return await asyncio.wait_for(body(h), TIMEOUT_S * 3)
        finally:
            await h.stop()
    return asyncio.run(main())


def ws_url(h, token=None):
    return f"ws://127.0.0.1:{h.port}/ws" + (f"?token={h.token if token is None else token}" if token != "" else "")


def ws_connect(h, *, token=None, origin="own"):
    return connect(ws_url(h, token), origin=h.origin if origin == "own" else origin, compression=None)


def http_get(h, path, *, token="own", origin=None, host=None, bearer=None, cookie=None):
    def blocking():
        c = http.client.HTTPConnection("127.0.0.1", h.port, timeout=TIMEOUT_S)
        sep = "&" if "?" in path else "?"
        full = path if token is None else f"{path}{sep}token={h.token if token == 'own' else token}"
        headers = {"Host": host or f"127.0.0.1:{h.port}"}
        if origin:
            headers["Origin"] = origin
        if bearer:
            headers["Authorization"] = f"Bearer {bearer}"
        if cookie:
            headers["Cookie"] = cookie
        c.putrequest("GET", full, skip_host=True)
        for k, v in headers.items():
            c.putheader(k, v)
        c.endheaders()
        r = c.getresponse()
        out = (r.status, dict(r.getheaders()), r.read())
        c.close()
        return out
    return asyncio.to_thread(blocking)


async def recv_bytes(ws, n):
    """Binary frames until n bytes have arrived (control events are skipped)."""
    got = b""
    while len(got) < n:
        m = await asyncio.wait_for(ws.recv(), TIMEOUT_S)
        if isinstance(m, bytes):
            got += m
    return got


async def control(ws, **msg):
    await ws.send(json.dumps(msg))
    while True:
        m = await asyncio.wait_for(ws.recv(), TIMEOUT_S)
        if isinstance(m, str) and "event" not in json.loads(m):
            return json.loads(m)


# ---- the gate ---------------------------------------------------------------------------------------------------------

def test_http_missing_or_wrong_token_is_refused(tmp_path):
    async def body(h):
        assert (await http_get(h, "/api/status", token=None))[0] == 401
        assert (await http_get(h, "/api/status", token="wrong"))[0] == 401
        assert (await http_get(h, "/api/status", token="own", origin=h.origin))[0] == 200
        assert (await http_get(h, "/api/status", token=None, bearer=h.token))[0] == 200
        assert (await http_get(h, "/api/status", token=None, bearer="wrong"))[0] == 401
        assert (await http_get(h, "/api/status", token=None, cookie=f"{helper_mod.COOKIE}={h.token}"))[0] == 200
        assert (await http_get(h, "/api/status", token=None, cookie=f"{helper_mod.COOKIE}=wrong"))[0] == 401
        assert (await http_get(h, "/api/images", token=None))[0] == 401
    run_with_helper(tmp_path, body)


def test_http_foreign_origin_or_host_is_refused(tmp_path):
    async def body(h):
        for origin in ("http://evil.example", "http://127.0.0.1:1", "http://localhost:%d" % h.port, "null"):
            assert (await http_get(h, "/api/status", origin=origin))[0] == 403, origin
        assert (await http_get(h, "/api/status", host="evil.example"))[0] == 403
        assert (await http_get(h, "/api/status", host=f"localhost:{h.port}"))[0] == 403
    run_with_helper(tmp_path, body)


def test_websocket_missing_or_wrong_token_is_refused(tmp_path):
    async def body(h):
        for token in ("", "wrong"):
            with pytest.raises(InvalidStatus) as e:
                async with ws_connect(h, token=token):
                    pass
            assert e.value.response.status_code == 401
        async with ws_connect(h):
            pass
    run_with_helper(tmp_path, body)


def test_websocket_foreign_or_missing_origin_is_refused(tmp_path):
    async def body(h):
        for origin in ("http://evil.example", "http://127.0.0.1:1", None):
            with pytest.raises(InvalidStatus) as e:
                async with ws_connect(h, origin=origin):
                    pass
            assert e.value.response.status_code == 403, origin
    run_with_helper(tmp_path, body)


def test_binds_loopback_only(tmp_path):
    async def body(h):
        assert [s.getsockname()[0] for s in h._server.sockets] == ["127.0.0.1"]
    run_with_helper(tmp_path, body)


def test_one_client_at_a_time(tmp_path):
    async def body(h):
        async with ws_connect(h) as first:
            await control(first, cmd="status")
            async with ws_connect(h) as second:
                with pytest.raises(ConnectionClosed) as e:
                    await asyncio.wait_for(second.recv(), TIMEOUT_S)
                assert e.value.rcvd.code == helper_mod.WS_BUSY
            assert (await control(first, cmd="status"))["status"]["client"] is True
    run_with_helper(tmp_path, body)


# ---- the relay and the ground log -------------------------------------------------------------------------------------

PAYLOAD = bytes((i * 131 + 7) % 256 for i in range(200_000))  # larger than one chunk, not valid in any protocol


def test_udp_relay_both_ways_and_ground_log_round_trip(tmp_path):
    class Echo(asyncio.DatagramProtocol):
        def connection_made(self, transport):
            self.transport = transport
            self.received = bytearray()

        def datagram_received(self, data, addr):
            self.received += data
            self.transport.sendto(data, addr)

    async def body(h):
        loop = asyncio.get_running_loop()
        transport, echo = await loop.create_datagram_endpoint(Echo, local_addr=("127.0.0.1", 0))
        port = transport.get_extra_info("sockname")[1]
        try:
            async with ws_connect(h) as ws:
                assert (await control(ws, cmd="open", udp_port=port))["ok"]
                sent = [PAYLOAD[i:i + 1000] for i in range(0, 20_000, 1000)]
                for chunk in sent:
                    await ws.send(chunk)
                got = await recv_bytes(ws, 20_000)
                assert got == PAYLOAD[:20_000]
                assert bytes(echo.received) == PAYLOAD[:20_000]
                assert (await control(ws, cmd="close"))["ok"]
        finally:
            transport.close()
    run_with_helper(tmp_path, body)
    log = next((tmp_path / "runs").glob("*/ground.log"))
    assert b"".join(groundlog.byte_stream(log, groundlog.TX)) == PAYLOAD[:20_000]
    assert b"".join(groundlog.byte_stream(log, groundlog.RX)) == PAYLOAD[:20_000]
    events = [r.data.decode() for r in groundlog.read_records(log) if r.kind == groundlog.EVENT]
    assert any(e.startswith("device opened: udp 127.0.0.1:") for e in events) and any(e.startswith("device closed") for e in events)


def test_serial_relay_both_ways_over_a_pty_pair_and_ground_log(tmp_path):
    master, slave = os.openpty()
    os.set_blocking(master, False)
    path = os.ttyname(slave)

    async def body(h):
        loop = asyncio.get_running_loop()
        from_ws = bytearray()
        loop.add_reader(master, lambda: from_ws.extend(_read_all(master)))
        try:
            async with ws_connect(h) as ws:
                reply = await control(ws, cmd="open", serial=path, baud=115200)
                assert reply["ok"], reply
                await ws.send(PAYLOAD)
                deadline = time.monotonic() + TIMEOUT_S
                while len(from_ws) < len(PAYLOAD) and time.monotonic() < deadline:
                    await asyncio.sleep(0.01)
                assert bytes(from_ws) == PAYLOAD
                for i in range(0, len(PAYLOAD), 4096):
                    await _write_all(master, PAYLOAD[i:i + 4096])
                assert await recv_bytes(ws, len(PAYLOAD)) == PAYLOAD
        finally:
            loop.remove_reader(master)
    try:
        run_with_helper(tmp_path, body)
    finally:
        os.close(master)
        os.close(slave)
    log = next((tmp_path / "runs").glob("*/ground.log"))
    assert b"".join(groundlog.byte_stream(log, groundlog.TX)) == PAYLOAD
    assert b"".join(groundlog.byte_stream(log, groundlog.RX)) == PAYLOAD


def _read_all(fd):
    out = b""
    while True:
        try:
            data = os.read(fd, 65536)
        except (BlockingIOError, OSError):
            return out
        if not data:
            return out
        out += data


async def _write_all(fd, data):
    while data:
        try:
            n = os.write(fd, data)
            data = data[n:]
        except BlockingIOError:
            await asyncio.sleep(0.001)


def test_second_device_is_refused_and_browser_serial_path_must_be_under_dev(tmp_path):
    async def body(h):
        async with ws_connect(h) as ws:
            assert (await control(ws, cmd="open", udp_port=9))["ok"]
            second = await control(ws, cmd="open", udp_port=10)
            assert not second["ok"] and "already open" in second["error"]
            assert (await control(ws, cmd="close"))["ok"]
            elsewhere = tmp_path / "not_a_device"
            elsewhere.write_text("")
            refused = await control(ws, cmd="open", serial=str(elsewhere))
            assert not refused["ok"] and "/dev" in refused["error"]
    run_with_helper(tmp_path, body)


def test_ground_log_records_round_trip_and_ignore_a_truncated_tail(tmp_path):
    path = tmp_path / "ground.log"
    log = groundlog.GroundLog(path)
    chunks = [(groundlog.RX, b"\x00\xfd\x01"), (groundlog.TX, b"abc"), (groundlog.RX, b""), (groundlog.RX, bytes(range(256)))]
    for kind, data in chunks:
        log.write(kind, data)
    log.event("note")
    log.close()
    records = list(groundlog.read_records(path))
    assert [(r.kind, r.data) for r in records] == chunks + [(groundlog.EVENT, b"note")]
    assert all(a.t_ns <= b.t_ns for a, b in zip(records, records[1:]))
    assert b"".join(groundlog.byte_stream(path, groundlog.RX)) == b"\x00\xfd\x01" + bytes(range(256))
    with open(path, "ab") as f:
        f.write(groundlog.RECORD.pack(groundlog.TX, 0, 100) + b"short")
    assert len(list(groundlog.read_records(path))) == len(records)
    out = subprocess.run([sys.executable, str(ROOT / "tools" / "gs" / "groundlog.py"), str(path), "--tx"], capture_output=True, check=True)
    assert out.stdout == b"abc"


def test_ground_log_rejects_a_foreign_file(tmp_path):
    (tmp_path / "x").write_bytes(b"not a log at all, really")
    with pytest.raises(ValueError):
        list(groundlog.read_records(tmp_path / "x"))


# ---- the image library ------------------------------------------------------------------------------------------------

def make_build_root(root):
    files = {
        "host-gz-quadfc/sim/quad_fc/libmarv_sil_quad_fc.so": b"sil library bytes",
        "host-gz-quadfc/generated/marv_params_fc3v2/marv/params/params_manifest.json": b'{"manifest": 1}',
        "host-gz-quadfc/generated/marv_params_fc3v2/marv/params/params_provenance.json": b'{"provenance": 1}',
        "host-gz-quadfc/generated/marv_params_fc3v2/marv_params_fc3v2.mcfg": b"MCFG factory",
        "m33/fw/hal/target/marv_fc3v2.bin": b"board binary",
        "m33/fw/hal/target/marv_fc3v2.uf2": b"board uf2",
        "m33/fw/hal/target/marv_fc3v2.elf": b"board elf",
        "m33/generated/marv_params_fc3v2/marv/params/params_manifest.json": b'{"manifest": 2}',
        "m33/generated/marv_params_fc3v2/marv_params_fc3v2.mcfg": b"MCFG factory m33",
        "elf-only/fw/hal/target/marv_fc3v2.elf": b"only an elf",
    }
    for rel, data in files.items():
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
    return files


def test_artifact_listing_hash_and_download(tmp_path):
    files = make_build_root(tmp_path / "build")

    async def body(h):
        assert (await http_get(h, "/api/images", token=None))[0] == 401
        status, _, raw = await http_get(h, "/api/images")
        assert status == 200
        lib = json.loads(raw)
        by_name = {e["tree"] + "/" + e["name"]: e for e in lib["images"]}
        assert sorted(by_name) == ["host-gz-quadfc/marv_sil_quad_fc", "m33/marv_fc3v2"]
        assert lib["unkeyed"] == [{"tree": "elf-only", "name": "marv_fc3v2", "reason": lib["unkeyed"][0]["reason"]}]
        sil_entry, board = by_name["host-gz-quadfc/marv_sil_quad_fc"], by_name["m33/marv_fc3v2"]
        assert sil_entry["hash"] == hashlib.sha256(files["host-gz-quadfc/sim/quad_fc/libmarv_sil_quad_fc.so"]).hexdigest()
        assert board["hash"] == hashlib.sha256(files["m33/fw/hal/target/marv_fc3v2.bin"]).hexdigest()  # the .bin, not the UF2 or ELF
        assert sil_entry["params_set"] == board["params_set"] == "marv_params_fc3v2"
        assert set(board["artifacts"]) == {"image", "elf", "uf2", "params_manifest", "factory_mcfg"}
        assert set(sil_entry["artifacts"]) == {"image", "params_manifest", "params_provenance", "factory_mcfg"}
        for entry, tree_files in ((sil_entry, "host-gz-quadfc"), (board, "m33")):
            for name, rel in entry["artifacts"].items():
                status, headers, data = await http_get(h, f"/api/images/{entry['hash']}/{name}")
                assert status == 200 and data == files[rel], (entry["name"], name)
                assert (await http_get(h, f"/api/images/{entry['hash']}/{name}", token=None))[0] == 401
        assert (await http_get(h, f"/api/images/{board['hash']}/params_provenance"))[0] == 404  # absent for this image
        assert (await http_get(h, "/api/images/" + "0" * 64 + "/image"))[0] == 404
        assert (await http_get(h, f"/api/images/{board['hash']}/../../etc/passwd"))[0] == 404
        assert (await http_get(h, f"/api/images/{board['hash']}/not_an_artifact"))[0] == 404
    run_with_helper(tmp_path, body, build_root=tmp_path / "build")


def test_artifact_listing_of_a_missing_build_root_is_empty(tmp_path):
    async def body(h):
        status, _, raw = await http_get(h, "/api/images")
        assert status == 200 and json.loads(raw) == {"images": [], "unkeyed": []}
    run_with_helper(tmp_path, body)


# ---- static serving -----------------------------------------------------------------------------------------------------

def test_static_gcs_serving_token_cookie_and_traversal(tmp_path):
    gcs = tmp_path / "gcs"
    (gcs / "_app").mkdir(parents=True)
    (gcs / "index.html").write_text("<html><head><title>x</title></head><body>gcs</body></html>")
    (gcs / "params.html").write_text("<html><head></head><body>params</body></html>")
    (gcs / "_app" / "app.js").write_text("console.log(1)")
    (tmp_path / "secret.txt").write_text("secret")

    async def body(h):
        assert (await http_get(h, "/", token=None))[0] == 401
        status, headers, page = await http_get(h, "/")
        assert status == 200 and b"gcs" in page
        meta = re.search(rb"<meta name=\"marv-helper\" content='([^']*)'>", page)
        assert json.loads(meta.group(1)) == {"token": h.token, "ws": "/ws"}
        cookie = headers["Set-Cookie"]
        assert "HttpOnly" in cookie and "SameSite=Strict" in cookie
        jar = cookie.split(";")[0]
        status, headers, data = await http_get(h, "/_app/app.js", token=None, cookie=jar)
        assert status == 200 and data == b"console.log(1)" and headers["Content-Type"].startswith(("text/javascript", "application/javascript"))
        assert b"params" in (await http_get(h, "/params", token=None, cookie=jar))[2]
        assert (await http_get(h, "/../secret.txt"))[0] == 404
        assert (await http_get(h, "/%2e%2e/secret.txt"))[0] == 404
        assert (await http_get(h, "/_app/../../secret.txt"))[0] == 404
    run_with_helper(tmp_path, body, gcs_dir=gcs)


def test_unbuilt_gcs_is_404_but_the_api_still_answers(tmp_path):
    async def body(h):
        assert (await http_get(h, "/"))[0] == 404
        status, _, raw = await http_get(h, "/api/status")
        assert status == 200 and json.loads(raw)["gcs_built"] is False
    run_with_helper(tmp_path, body)


# ---- the SIL launcher ---------------------------------------------------------------------------------------------------

def test_sil_launcher_starts_and_stops_a_run_and_reports_no_link(tmp_path):
    command = [sys.executable, "-c", f"import os, time; print(os.environ.get('{sil.UDP_PORT_ENV}'), flush=True); time.sleep(600)"]

    async def body(h):
        async with ws_connect(h) as ws:
            started = await control(ws, cmd="sil_start")
            assert started["ok"]
            status = (await control(ws, cmd="status"))["status"]["sil"]
            assert status["running"] and status["udp_port"] == 4242 and status["link"] == sil.LINK_NOT_AVAILABLE
            again = await control(ws, cmd="sil_start")
            assert not again["ok"]
            pid = status["pid"]
            launcher_log = next((h.run_dir / "sil").glob("launcher.log"))
            deadline = time.monotonic() + TIMEOUT_S
            while not launcher_log.read_text() and time.monotonic() < deadline:
                await asyncio.sleep(0.02)
            stopped = await control(ws, cmd="sil_stop")
            assert stopped["ok"] and stopped["exit_status"] == -signal.SIGTERM
            assert not (await control(ws, cmd="status"))["status"]["sil"]["running"]
            with pytest.raises(ProcessLookupError):
                os.kill(pid, 0)
    run_with_helper(tmp_path, body, sil_command=command, sil_udp_port=4242)
    assert (next((tmp_path / "runs").glob("*/sil/launcher.log"))).read_text().strip() == "4242"


def test_helper_stop_stops_a_running_sil(tmp_path):
    command = [sys.executable, "-c", "import time; time.sleep(600)"]
    pids = []

    async def body(h):
        h.sil.start()
        pids.append(h.sil.status()["pid"])
    run_with_helper(tmp_path, body, sil_command=command)
    with pytest.raises(ProcessLookupError):
        os.kill(pids[0], 0)


# ---- the command line -----------------------------------------------------------------------------------------------------

def test_command_line_start_prints_the_url_and_serves(tmp_path):
    proc = subprocess.Popen([sys.executable, str(ROOT / "tools" / "gs" / "helper.py"), "--port", "0", "--runs-dir", str(tmp_path / "runs"),
                             "--build-root", str(tmp_path / "build"), "--gcs-dir", str(tmp_path / "gcs")],
                            stdout=subprocess.PIPE, text=True)
    try:
        line = proc.stdout.readline()
        m = re.match(r"GCS:\s+http://127\.0\.0\.1:(\d+)/\?token=(\S+)$", line)
        assert m, line
        port, token = m.groups()
        c = http.client.HTTPConnection("127.0.0.1", int(port), timeout=TIMEOUT_S)
        c.request("GET", "/api/status")
        assert c.getresponse().status == 401
        c = http.client.HTTPConnection("127.0.0.1", int(port), timeout=TIMEOUT_S)
        c.request("GET", f"/api/status?token={token}")
        r = c.getresponse()
        assert r.status == 200 and json.loads(r.read())["device"] is None
    finally:
        proc.send_signal(signal.SIGINT)
        assert proc.wait(TIMEOUT_S) == 0
        proc.stdout.close()
