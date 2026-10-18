"""The ground-segment helper (tools/gs, ground spec section 3, decision 0053 unit 7): the gate (token, Origin, Host), the byte
relay both ways over a loopback UDP echo and a pty serial pair, the ground log round trip (GS-8), the image library, the
SIL launcher's start, stop, link state and relaunch on a reboot request (ground-segment unit 6), and the validation command and
the report store keyed by (image hash, config hash) (ground-segment unit 9)."""

import argparse
import asyncio
import hashlib
import http.client
import json
import os
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import pytest
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed, InvalidStatus

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "gs"))

import device  # noqa: E402
import groundlog  # noqa: E402
import helper as helper_mod  # noqa: E402
import sil  # noqa: E402

TIMEOUT_S = 10  # labelled harness value: bounds a hung relay only


def run_with_helper(tmp_path, body, *, build_root=None, gcs_dir=None, sil_command=None, sil_udp_port=None, extra_origins=(),
                    validate_command=None):
    """Runs `await body(helper)` with a started helper on a free port; always stops the helper."""
    async def main():
        h = helper_mod.Helper(runs_dir=tmp_path / "runs", build_root=build_root or tmp_path / "build",
                              gcs_dir=gcs_dir or tmp_path / "gcs", sil_udp_port=sil_udp_port, sil_command=sil_command, extra_origins=extra_origins,
                              reports_dir=tmp_path / "reports", validate_command=validate_command)
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


def test_methods_other_than_get_are_refused_with_405(tmp_path):
    async def body(h):
        def request(method, path):
            c = http.client.HTTPConnection("127.0.0.1", h.port, timeout=TIMEOUT_S)
            c.request(method, path, headers={"Origin": h.origin, "Authorization": f"Bearer {h.token}"})
            r = c.getresponse()
            out = (r.status, r.getheader("Allow"))
            c.close()
            return out
        for method in ("POST", "PUT", "DELETE", "PATCH", "HEAD", "OPTIONS"):
            for path in ("/api/status", "/", "/ws", "/api/images"):
                assert await asyncio.to_thread(request, method, path) == (405, "GET"), (method, path)
        assert await asyncio.to_thread(request, "GET", "/api/status") == (200, None)
        # refused before the token is looked at
        def no_token():
            c = http.client.HTTPConnection("127.0.0.1", h.port, timeout=TIMEOUT_S)
            c.request("POST", "/api/status")
            return c.getresponse().status
        assert await asyncio.to_thread(no_token) == 405
    run_with_helper(tmp_path, body)


def test_extra_origin_is_accepted_only_when_given(tmp_path):
    dev = "http://localhost:5173"

    async def body(h):
        status, headers, _ = await http_get(h, "/api/status", origin=dev)
        assert status == 200 and headers["Access-Control-Allow-Origin"] == dev
        assert (await http_get(h, "/api/status", origin=dev, token=None))[0] == 401  # the token is still needed
        assert (await http_get(h, "/api/status", origin="http://localhost:5174"))[0] == 403
        async with ws_connect(h, origin=dev):
            pass
        with pytest.raises(InvalidStatus):
            async with ws_connect(h, origin="http://localhost:5174"):
                pass
        assert "Access-Control-Allow-Origin" not in (await http_get(h, "/api/status", origin=h.origin))[1]

    async def refused(h):
        assert (await http_get(h, "/api/status", origin=dev))[0] == 403
        with pytest.raises(InvalidStatus):
            async with ws_connect(h, origin=dev):
                pass
    run_with_helper(tmp_path / "with", body, extra_origins=(dev,))
    run_with_helper(tmp_path / "without", refused)


def test_extra_origin_argument_must_be_an_origin():
    for good in ("http://localhost:5173", "https://example.org"):
        assert helper_mod.origin_arg(good) == good
    for bad in ("localhost:5173", "http://localhost:5173/", "http://localhost/x", "ftp://h", "http://", "http://u@h"):
        with pytest.raises(argparse.ArgumentTypeError):
            helper_mod.origin_arg(bad)


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


def test_udp_device_sends_an_empty_datagram_on_open_and_a_keepalive_that_a_restarted_peer_receives(monkeypatch):
    """The SIL's link answers the sender of its latest datagram: the device must announce itself at once and keep doing so,
    so that a relaunched SIL (a new socket on the same port) learns it without the GCS sending anything."""
    class Rx(asyncio.DatagramProtocol):
        def __init__(self):
            self.got = []

        def datagram_received(self, data, addr):
            self.got.append((data, addr))

    async def until(predicate):
        deadline = time.monotonic() + TIMEOUT_S
        while not predicate() and time.monotonic() < deadline:
            await asyncio.sleep(0.01)
        return predicate()

    async def main():
        loop = asyncio.get_running_loop()
        first, rx1 = await loop.create_datagram_endpoint(Rx, local_addr=("127.0.0.1", 0))
        port = first.get_extra_info("sockname")[1]
        monkeypatch.setattr(device, "KEEPALIVE_S", TIMEOUT_S * 10)  # no keepalive within the test: only the one sent on open
        dev = await device.UdpDevice.open(port, lambda data: None, lambda reason: None)
        try:
            assert await until(lambda: rx1.got)
            assert rx1.got == [(b"", dev._sock.getsockname())]
        finally:
            dev.close()
        monkeypatch.setattr(device, "KEEPALIVE_S", 0.05)
        dev = await device.UdpDevice.open(port, lambda data: None, lambda reason: None)
        second = None
        try:
            assert await until(lambda: len(rx1.got) >= 3)
            first.close()  # the SIL stops; nothing is bound to the port for a few keepalive periods
            await asyncio.sleep(0.2)
            second, rx2 = await loop.create_datagram_endpoint(Rx, local_addr=("127.0.0.1", port))
            assert await until(lambda: rx2.got)
            assert set(rx2.got) == {(b"", dev._sock.getsockname())}
        finally:
            dev.close()
            if second is not None:
                second.close()
    asyncio.run(main())


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
                await h.open_serial(path, 115200, from_browser=False)  # /dev/pts is not a name the browser may choose
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


def test_second_device_is_refused_and_browser_serial_path_must_be_a_tty_name(tmp_path):
    async def body(h):
        async with ws_connect(h) as ws:
            assert (await control(ws, cmd="open", udp_port=9))["ok"]
            second = await control(ws, cmd="open", udp_port=10)
            assert not second["ok"] and "already open" in second["error"]
            assert (await control(ws, cmd="close"))["ok"]
            elsewhere = tmp_path / "not_a_device"
            elsewhere.write_text("")
            refused = await control(ws, cmd="open", serial=str(elsewhere))
            assert not refused["ok"] and "tty device" in refused["error"]
            for path in ("/dev/null", "/dev/pts/0", "/dev/sda", "/dev/ttyACM0/../null", "/dev//ttyACM0", "ttyACM0", "/dev/serial/by-id/../../null"):
                refused = await control(ws, cmd="open", serial=path)
                assert not refused["ok"] and "tty device" in refused["error"], path
            # a permitted name is attempted (and fails here only because no such device exists)
            missing = await control(ws, cmd="open", serial="/dev/ttyACM99")
            assert not missing["ok"] and "cannot open" in missing["error"]
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


# ---- validation (ground-segment unit 9) ---------------------------------------------------------------------------------

# A stand-in validator: one progress line on stderr, a report at {out}, its id and verdict on stdout (the real one's output
# contract, tools/validate/validate_config.py).
FAKE_VALIDATOR = ("import hashlib, sys; out = sys.argv[sys.argv.index('--out') + 1]; "
                  "print('validate_config: records: PASS', file=sys.stderr, flush=True); "
                  "body = ('{\"verdict\":\"' + sys.argv[sys.argv.index('--verdict') + 1] + '\"}\\n').encode(); "
                  "open(out, 'wb').write(body); print('report: ' + out); "
                  "print('report_id: ' + hashlib.sha256(body).hexdigest()); print('verdict: ' + body.decode()[12:16])")


def config_view():
    sys.path.insert(0, str(ROOT / "tools" / "config"))
    import mcfg
    records = [mcfg.Record("f32", 0x3F800000, 0, "Choice", "scenario", "Manual", False, "a test value"),
               mcfg.Record("i32", 7, 0, "Exact", "scenario", "DefaultFromRegister", True, "a locked test value")]
    cfg = mcfg.Config(0, bytes(mcfg.DIGEST), bytes(mcfg.DIGEST), (mcfg.build_section("quad", 0x1234, records),))
    return mcfg.view(cfg), mcfg.config_hash(cfg).hex()


def test_validation_runs_the_validator_on_the_view_streams_progress_and_keeps_the_report_by_image_and_config(tmp_path):
    files = make_build_root(tmp_path / "build")
    image = hashlib.sha256(files["host-gz-quadfc/sim/quad_fc/libmarv_sil_quad_fc.so"]).hexdigest()
    view, config_hash = config_view()
    command = [sys.executable, "-c", FAKE_VALIDATOR, "--config", "{config}", "--image", "{image}", "--build", "{build}",
               "--work-dir", "{work}", "--out", "{out}", "--verdict", "PASS"]

    async def body(h):
        async with ws_connect(h) as ws:
            r = await control(ws, cmd="config_report", image_hash=image, view=view)
            assert r == {"ok": True, "config_hash": config_hash, "stored": False, "verdict": None}
            assert (await http_get(h, f"/api/reports/{image}/{config_hash}"))[0] == 404
            r = await control(ws, cmd="validate", image_hash="0" * 64, view=view)
            assert not r["ok"] and "not in the image library" in r["error"]
            r = await control(ws, cmd="validate", image_hash=image, view={"sections": "nonsense"})
            assert not r["ok"] and "not a config" in r["error"]
            r = await control(ws, cmd="validate", image_hash=image, view=view)
            assert r == {"ok": True, "config_hash": config_hash, "started": True}
            events = []
            while not events or events[-1]["event"] != "validate_done":
                m = await asyncio.wait_for(ws.recv(), TIMEOUT_S)
                if isinstance(m, str) and "event" in json.loads(m):
                    events.append(json.loads(m))
            assert {"event": "validate_progress", "image_hash": image, "config_hash": config_hash,
                    "line": "validate_config: records: PASS"} in events
            done = events[-1]
            stored = h.reports.report(image, config_hash)
            report = stored.read_bytes()
            assert done["verdict"] == "PASS" and done["report_id"] == hashlib.sha256(report).hexdigest()
            assert (stored.parent / "config.mcfg").read_bytes() == helper_mod.Helper.config_of_view(view)[0]
            status, headers, data = await http_get(h, f"/api/reports/{image}/{config_hash}")
            assert status == 200 and data == report and headers["Content-Type"] == "application/json"
            assert (await http_get(h, f"/api/reports/{image}/{config_hash}", token=None))[0] == 401
            assert (await http_get(h, f"/api/reports/{image}/../{config_hash}"))[0] == 404
            r = await control(ws, cmd="config_report", image_hash=image, view=view)
            assert r == {"ok": True, "config_hash": config_hash, "stored": True, "verdict": "PASS"}
    run_with_helper(tmp_path, body, build_root=tmp_path / "build", validate_command=command)


def test_validation_whose_validator_writes_no_report_is_done_without_a_report_id(tmp_path):
    files = make_build_root(tmp_path / "build")
    image = hashlib.sha256(files["host-gz-quadfc/sim/quad_fc/libmarv_sil_quad_fc.so"]).hexdigest()
    view, config_hash = config_view()
    command = [sys.executable, "-c", "import sys; print('report_id: ' + 'ab' * 32); print('verdict: PASS'); sys.exit(2)"]

    async def body(h):
        async with ws_connect(h) as ws:
            assert (await control(ws, cmd="validate", image_hash=image, view=view))["started"]
            while True:
                m = json.loads(await asyncio.wait_for(ws.recv(), TIMEOUT_S))
                if m.get("event") == "validate_done":
                    break
            assert (m["verdict"], m["report_id"], m["exit_status"]) == (None, None, 2)
            assert h.reports.report(image, config_hash) is None
    run_with_helper(tmp_path, body, build_root=tmp_path / "build", validate_command=command)


# ---- the SIL launcher ---------------------------------------------------------------------------------------------------

def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# A stand-in SIL that prints the port it was given and binds it, as the SIL's link does.
BIND_PORT = (f"import os, socket, time; port = os.environ.get('{sil.UDP_PORT_ENV}'); print(port, flush=True); "
             "s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(('127.0.0.1', int(port))); time.sleep(600)")


def wait_for(predicate):
    deadline = time.monotonic() + TIMEOUT_S
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.02)
    return predicate()


def bounded(fn):
    """fn() in a daemon thread: its result, or a test failure (not a hang) when it has not returned within TIMEOUT_S."""
    out = {}
    worker = threading.Thread(target=lambda: out.setdefault("result", fn()), daemon=True)
    worker.start()
    worker.join(TIMEOUT_S)
    assert "result" in out, f"{fn.__qualname__} did not return within {TIMEOUT_S} s"
    return out["result"]


def test_sil_launcher_starts_and_stops_a_run_and_reports_its_link(tmp_path):
    port = free_udp_port()
    command = [sys.executable, "-c", BIND_PORT]

    async def body(h):
        async with ws_connect(h) as ws:
            started = await control(ws, cmd="sil_start")
            assert started["ok"]
            status = (await control(ws, cmd="status"))["status"]["sil"]
            assert status["running"] and status["udp_port"] == port and status["link"] in ("starting", "ready")
            assert status["relaunches"] == 0 and status["config_dir"] == str(h.run_dir / "store")
            again = await control(ws, cmd="sil_start")
            assert not again["ok"]
            pid = status["pid"]
            assert await asyncio.to_thread(wait_for, lambda: h.sil.link_state() == "ready")
            stopped = await control(ws, cmd="sil_stop")
            assert stopped["ok"] and stopped["exit_status"] == -signal.SIGTERM
            status = (await control(ws, cmd="status"))["status"]["sil"]
            assert not status["running"] and status["link"] == "stopped"
            with pytest.raises(ProcessLookupError):
                os.kill(pid, 0)
    run_with_helper(tmp_path, body, sil_command=command, sil_udp_port=port)
    assert (next((tmp_path / "runs").glob("*/sil/launcher.log"))).read_text().strip() == str(port)


def test_sil_default_command_is_the_interactive_run_on_the_run_directory_store(tmp_path):
    command = sil.default_command(tmp_path / "sil", tmp_path / "store", 4242)
    assert command[1].endswith("tools/sim/run_quad_fc.py") and "--interactive" in command
    assert command[command.index("--config-dir") + 1] == str(tmp_path / "store")
    assert command[command.index("--link-udp-port") + 1] == "4242"
    assert "--link-udp-port" not in sil.default_command(tmp_path / "sil", tmp_path / "store", None)


def test_sil_run_that_exits_with_ex_tempfail_is_relaunched_and_stop_ends_the_relaunching(tmp_path):
    port = free_udp_port()
    marker = tmp_path / "first.done"
    # first launch: exit EX_TEMPFAIL (a reboot request); every later launch: bind the port, and exit EX_TEMPFAIL on SIGTERM
    stand_in = (f"if [ ! -e {marker} ]; then touch {marker}; exit {os.EX_TEMPFAIL}; fi; "
                f"trap 'exit {os.EX_TEMPFAIL}' TERM; {sys.executable} -c \"{BIND_PORT}\" & wait")
    launcher = sil.SilLauncher(tmp_path, port, ["sh", "-c", stand_in])
    launcher.start()
    assert wait_for(lambda: launcher.status()["relaunches"] == 1 and launcher.link_state() == "ready")
    first_pid = launcher.status()["pid"]
    assert launcher.running() and marker.exists()
    assert bounded(launcher.stop) == os.EX_TEMPFAIL  # the stand-in's answer to SIGTERM: a stop is never taken for a reboot
    time.sleep(0.2)
    status = launcher.status()
    assert status["relaunches"] == 1 and not status["running"] and status["pid"] == first_pid and status["link"] == "stopped"
    log = (tmp_path / "sil" / "launcher.log").read_text()
    assert log.count("relaunch 1") == 1 and "relaunch 2" not in log


def test_control_sil_run_that_exits_with_another_status_is_not_relaunched(tmp_path):
    launcher = sil.SilLauncher(tmp_path, None, ["sh", "-c", f"exit {os.EX_TEMPFAIL + 1}"])
    launcher.start()
    assert wait_for(lambda: not launcher.running())
    time.sleep(0.2)
    status = launcher.status()
    assert status["exit_status"] == os.EX_TEMPFAIL + 1 and status["relaunches"] == 0 and status["link"] == "none"
    assert launcher.stop() == os.EX_TEMPFAIL + 1


def test_helper_stop_stops_a_running_sil(tmp_path):
    command = [sys.executable, "-c", "import time; time.sleep(600)"]
    pids = []

    async def body(h):
        h.sil.start()
        pids.append(h.sil.status()["pid"])
    run_with_helper(tmp_path, body, sil_command=command)
    with pytest.raises(ProcessLookupError):
        os.kill(pids[0], 0)


def test_sil_stop_signals_the_group_when_the_leader_has_already_exited(tmp_path):
    """The leader starts a child in its group and exits; stop() must still end the child."""
    command = ["sh", "-c", "sleep 600 & echo $!"]
    launcher = sil.SilLauncher(tmp_path, None, command)
    launcher.start()
    launcher._proc.wait(TIMEOUT_S)
    log = tmp_path / "sil" / "launcher.log"
    child = int(log.read_text().split()[0])
    os.kill(child, 0)  # alive although the leader is gone
    assert launcher.stop() == 0
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            os.kill(child, 0)
        except ProcessLookupError:
            break
        time.sleep(0.02)
    with pytest.raises(ProcessLookupError):
        os.kill(child, 0)


def test_sil_stop_kills_a_run_that_ignores_sigterm(tmp_path, monkeypatch):
    monkeypatch.setattr(sil, "STOP_GRACE_S", 0.3)
    code = "import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); print('up', flush=True); time.sleep(600)"
    launcher = sil.SilLauncher(tmp_path, None, [sys.executable, "-c", code])
    launcher.start()
    log = tmp_path / "sil" / "launcher.log"
    deadline = time.monotonic() + TIMEOUT_S
    while not log.read_text() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert launcher.stop() == -signal.SIGKILL


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


@pytest.mark.parametrize("sig", [signal.SIGTERM, signal.SIGHUP, signal.SIGINT])
def test_command_line_signal_stops_the_sil_run_and_closes_the_log(tmp_path, sig):
    marker = tmp_path / "child.pid"
    # the stand-in SIL: a leader and a child in one process group, both recorded
    stand_in = f"sh -c 'sleep 600 & echo $! > {marker}; wait'"
    proc = subprocess.Popen([sys.executable, str(ROOT / "tools" / "gs" / "helper.py"), "--port", "0", "--runs-dir", str(tmp_path / "runs"),
                             "--build-root", str(tmp_path / "build"), "--gcs-dir", str(tmp_path / "gcs"), "--sil-command", stand_in],
                            stdout=subprocess.PIPE, text=True)
    try:
        m = re.match(r"GCS:\s+http://127\.0\.0\.1:(\d+)/\?token=(\S+)$", proc.stdout.readline())
        assert m
        port, token = m.groups()

        async def start_sil():
            async with connect(f"ws://127.0.0.1:{port}/ws?token={token}", origin=f"http://127.0.0.1:{port}") as ws:
                await ws.send(json.dumps({"cmd": "sil_start"}))
                return json.loads(await asyncio.wait_for(ws.recv(), TIMEOUT_S))
        assert asyncio.run(start_sil())["ok"]
        deadline = time.monotonic() + TIMEOUT_S
        while not (marker.exists() and marker.read_text().strip()) and time.monotonic() < deadline:
            time.sleep(0.02)
        child = int(marker.read_text())
        os.kill(child, 0)
        proc.send_signal(sig)
        assert proc.wait(TIMEOUT_S) == 0
        deadline = time.monotonic() + TIMEOUT_S
        while time.monotonic() < deadline:
            try:
                os.kill(child, 0)
            except ProcessLookupError:
                break
            time.sleep(0.02)
        with pytest.raises(ProcessLookupError):
            os.kill(child, 0)
        log = next((tmp_path / "runs").glob("*/ground.log"))
        events = [r.data.decode() for r in groundlog.read_records(log) if r.kind == groundlog.EVENT]
        assert events[-1] == "helper stopped"
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        proc.stdout.close()
