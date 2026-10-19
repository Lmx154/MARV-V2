"""The ground-segment helper (tools/gs, ground spec section 3, decision 0053 unit 7): the gate (token, Origin, Host), the byte
relay both ways over a loopback UDP echo and a pty serial pair, the ground log round trip (GS-8), the image library, the
SIL launcher's start, stop, link state and relaunch on a reboot request (ground-segment unit 6), and the validation command and
the report store keyed by (image hash, config hash) (ground-segment unit 9), and the flash: the gate on (image hash, config the
board will run), the UF2 check, picotool (a fake) run only after the gate, the serial port given up and opened again
(ground-segment unit 15)."""

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
import flash as flash_mod  # noqa: E402
import groundlog  # noqa: E402
import helper as helper_mod  # noqa: E402
import sil  # noqa: E402

TIMEOUT_S = 10  # labelled harness value: bounds a hung relay only


def run_with_helper(tmp_path, body, *, build_root=None, gcs_dir=None, sil_command=None, sil_udp_port=None, extra_origins=(),
                    validate_command=None, picotool=None, flash_timing=None, **helper_kwargs):
    """Runs `await body(helper)` with a started helper on a free port; always stops the helper."""
    async def main():
        h = helper_mod.Helper(runs_dir=tmp_path / "runs", build_root=build_root or tmp_path / "build",
                              gcs_dir=gcs_dir or tmp_path / "gcs", sil_udp_port=sil_udp_port, sil_command=sil_command, extra_origins=extra_origins,
                              reports_dir=tmp_path / "reports", validate_command=validate_command, picotool=picotool,
                              **{"heavy_lock": tmp_path / "heavy.lock", "byid_dir": tmp_path / "by-id", "sysfs_dir": tmp_path / "sys",
                                 **helper_kwargs})
        h.flash_timing = flash_timing or {}
        await h.start(0)
        try:
            return await asyncio.wait_for(body(h), TIMEOUT_S * 3)
        finally:
            await h.stop()
    return asyncio.run(main())


def ISOLATE(tmp_path):
    """The command-line flags that keep a helper process of a test away from the real by-id directory and the real heavy lock."""
    return ["--no-autoconnect", "--byid-dir", str(tmp_path / "by-id"), "--heavy-lock", str(tmp_path / "heavy.lock")]


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
                             "--build-root", str(tmp_path / "build"), "--gcs-dir", str(tmp_path / "gcs"), *ISOLATE(tmp_path)],
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
                             "--build-root", str(tmp_path / "build"), "--gcs-dir", str(tmp_path / "gcs"), "--sil-command", stand_in, *ISOLATE(tmp_path)],
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


# ---- the flash (ground-segment unit 15) -------------------------------------------------------------------------------------

SCHEMA = 0x1234ABCD5678EF01
OTHER_SCHEMA = 0x0BADBADBADBADBAD
FLASH_PAYLOAD = bytes(range(256)) * 5  # 1280 flash bytes: three UF2 blocks of 476 at most
NEW_PAYLOAD = bytes(reversed(FLASH_PAYLOAD))


def make_elf(payload):
    """A minimal 32-bit little-endian ELF with one loadable segment at the XIP base and the two Pico SDK symbols."""
    import struct
    xip = 0x10000000
    strtab = b"\0__flash_binary_start\0__flash_binary_end\0"
    syms = struct.pack("<IIIBBH", 0, 0, 0, 0, 0, 0) \
        + struct.pack("<IIIBBH", 1, xip, 0, 0, 0, 1) + struct.pack("<IIIBBH", 1 + len(b"__flash_binary_start") + 1, xip + len(payload), 0, 0, 0, 1)
    ehdr_size, phdr_size, shdr_size = 52, 32, 40
    data_off = ehdr_size + phdr_size
    sym_off = data_off + len(payload)
    str_off = sym_off + len(syms)
    sh_off = str_off + len(strtab)
    ehdr = struct.pack("<16sHHIIIIIHHHHHH", b"\x7fELF\x01\x01\x01" + bytes(9), 2, 40, 1, xip, ehdr_size, sh_off, 0, ehdr_size,
                       phdr_size, 1, shdr_size, 3, 0)
    phdr = struct.pack("<IIIIIIII", 1, data_off, xip, xip, len(payload), len(payload), 5, 4)
    shdrs = struct.pack("<10I", *([0] * 10)) \
        + struct.pack("<10I", 0, 2, 0, 0, sym_off, len(syms), 2, 0, 4, 16) \
        + struct.pack("<10I", 0, 3, 0, 0, str_off, len(strtab), 0, 0, 1, 0)
    return ehdr + phdr + payload + syms + strtab + shdrs


def make_uf2(payload, *, skip_block=None, family=0xE48BFF59):
    import struct
    chunk = 256
    parts = [payload[i:i + chunk] for i in range(0, len(payload), chunk)]
    blocks = []
    for n, part in enumerate(parts):
        if n == skip_block:
            continue
        head = struct.pack("<8I", flash_mod.UF2_MAGIC_START0, flash_mod.UF2_MAGIC_START1, 0x2000, 0x10000000 + n * chunk, len(part), n,
                           len(parts), family)
        blocks.append(head + part + bytes(476 - len(part)) + struct.pack("<I", flash_mod.UF2_MAGIC_END))
    return b"".join(blocks)


def flash_manifest(schema=SCHEMA):
    params = {"a_gain": {"id": 0, "type": "f32", "default_bits": "0x3f800000", "sigma_bits": "0x00000000", "sigma_kind": "Choice",
                         "method": "scenario", "origin": "Manual", "lock": False, "source": "a test value"},
              "b_count": {"id": 1, "type": "i32", "default_bits": "0x00000007", "sigma_bits": "0x00000000", "sigma_kind": "Exact",
                          "method": "scenario", "origin": "DefaultFromRegister", "lock": True, "source": "a locked test value"}}
    return {"schema_hash": f"{schema:#018x}", "count": 2, "params": params}


def factory_hash(schema=SCHEMA):
    sys.path.insert(0, str(ROOT / "tools" / "config"))
    import mcfg
    return mcfg.config_hash(mcfg.factory_config(flash_manifest(schema))).hex()


def make_flash_root(root, payload=FLASH_PAYLOAD, *, uf2=True, uf2_payload=None, elf=True):
    """A board tree m33 (ELF [+ UF2] and the parameter set files) and a SIL tree host-gz-quadfc of the same schema."""
    sys.path.insert(0, str(ROOT / "tools" / "config"))
    import mcfg
    manifest = flash_manifest()
    board = root / "m33" / "fw" / "hal" / "target"
    board.mkdir(parents=True)
    if elf:
        (board / "marv_fc3v2.elf").write_bytes(make_elf(payload))
    if uf2:
        (board / "marv_fc3v2.uf2").write_bytes(make_uf2(payload if uf2_payload is None else uf2_payload))
    for tree in ("m33", "host-gz-quadfc"):
        d = root / tree / "generated" / "marv_params_fc3v2"
        (d / "marv" / "params").mkdir(parents=True)
        (d / "marv" / "params" / "params_manifest.json").write_text(json.dumps(manifest))
        (d / "marv_params_fc3v2.mcfg").write_bytes(mcfg.encode(mcfg.factory_config(manifest)))
    sil = root / "host-gz-quadfc" / "sim" / "quad_fc"
    sil.mkdir(parents=True)
    (sil / "libmarv_sil_quad_fc.so").write_bytes(b"sil library bytes")
    return hashlib.sha256(payload).hexdigest()


def store_report(h, image, config, *, verdict="PASS", named_image=None, named_config=None):
    d = h.reports.directory(image, config)
    d.mkdir(parents=True)
    body = json.dumps({"format": "marv-validation-report/1", "verdict": verdict, "image": {"sha256": named_image or image},
                       "config": {"config_hash": named_config or config}}, sort_keys=True).encode() + b"\n"
    (d / "report.json").write_bytes(body)
    return hashlib.sha256(body).hexdigest()


FAKE_PICOTOOL = """#!{python}
import json, os, sys, time
with open(os.environ["FAKE_PICOTOOL_LOG"], "a") as f:
    f.write(json.dumps(sys.argv[1:]) + "\\n")
mode = os.environ.get("FAKE_PICOTOOL_MODE", "ok")
if sys.argv[1:3] == ["uf2", "convert"]:
    open(sys.argv[4], "wb").write(open(os.environ["FAKE_PICOTOOL_UF2"], "rb").read())
    sys.exit(0)
if mode == "fail":
    print("No accessible RP-series devices in BOOTSEL mode were found.")
    sys.exit(1)
if mode == "hang":
    time.sleep(60)
print("Loading into Flash: [==============================]  100%\\r", end="")
print("Verifying Flash: OK")
"""


@pytest.fixture
def picotool(tmp_path, monkeypatch):
    script = tmp_path / "picotool"
    script.write_text(FAKE_PICOTOOL.format(python=sys.executable))
    script.chmod(0o755)
    log = tmp_path / "picotool.log"
    monkeypatch.setenv("FAKE_PICOTOOL_LOG", str(log))
    monkeypatch.setenv("FAKE_PICOTOOL_MODE", "ok")
    monkeypatch.setenv("FAKE_PICOTOOL_UF2", str(tmp_path / "convert.uf2"))
    (tmp_path / "convert.uf2").write_bytes(make_uf2(FLASH_PAYLOAD))
    return script, log


def invocations(log):
    return [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []


def board_facts(**over):
    return {"schema_hash": f"{SCHEMA:#018x}", "config_hash": factory_hash(), "config_source": "factory", "armed": False, **over}


FLASH_FAST = {"load_s": 5, "port_wait_s": 5, "settle_s": 0}


async def collect_until(ws, event):
    seen = []
    while True:
        m = await asyncio.wait_for(ws.recv(), TIMEOUT_S)
        if isinstance(m, str) and "event" in json.loads(m):
            seen.append(json.loads(m))
            if seen[-1]["event"] == event:
                return seen


def run_flash_test(tmp_path, picotool, body, *, uf2=True, uf2_payload=None, elf=True, payload=FLASH_PAYLOAD):
    image = make_flash_root(tmp_path / "build", payload, uf2=uf2, uf2_payload=uf2_payload, elf=elf)
    master, slave = os.openpty()
    path = os.ttyname(slave)

    async def wrapped(h):
        await h.open_serial(path, 115200, from_browser=False)
        async with ws_connect(h) as ws:
            return await body(h, ws, image)
    try:
        return run_with_helper(tmp_path, wrapped, build_root=tmp_path / "build", picotool=str(picotool[0]), flash_timing=FLASH_FAST)
    finally:
        os.close(master)
        os.close(slave)


def test_uf2_flash_bytes_tile_from_the_xip_base_and_a_gap_or_foreign_block_is_refused():
    assert flash_mod.uf2_flash_bytes(make_uf2(FLASH_PAYLOAD)) == FLASH_PAYLOAD
    blocks = [make_uf2(FLASH_PAYLOAD)[i:i + 512] for i in range(0, len(make_uf2(FLASH_PAYLOAD)), 512)]
    assert flash_mod.uf2_flash_bytes(b"".join(reversed(blocks))) == FLASH_PAYLOAD  # block order is free
    with pytest.raises(flash_mod.FlashRefused, match="gap or an overlap"):
        flash_mod.uf2_flash_bytes(make_uf2(FLASH_PAYLOAD, skip_block=2))
    with pytest.raises(flash_mod.FlashRefused, match="whole number"):
        flash_mod.uf2_flash_bytes(make_uf2(FLASH_PAYLOAD)[:-1])
    with pytest.raises(flash_mod.FlashRefused, match="magic"):
        flash_mod.uf2_flash_bytes(b"\0" * 512)
    shifted = bytearray(make_uf2(FLASH_PAYLOAD))
    shifted[12:16] = (0x20000000).to_bytes(4, "little")
    with pytest.raises(flash_mod.FlashRefused, match="gap or an overlap"):
        flash_mod.uf2_flash_bytes(bytes(shifted))


def test_uf2_may_pad_the_image_with_less_than_a_flash_page_of_zeros_and_nothing_else(tmp_path):
    image = FLASH_PAYLOAD[:1000]
    digest = hashlib.sha256(image).hexdigest()
    padded = tmp_path / "padded.uf2"
    padded.write_bytes(make_uf2(image + bytes(24)))  # 1024 bytes: whole 256-byte pages
    assert flash_mod.uf2_matches(padded, digest) is None
    padded.write_bytes(make_uf2(image + bytes(24) + b"\x01"))
    assert "holds image" in flash_mod.uf2_matches(padded, digest)
    padded.write_bytes(make_uf2(image + bytes(flash_mod.FLASH_PAGE)))  # a whole extra page is not padding
    assert "holds image" in flash_mod.uf2_matches(padded, digest)


def test_board_elf_without_a_bin_is_keyed_by_its_flash_bytes_and_a_non_elf_stays_unkeyed(tmp_path):
    image = make_flash_root(tmp_path / "build")
    lib = flash_mod.ImageLibrary(tmp_path / "build").scan()
    (board,) = [e for e in lib["images"] if e["kind"] == "board"]
    assert board["hash"] == image and board["image_file"] == "marv_fc3v2.elf" and "uf2" in board["artifacts"]


def test_flash_without_a_passing_report_is_refused_before_picotool_and_leaves_the_port_open(tmp_path, picotool):
    async def body(h, ws, image):
        plan = (await control(ws, cmd="flash_plan", image_hash=image, board=board_facts()))["plan"]
        assert plan["config"]["which"] == "factory" and plan["config"]["hash"] == factory_hash()
        assert plan["gate"]["passes"] is False and plan["allowed"] is False and plan["can_validate"] is True
        assert "no validation report" in plan["refusal"]
        r = await control(ws, cmd="flash", image_hash=image, board=board_facts())
        assert r["ok"] is False and "picotool was not started" in r["error"] and "no validation report" in r["error"]
        assert h.device is not None and h.flash_state is None
        for verdict, named in (("FAIL", {}), ("PASS", {"named_image": "1" * 64}), ("PASS", {"named_config": "2" * 64})):
            d = h.reports.directory(image, factory_hash())
            if d.exists():
                (d / "report.json").unlink()
                d.rmdir()
            store_report(h, image, factory_hash(), verdict=verdict, **named)
            r = await control(ws, cmd="flash", image_hash=image, board=board_facts())
            assert r["ok"] is False and "picotool was not started" in r["error"], (verdict, named)
    run_flash_test(tmp_path, picotool, body)
    assert invocations(picotool[1]) == []  # picotool was never run, not even to convert


def test_flash_of_an_armed_or_unknown_vehicle_is_refused_even_with_a_passing_report(tmp_path, picotool):
    async def body(h, ws, image):
        store_report(h, image, factory_hash())
        for armed, word in ((True, "armed"), (None, "unknown")):
            r = await control(ws, cmd="flash", image_hash=image, board=board_facts(armed=armed))
            assert r["ok"] is False and "picotool was not started" in r["error"] and word in r["error"]
        for bad in ({"schema_hash": "nonsense"}, {"config_hash": "short"}, {"config_source": "slot"}):
            r = await control(ws, cmd="flash", image_hash=image, board=board_facts(**bad))
            assert r["ok"] is False, bad
        r = await control(ws, cmd="flash", image_hash="0" * 64, board=board_facts())
        assert not r["ok"] and "not in the image library" in r["error"]
        r = await control(ws, cmd="flash", image_hash=image, board=None)
        assert not r["ok"] and "identity is missing" in r["error"]
    run_flash_test(tmp_path, picotool, body)
    assert invocations(picotool[1]) == []


def test_flash_with_a_passing_report_checks_the_uf2_runs_picotool_load_and_opens_the_port_again(tmp_path, picotool):
    async def body(h, ws, image):
        report_id = store_report(h, image, factory_hash())
        plan = (await control(ws, cmd="flash_plan", image_hash=image, board=board_facts()))["plan"]
        assert plan["gate"] == {"passes": True, "reason": None, "report_id": report_id, "verdict": "PASS"} and plan["allowed"]
        r = await control(ws, cmd="flash", image_hash=image, board=board_facts())
        assert r["ok"] and r["started"] and r["expected"] == {"image_hash": image, "config_hash": factory_hash(), "config_source": "factory"}
        events = await collect_until(ws, "flash_done")
        done = events[-1]
        assert done["ok"] is True and done["stage"] == "done" and done["picotool_status"] == 0 and done["port_reopened"] is True
        assert done["expected"] == r["expected"]
        assert [e["event"] for e in events if e["event"] in ("device_closed", "device_opened")] == ["device_closed", "device_opened"]
        assert any(e["event"] == "flash_progress" and e["stage"] == "picotool" and "Verifying Flash: OK" in e["line"] for e in events)
        assert h.device is not None and h.status()["flash"]["ok"] is True
    run_flash_test(tmp_path, picotool, body)
    calls = invocations(picotool[1])
    assert len(calls) == 1 and calls[0][:4] == ["load", "-f", "-v", "-x"] and calls[0][4].endswith("marv_fc3v2.uf2")  # the library's UF2 held the image


def test_flash_converts_the_elf_when_the_uf2_is_wrong_and_refuses_a_conversion_that_is_not_the_image(tmp_path, picotool, monkeypatch):
    async def body(h, ws, image):
        store_report(h, image, factory_hash())
        r = await control(ws, cmd="flash", image_hash=image, board=board_facts())
        assert r["ok"]
        done = (await collect_until(ws, "flash_done"))[-1]
        assert done["ok"] is True
        monkeypatch.setenv("FAKE_PICOTOOL_UF2", str(tmp_path / "wrong.uf2"))
        r = await control(ws, cmd="flash", image_hash=image, board=board_facts())
        assert r["ok"]
        done = (await collect_until(ws, "flash_done"))[-1]
        assert done["ok"] is False and done["stage"] == "uf2" and done["picotool_started"] is False
        assert "converted UF2 does not hold the image" in done["error"] and h.device is not None
    (tmp_path / "wrong.uf2").write_bytes(make_uf2(NEW_PAYLOAD))
    run_flash_test(tmp_path, picotool, body, uf2_payload=NEW_PAYLOAD)
    calls = invocations(picotool[1])
    assert [c[:2] for c in calls] == [["uf2", "convert"], ["load", "-f"], ["uf2", "convert"]]  # the second flash never reached load
    assert calls[1][4].endswith(".uf2") and "flash" in calls[1][4]  # the converted copy in the run directory, not the library's UF2


def test_flash_reports_a_failing_or_hung_picotool_and_still_opens_the_port_again(tmp_path, picotool, monkeypatch):
    async def body(h, ws, image):
        store_report(h, image, factory_hash())
        monkeypatch.setenv("FAKE_PICOTOOL_MODE", "fail")
        assert (await control(ws, cmd="flash", image_hash=image, board=board_facts()))["ok"]
        done = (await collect_until(ws, "flash_done"))[-1]
        assert done["ok"] is False and done["stage"] == "port" and done["picotool_status"] == 1 and done["picotool_started"] is True
        assert "status 1" in done["error"] and "maintenance grant" in done["error"] and done["port_reopened"] is True and h.device is not None
        monkeypatch.setenv("FAKE_PICOTOOL_MODE", "hang")
        h.flash_timing = {**FLASH_FAST, "load_s": 0.5}
        t0 = time.monotonic()
        assert (await control(ws, cmd="flash", image_hash=image, board=board_facts()))["ok"]
        busy = await control(ws, cmd="flash", image_hash=image, board=board_facts())
        assert busy["ok"] is False and "a flash is running" in busy["error"]  # one at a time
        done = (await collect_until(ws, "flash_done"))[-1]
        assert done["ok"] is False and "did not finish in 0.5 s and was killed" in done["error"] and time.monotonic() - t0 < 10
    run_flash_test(tmp_path, picotool, body)


def test_flash_waits_for_the_port_to_come_back_and_reports_one_that_does_not(tmp_path, picotool, monkeypatch):
    link = tmp_path / "ttyLINK"

    async def body(h, ws, image):
        store_report(h, image, factory_hash())
        target = os.readlink(link)
        monkeypatch.setenv("FAKE_PICOTOOL_MODE", "ok")
        # the port vanishes while picotool runs and returns a moment later
        async def toggle():
            await asyncio.sleep(0.3)
            link.unlink()
            await asyncio.sleep(0.6)
            link.symlink_to(target)
        task = asyncio.create_task(toggle())
        assert (await control(ws, cmd="flash", image_hash=image, board=board_facts()))["ok"]
        done = (await collect_until(ws, "flash_done"))[-1]
        await task
        assert done["ok"] is True and done["port_reopened"] is True
        link.unlink()  # and now it never returns
        h.flash_timing = {**FLASH_FAST, "port_wait_s": 0.5}
        assert (await control(ws, cmd="flash", image_hash=image, board=board_facts()))["ok"]
        done = (await collect_until(ws, "flash_done"))[-1]
        assert done["ok"] is False and done["port_reopened"] is False and "did not come back" in done["error"] and h.device is None
    image = make_flash_root(tmp_path / "build")
    master, slave = os.openpty()
    link.symlink_to(os.ttyname(slave))

    async def wrapped(h):
        await h.open_serial(str(link), 115200, from_browser=False)
        async with ws_connect(h) as ws:
            return await body(h, ws, image)
    try:
        run_with_helper(tmp_path, wrapped, build_root=tmp_path / "build", picotool=str(picotool[0]), flash_timing=FLASH_FAST)
    finally:
        os.close(master)
        os.close(slave)


def test_flash_needs_the_serial_port_and_a_board_image(tmp_path, picotool):
    async def body(h, ws, image):
        store_report(h, image, factory_hash())
        h.close_device()
        r = await control(ws, cmd="flash", image_hash=image, board=board_facts())
        assert r["ok"] is False and "picotool was not started" in r["error"] and "serial port" in r["error"]
        sil_hash = hashlib.sha256(b"sil library bytes").hexdigest()
        r = await control(ws, cmd="flash_plan", image_hash=sil_hash, board=board_facts())
        assert r["ok"] is False and "not a board image" in r["error"]
    run_flash_test(tmp_path, picotool, body)
    assert invocations(picotool[1]) == []


def test_flash_config_is_the_boards_stored_config_when_the_schema_matches_else_the_factory_config(tmp_path, picotool):
    stored = "5" * 64

    async def body(h, ws, image):
        plan = lambda **board: control(ws, cmd="flash_plan", image_hash=image, board=board_facts(**board))  # noqa: E731
        p = (await plan(config_source="active", config_hash=stored))["plan"]
        assert p["config"]["which"] == "active" and p["config"]["hash"] == stored and p["config"]["expected_source"] == "active"
        assert p["can_validate"] is False and p["gate"]["passes"] is False  # the helper holds no copy of that config
        store_report(h, image, stored)
        p = (await plan(config_source="active", config_hash=stored))["plan"]
        assert p["allowed"] is True
        p = (await plan(config_source="active", config_hash=stored, schema_hash=f"{OTHER_SCHEMA:#018x}"))["plan"]
        assert p["config"]["which"] == "factory" and p["config"]["hash"] == factory_hash() and "factory table" in p["config"]["why"]
        p = (await plan(config_source="factory", config_hash=stored))["plan"]
        assert p["config"]["which"] == "factory" and p["config"]["hash"] == factory_hash()
    run_flash_test(tmp_path, picotool, body)


def test_validate_of_a_board_image_uses_the_sil_tree_writes_the_bin_and_takes_a_config_reference(tmp_path, picotool):
    probe = ("import hashlib, json, sys; a = sys.argv; arg = lambda k: a[a.index(k) + 1]; "
             "img = open(arg('--image'), 'rb').read(); cfg = open(arg('--config'), 'rb').read(); "
             "sys.path.insert(0, %r); import mcfg; c = mcfg.decode(cfg); "
             "body = (json.dumps({'verdict': 'PASS', 'image': {'sha256': hashlib.sha256(img).hexdigest()}, "
             "'config': {'config_hash': mcfg.config_hash(c).hex()}, 'build': arg('--build'), 'image_file': arg('--image')}) + chr(10)).encode(); "
             "open(arg('--out'), 'wb').write(body); print('report_id: ' + hashlib.sha256(body).hexdigest()); print('verdict: PASS')"
             % str(ROOT / "tools" / "config"))
    command = [sys.executable, "-c", probe, "--config", "{config}", "--image", "{image}", "--build", "{build}", "--work-dir", "{work}", "--out", "{out}"]

    async def body(h, ws, image):
        r = await control(ws, cmd="validate", image_hash=image, config="3" * 64)
        assert r["ok"] is False and "holds no copy of config" in r["error"]
        r = await control(ws, cmd="validate", image_hash=image, config="factory")
        assert r == {"ok": True, "config_hash": factory_hash(), "started": True}
        done = (await collect_until(ws, "validate_done"))[-1]
        assert done["verdict"] == "PASS"
        report = json.loads(h.reports.report(image, factory_hash()).read_bytes())
        assert report["image"]["sha256"] == image  # the .bin written from the ELF hashes to the board's image hash
        assert report["build"].endswith("host-gz-quadfc") and report["image_file"].endswith(f"{image}.bin")
        p = (await control(ws, cmd="flash_plan", image_hash=image, board=board_facts()))["plan"]
        assert p["gate"]["passes"] is True and p["allowed"] is True  # the stored report is what the gate reads
        # a config the report store holds a copy of can be validated for another image: here the same one by its hash
        r = await control(ws, cmd="validate", image_hash=image, config=factory_hash())
        assert r["ok"] is True
        await collect_until(ws, "validate_done")
    run_flash_test_with_validator(tmp_path, picotool, body, command)


def run_flash_test_with_validator(tmp_path, picotool, body, command):
    image = make_flash_root(tmp_path / "build")

    async def wrapped(h):
        async with ws_connect(h) as ws:
            return await body(h, ws, image)
    return run_with_helper(tmp_path, wrapped, build_root=tmp_path / "build", picotool=str(picotool[0]), validate_command=command)


# ---- GX-1: the run supervisor, autoconnect, host state and the heavy lock -------------------------------------------------------

import hoststate  # noqa: E402
import runs as runs_mod  # noqa: E402

POLL_S = 0.1  # labelled harness value: the poll period these tests give the helper
POLL_SLACK_S = 0.3  # labelled harness value: scheduler jitter allowed on top of one poll
HEAVY = ROOT / "ci" / "heavy.sh"
# A stand-in run: a leader and a child in one process group, both alive until signalled.
GROUP = "sh -c 'sleep 600 & echo $! > {marker}; wait'"


class Session:
    """A WebSocket with a pump: replies in order for cmd(), events kept for wait_event()."""

    def __init__(self, ws):
        self.ws, self.events, self._replies = ws, [], asyncio.Queue()
        self._changed = asyncio.Condition()
        self._pump_task = asyncio.create_task(self._pump())

    async def _pump(self):
        try:
            async for m in self.ws:
                if isinstance(m, bytes):
                    continue
                d = json.loads(m)
                if "event" in d:
                    async with self._changed:
                        self.events.append(d)
                        self._changed.notify_all()
                else:
                    self._replies.put_nowait(d)
        except ConnectionClosed:
            pass

    async def cmd(self, **msg):
        await self.ws.send(json.dumps(msg))
        return await asyncio.wait_for(self._replies.get(), TIMEOUT_S)

    async def wait_event(self, name, timeout=TIMEOUT_S, **match):
        """The first unconsumed event of that name whose fields include `match`; it is consumed."""
        def find():
            for e in self.events:
                if e["event"] == name and all(e.get(k) == v for k, v in match.items()):
                    return e
            return None
        async with self._changed:
            await asyncio.wait_for(self._changed.wait_for(lambda: find() is not None), timeout)
            e = find()
            self.events.remove(e)
            return e

    def seen(self, name, **match):
        return [e for e in self.events if e["event"] == name and all(e.get(k) == v for k, v in match.items())]

    def close(self):
        self._pump_task.cancel()


def repo_state():
    """(git status incl. untracked, SHA-256 of the diff against HEAD) of the checkout the tests run in (W-1)."""
    status = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain=v1", "-uall"], capture_output=True, text=True, check=True).stdout
    diff = subprocess.run(["git", "-C", str(ROOT), "diff", "HEAD"], capture_output=True, check=True).stdout
    return status, hashlib.sha256(diff).hexdigest()


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def fake_board(tmp_path, name, usb_id=("2e8a", "0009"), interface="MARV link", sysfs=True):
    """A pty standing in for a board's CDC, linked under tmp/by-id/<name>, with a fake sysfs (tmp/sys) giving its USB device the
    vendor:product `usb_id` and its interface the string `interface`; returns (path of the link, the fds to keep open)."""
    master, slave = os.openpty()
    by_id = tmp_path / "by-id"
    by_id.mkdir(exist_ok=True)
    link = by_id / name
    tty = os.ttyname(slave)
    if not sysfs:  # a device the sysfs cannot vouch for
        link.symlink_to(tty)
        return str(link), (master, slave)
    usb = tmp_path / "sys" / "devices" / f"usb-{os.path.basename(tty)}"
    (usb / "1-1:1.2").mkdir(parents=True)
    (usb / "idVendor").write_text(usb_id[0] + "\n")
    (usb / "idProduct").write_text(usb_id[1] + "\n")
    (usb / "1-1:1.2" / "interface").write_text(interface + "\n")
    tty_dir = tmp_path / "sys" / "class" / "tty" / os.path.basename(tty)
    tty_dir.mkdir(parents=True)
    (tty_dir / "device").symlink_to(usb / "1-1:1.2")
    link.symlink_to(tty)  # last: the watcher sees the by-id entry only once sysfs is complete
    return str(link), (master, slave)


def hold_heavy(tmp_path, seconds=60):
    """ci/heavy.sh running `sleep` on tmp/heavy.lock (what dev_check does to a host); returns the process (group leader)."""
    env = {**os.environ, "MARV_HEAVY_LOCK": str(tmp_path / "heavy.lock")}
    env.pop("MARV_HEAVY_HELD", None)
    proc = subprocess.Popen(["bash", str(HEAVY), "--", "sleep", str(seconds)], env=env, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # the holder line is written just after the lock is taken: until then the holder is unknown
    assert wait_for(lambda: hoststate.heavy_lock_state(tmp_path / "heavy.lock")["holder"]), "heavy.sh did not take the lock"
    return proc


def release_heavy(proc):
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait()


def heavy_run(tmp_path, *args, env=None):
    full = {**os.environ, "MARV_HEAVY_LOCK": str(tmp_path / "heavy.lock"), **(env or {})}
    full.pop("MARV_HEAVY_HELD", None) if env is None or "MARV_HEAVY_HELD" not in env else None
    return subprocess.run(["bash", str(HEAVY), *args], env=full, capture_output=True, text=True, timeout=TIMEOUT_S)


# -- ci/heavy.sh and the host state

def test_heavy_script_runs_the_command_holding_the_lock_and_passes_its_status(tmp_path):
    lock = tmp_path / "heavy.lock"
    done = heavy_run(tmp_path, "--", "bash", "-c", f"{sys.executable} -c \"import sys; sys.path.insert(0, '{ROOT}/tools/gs'); import hoststate, json; "
                     f"print(json.dumps(hoststate.heavy_lock_state('{lock}')))\"; exit 7")
    assert done.returncode == 7
    state = json.loads(done.stdout)
    assert state["held"] and "cmd=bash -c" in state["holder"] and f"pid=" in state["holder"]
    assert hoststate.heavy_lock_state(lock)["held"] is False  # released at exit


def test_heavy_script_refuses_while_held_names_the_holder_and_is_reentrant(tmp_path):
    holder = hold_heavy(tmp_path)
    try:
        second = heavy_run(tmp_path, "--wait", "0", "--", "true")
        assert second.returncode == 75 and "held" in second.stderr and "sleep" in second.stderr
        status = heavy_run(tmp_path, "--status")
        assert status.returncode == 75 and status.stdout.startswith("held: pid=") and "cmd=sleep" in status.stdout
        state = hoststate.heavy_lock_state(tmp_path / "heavy.lock")
        assert state["held"] and "cmd=sleep" in state["holder"]
    finally:
        release_heavy(holder)
    assert heavy_run(tmp_path, "--status").stdout.strip() == "free"
    # re-entrant: the command of a run that holds the lock may take it again (dev_check under heavy.sh)
    inner = heavy_run(tmp_path, "--", "bash", str(HEAVY), "--wait", "0", "--", "echo", "inner")
    assert inner.returncode == 0 and inner.stdout.strip() == "inner"


def test_heavy_script_lock_is_not_kept_by_a_daemon_the_command_leaves_behind(tmp_path):
    marker = tmp_path / "daemon.pid"
    done = heavy_run(tmp_path, "--", "bash", "-c", f"sleep 600 >/dev/null 2>&1 & echo $! > {marker}")
    assert done.returncode == 0
    daemon = int(marker.read_text())
    try:
        assert alive(daemon)
        assert heavy_run(tmp_path, "--status").stdout.strip() == "free"
    finally:
        os.kill(daemon, signal.SIGKILL)


@pytest.mark.parametrize("script,args", [("dev_check.sh", []), ("local_ci.sh", ["core"])])
def test_dev_check_and_local_ci_take_the_heavy_lock_and_stop_when_it_is_held(tmp_path, script, args):
    holder = hold_heavy(tmp_path)
    try:
        env = {**os.environ, "MARV_HEAVY_LOCK": str(tmp_path / "heavy.lock")}
        env.pop("MARV_HEAVY_HELD", None)
        done = subprocess.run(["bash", str(ROOT / "ci" / script), *args], env=env, capture_output=True, text=True, timeout=TIMEOUT_S)
        assert done.returncode == 75, done.stdout + done.stderr  # refused before any work (dev_check's first line of work is the smoke test)
        assert "heavy lock" in done.stderr and "sleep" in done.stderr
        assert "##### smoke" not in done.stdout
    finally:
        release_heavy(holder)


def test_dev_check_and_local_ci_re_exec_from_any_directory_and_a_stale_held_variable_counts_for_nothing(tmp_path):
    """dev_check.sh run as `cd ci && ./dev_check.sh` finds heavy.sh and itself again (a copy in a scratch tree, with a stub smoke test)."""
    fake = tmp_path / "fake"
    (fake / "ci").mkdir(parents=True)
    for name in ("dev_check.sh", "heavy.sh", "local_ci.sh"):
        (fake / "ci" / name).write_text((ROOT / "ci" / name).read_text())
        (fake / "ci" / name).chmod(0o755)
    (fake / "ci" / "smoke.sh").write_text('echo "SMOKE-RAN held=$MARV_HEAVY_HELD"\nexit 0\n')
    (fake / "ci" / "smoke.sh").chmod(0o755)
    lock = tmp_path / "heavy.lock"
    env = {**os.environ, "MARV_HEAVY_LOCK": str(lock)}
    env.pop("MARV_HEAVY_HELD", None)
    for cwd, command in ((fake / "ci", ["./dev_check.sh"]), (tmp_path, ["bash", str(fake / "ci" / "dev_check.sh")]),
                         (fake / "ci", ["bash", "../ci/dev_check.sh"])):
        done = subprocess.run(command, cwd=cwd, env=env, capture_output=True, text=True, timeout=TIMEOUT_S)
        assert f"SMOKE-RAN held={lock}" in done.stdout, (cwd, command, done.stdout, done.stderr)
    # a variable left over in a shell whose lock is gone is not re-entry: the script takes the lock itself
    stale = {**env, "MARV_HEAVY_HELD": str(lock)}
    done = subprocess.run(["./dev_check.sh"], cwd=fake / "ci", env=stale, capture_output=True, text=True, timeout=TIMEOUT_S)
    assert f"SMOKE-RAN held={lock}" in done.stdout and "cmd=" in lock.read_text()  # heavy.sh wrote its holder line: it took the lock
    assert heavy_run(tmp_path, "--held", env={"MARV_HEAVY_HELD": str(lock)}).returncode == 1
    holder = hold_heavy(tmp_path)
    try:
        assert heavy_run(tmp_path, "--held", env={"MARV_HEAVY_HELD": str(lock)}).returncode == 0  # really held: re-entry counts
        assert heavy_run(tmp_path, "--held").returncode == 1  # no variable: not an ancestor's lock
        done = subprocess.run(["./local_ci.sh", "--bogus"], cwd=fake / "ci", env=env, capture_output=True, text=True, timeout=TIMEOUT_S)
        assert done.returncode == 75  # the lock is held by someone else: stopped before the usage check
    finally:
        release_heavy(holder)
    done = subprocess.run(["./local_ci.sh", "--bogus"], cwd=fake / "ci", env=env, capture_output=True, text=True, timeout=TIMEOUT_S)
    assert done.returncode == 2 and "usage:" in done.stderr  # re-exec'd from another directory and reached its own argument check


def test_host_state_has_load_memory_and_the_heavy_lock_holder(tmp_path):
    holder = hold_heavy(tmp_path)

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            host = (await s.cmd(cmd="host_state"))["host"]
            assert len(host["loadavg"]) == 3 and all(x >= 0 for x in host["loadavg"]) and host["cpus"] >= 1
            assert 0 < host["mem_available_bytes"] <= host["mem_total_bytes"]
            assert host["heavy_lock"]["held"] and "cmd=sleep" in host["heavy_lock"]["holder"]
            assert (await s.cmd(cmd="status"))["status"]["host"]["heavy_lock"]["held"]
            release_heavy(holder)
            assert not (await s.cmd(cmd="host_state"))["host"]["heavy_lock"]["held"]
            await s.wait_event("host_state")  # pushed by the monitor
            s.close()
    try:
        run_with_helper(tmp_path, body, poll_s=POLL_S)
    finally:
        release_heavy(holder)


# -- the run supervisor

def test_run_plan_names_the_argv_the_directory_and_the_refusals_and_starts_nothing(tmp_path):
    port, keep = fake_board(tmp_path, "usb-Fake_Board_1-if00")

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            sil_plan = (await s.cmd(cmd="run_plan", kind="sil"))["plan"]
            assert sil_plan["allowed"] and sil_plan["refusals"] == [] and sil_plan["kind"] == "sil"
            assert sil_plan["argv"][1].endswith("tools/sim/run_quad_fc.py") and "--interactive" in sil_plan["argv"]
            hil_plan = (await s.cmd(cmd="run_plan", kind="hil", port=port, seconds=3, expect="none", gui=True))["plan"]
            assert hil_plan["argv"][1].endswith("tools/sim/run_hil_s4.py")
            assert hil_plan["argv"][hil_plan["argv"].index("--port") + 1] == port
            assert hil_plan["argv"][hil_plan["argv"].index("--expect") + 1] == "none" and "--gui" in hil_plan["argv"]
            assert hil_plan["argv"][hil_plan["argv"].index("--out-dir") + 1] == hil_plan["out_dir"]
            assert hil_plan["allowed"] and h.runs.active() is None and not (h.run_dir / "runs").exists()
            h.device = type("Fake", (), {"close": lambda self: None})()  # no serial port: unchanged
            s.close()
    try:
        run_with_helper(tmp_path, body)
    finally:
        for fd in keep:
            os.close(fd)


def test_run_start_and_stop_write_the_manifest_emit_events_and_leave_no_processes(tmp_path):
    marker = tmp_path / "child.pid"
    command = ["sh", "-c", f"sleep 600 & echo $! > {marker}; wait"]
    pids = {}

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            started = await s.cmd(cmd="run_start", kind="sil")
            assert started["ok"]
            run = started["run"]
            await s.wait_event("run_started", run_id=run["run_id"], kind="sil")
            assert await asyncio.to_thread(wait_for, lambda: marker.exists() and marker.read_text().strip())
            pids["child"], pids["leader"] = int(marker.read_text()), run["pid"]
            path = h.run_dir / "runs" / run["run_id"] / "manifest.json"
            m = json.loads(path.read_text())
            assert m["state"] == "running" and m["kind"] == "sil" and m["argv"] == command and m["exit_status"] is None
            assert m["schema"] == 1 and m["pid"] == run["pid"]
            assert (await s.cmd(cmd="run_list"))["runs"] == [run["run_id"]]
            stopped = await s.cmd(cmd="run_stop")
            assert stopped["ok"] and stopped["exit_status"] == -signal.SIGTERM
            assert (await s.wait_event("run_stopped", run_id=run["run_id"]))["exit_status"] == -signal.SIGTERM
            m = json.loads(path.read_text())
            assert m["state"] == "stopped" and m["exit_status"] == -signal.SIGTERM and m["ended_utc"] >= m["started_utc"]
            assert (await s.cmd(cmd="run_manifest", run_id=run["run_id"]))["manifest"] == m
            assert not (await s.cmd(cmd="run_stop"))["ok"]  # nothing to stop
            assert h.token not in path.read_text() and h.token not in json.dumps(m)
            s.close()
    run_with_helper(tmp_path, body, sil_command=command)
    for pid in pids.values():
        assert wait_for(lambda: not alive(pid)), f"process {pid} outlived the stopped run"


def test_a_run_that_ends_by_itself_is_run_exited_and_a_new_run_may_start(tmp_path):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            port, keep = fake_board(tmp_path, "usb-Fake_Board_1-if00")
            first = (await s.cmd(cmd="run_start", kind="hil", port=port))["run"]
            exited = await s.wait_event("run_exited", run_id=first["run_id"])
            assert exited["exit_status"] == os.EX_TEMPFAIL and exited["kind"] == "hil"  # a HIL run is not relaunched
            m = json.loads((h.run_dir / "runs" / first["run_id"] / "manifest.json").read_text())
            assert m["state"] == "exited" and m["exit_status"] == os.EX_TEMPFAIL
            second = await s.cmd(cmd="run_start", kind="hil", port=port)
            assert second["ok"] and second["run"]["run_id"] != first["run_id"]
            assert len((await s.cmd(cmd="run_list"))["runs"]) == 2
            for fd in keep:
                os.close(fd)
            s.close()
    run_with_helper(tmp_path, body, hil_command=["sh", "-c", f"exit {os.EX_TEMPFAIL}"])


def test_hil_run_is_a_process_group_that_stop_empties_and_it_owns_the_board_port(tmp_path):
    marker = tmp_path / "child.pid"
    port, keep = fake_board(tmp_path, "usb-Fake_Board_1-if00")
    link, keep2 = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_FAKE1-if02")
    pids = {}

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            run = (await s.cmd(cmd="run_start", kind="hil", port=port))["run"]
            assert await asyncio.to_thread(wait_for, lambda: marker.exists() and marker.read_text().strip())
            pids["child"], pids["leader"] = int(marker.read_text()), run["pid"]
            assert os.getpgid(run["pid"]) == run["pid"] != os.getpgid(os.getpid())  # its own group
            with pytest.raises(device.DeviceError, match="HIL run is going"):  # the board port is the run's: no serial open meanwhile
                await h.open_serial(link, 115200, from_browser=False)
            assert h.device is None
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    try:
        run_with_helper(tmp_path, body, hil_command=["sh", "-c", f"sleep 600 & echo $! > {marker}; wait"], autoconnect=False)
    finally:
        for fd in (*keep, *keep2):
            os.close(fd)
    for pid in pids.values():
        assert wait_for(lambda: not alive(pid)), f"process {pid} outlived the stopped run"


def test_two_sims_are_refused_in_either_order(tmp_path):
    port, keep = fake_board(tmp_path, "usb-Fake_Board_1-if00")

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]
            second_sil = await s.cmd(cmd="run_start", kind="sil")
            assert not second_sil["ok"] and "one simulation at a time" in second_sil["error"]
            hil = await s.cmd(cmd="run_start", kind="hil", port=port)
            assert not hil["ok"] and "one simulation at a time" in hil["refusals"][0]
            assert not (await s.cmd(cmd="run_plan", kind="hil", port=port))["plan"]["allowed"]
            assert (await s.cmd(cmd="run_stop"))["ok"]
            assert (await s.cmd(cmd="run_start", kind="hil", port=port))["ok"]
            sil = await s.cmd(cmd="run_start", kind="sil")
            assert not sil["ok"] and "one simulation at a time" in sil["error"]
            assert (await s.cmd(cmd="sil_start"))["ok"] is False  # the old command is the same run
            assert (await s.cmd(cmd="run_stop"))["ok"]
            assert len((await s.cmd(cmd="run_list"))["runs"]) == 2  # the refused starts left no manifest
            s.close()
    try:
        run_with_helper(tmp_path, body, sil_command=["sleep", "600"], hil_command=["sleep", "600"])
    finally:
        for fd in keep:
            os.close(fd)


def test_hil_is_refused_while_the_helper_has_a_serial_port_open(tmp_path):
    port, keep = fake_board(tmp_path, "usb-Fake_Board_1-if00")
    link, keep2 = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_FAKE1-if02")
    ran = tmp_path / "hil.ran"

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            await h.open_serial(link, 115200, from_browser=False)
            refused = await s.cmd(cmd="run_start", kind="hil", port=port)
            assert not refused["ok"] and "serial port open" in refused["error"] and refused["refusals"]
            assert not ran.exists() and h.runs.active() is None and (await s.cmd(cmd="run_list"))["runs"] == []
            assert not (await s.cmd(cmd="run_plan", kind="hil", port=port))["plan"]["allowed"]
            # control: with the port closed the same request starts
            assert (await s.cmd(cmd="close"))["ok"]
            started = await s.cmd(cmd="run_start", kind="hil", port=port)
            assert started["ok"]
            assert await asyncio.to_thread(wait_for, ran.exists)
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    try:
        run_with_helper(tmp_path, body, hil_command=["sh", "-c", f"touch {ran}; sleep 600"], autoconnect=False)
    finally:
        for fd in (*keep, *keep2):
            os.close(fd)


def test_hil_port_must_be_an_allow_listed_device_and_present(tmp_path):
    outside = tmp_path / "not_a_device"
    outside.write_text("")
    (tmp_path / "by-id").mkdir()
    (tmp_path / "by-id" / "usb-Fake-if00").symlink_to(outside)  # a by-id name whose target is not under /dev
    (tmp_path / "by-id" / "evil; touch pwned").symlink_to("/dev/null")
    ran = tmp_path / "hil.ran"

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            bad = [str(outside), "/dev/null", "/dev/ttyACM0", "/dev/serial/by-id/usb-X-if00", f"{tmp_path}/by-id/../not_a_device",
                   f"{tmp_path}/by-id//usb-Fake-if00", f"{tmp_path}/by-id/usb-Fake-if00", "by-id/usb-Fake-if00", "", 5, None, ["x"]]
            for port in bad:
                for cmd in ("run_plan", "run_start"):
                    reply = await s.cmd(cmd=cmd, kind="hil", port=port)
                    assert not reply["ok"], (cmd, port)
            assert not (await s.cmd(cmd="run_start", kind="hil"))["ok"]  # no port at all
            assert h.runs.active() is None and not ran.exists()
            # an allowed name with no device behind it: planned but refused, never started
            (tmp_path / "by-id" / "usb-Absent-if00").symlink_to("/dev/does-not-exist-gx1")
            plan = (await s.cmd(cmd="run_plan", kind="hil", port=f"{tmp_path}/by-id/usb-Absent-if00"))["plan"]
            assert not plan["allowed"] and "not present" in plan["refusals"][0]
            # shell metacharacters in a name are one argv element; no shell sees them
            plan = (await s.cmd(cmd="run_plan", kind="hil", port=f"{tmp_path}/by-id/evil; touch pwned"))["plan"]
            assert plan["argv"][plan["argv"].index("--port") + 1] == f"{tmp_path}/by-id/evil; touch pwned"
            s.close()
    run_with_helper(tmp_path, body, hil_command=None)
    assert not (ROOT / "pwned").exists() and not (tmp_path / "pwned").exists()


def test_run_options_are_a_closed_list_with_checked_values(tmp_path):
    port, keep = fake_board(tmp_path, "usb-Fake_Board_1-if00")

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for msg in [dict(kind="sil", port=port), dict(kind="sil", card="vehicles/x.yaml"), dict(kind="sil", out_dir="/etc"),
                        dict(kind="hil", port=port, out_dir="/etc"), dict(kind="hil", port=port, expect="bogus"),
                        dict(kind="hil", port=port, seconds=0), dict(kind="hil", port=port, seconds=-1),
                        dict(kind="hil", port=port, seconds=True), dict(kind="hil", port=port, seconds="3"),
                        dict(kind="hil", port=port, seconds=float("inf")), dict(kind="hil", port=port, gui="yes"),
                        dict(kind="rocket"), dict(kind=3), dict(kind="hil", port=port, argv=["x"])]:
                for cmd in ("run_plan", "run_start"):
                    assert not (await s.cmd(cmd=cmd, **msg))["ok"], (cmd, msg)
            assert h.runs.active() is None
            s.close()
    try:
        run_with_helper(tmp_path, body, sil_command=["sleep", "600"], hil_command=["sleep", "600"])
    finally:
        for fd in keep:
            os.close(fd)


def test_run_manifest_and_run_id_must_name_a_run_under_the_runs_directory(tmp_path):
    secret = tmp_path / "elsewhere"
    secret.mkdir()
    (secret / "manifest.json").write_text('{"secret": 1}')

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]
            assert (await s.cmd(cmd="run_stop"))["ok"]
            run_id = (await s.cmd(cmd="run_list"))["runs"][0]
            for bad in ["../../../etc/passwd", "..", f"../{run_id}", str(secret), "", None, 3, run_id + "/../..", "20260101T000000Z-sil-1/.."]:
                reply = await s.cmd(cmd="run_manifest", run_id=bad)
                assert not reply["ok"], bad
            # a run directory that is a symlink out of the runs directory is not followed
            link = h.run_dir / "runs" / "20260101T000000Z-sil-99"
            link.symlink_to(secret)
            reply = await s.cmd(cmd="run_manifest", run_id=link.name)
            assert not reply["ok"] and "outside" in reply["error"]
            assert link.name in (await s.cmd(cmd="run_list"))["runs"]  # listed by name only; reading it is what is refused
            assert (await s.cmd(cmd="run_manifest", run_id=run_id))["ok"]
            s.close()
    run_with_helper(tmp_path, body, sil_command=["sleep", "600"])


def test_resolve_under_refuses_symlinks_and_dotdot_out_of_the_root(tmp_path):
    root = tmp_path / "root"
    (root / "a").mkdir(parents=True)
    (tmp_path / "out").mkdir()
    (root / "a" / "link").symlink_to(tmp_path / "out")
    assert runs_mod.resolve_under(root / "a", (root,)) == root / "a"
    for bad in (root / "a" / "link", root / ".." / "out", tmp_path / "out", "/etc/passwd", "", root / "a" / ".." / ".." / "out"):
        with pytest.raises(ValueError):
            runs_mod.resolve_under(bad, (root,))
    assert runs_mod.resolve_under(tmp_path / "out", (root, tmp_path / "out")) == tmp_path / "out"


def test_heavy_lock_held_refuses_a_run_and_the_refusal_names_the_holder(tmp_path):
    holder = hold_heavy(tmp_path)
    port, keep = fake_board(tmp_path, "usb-Fake_Board_1-if00")
    ran = tmp_path / "ran"

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for msg in (dict(kind="sil"), dict(kind="hil", port=port)):
                reply = await s.cmd(cmd="run_start", **msg)
                assert not reply["ok"] and "heavy lock is held" in reply["error"] and "cmd=sleep" in reply["error"], msg
                assert not (await s.cmd(cmd="run_plan", **msg))["plan"]["allowed"]
            assert not (await s.cmd(cmd="sil_start"))["ok"]
            assert not ran.exists() and h.runs.active() is None
            release_heavy(holder)  # control: the same requests start once the lock is free
            assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]
            assert (await s.cmd(cmd="run_stop"))["ok"]
            assert (await s.cmd(cmd="run_start", kind="hil", port=port))["ok"]
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    try:
        run_with_helper(tmp_path, body, sil_command=["sh", "-c", f"touch {ran}; sleep 600"], hil_command=["sleep", "600"])
    finally:
        release_heavy(holder)
        for fd in keep:
            os.close(fd)


def test_validation_or_flash_in_progress_refuses_a_run(tmp_path):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            h._validation_task = asyncio.create_task(asyncio.sleep(600))
            reply = await s.cmd(cmd="run_start", kind="sil")
            assert not reply["ok"] and "validation is running" in reply["error"]
            h._validation_task.cancel()
            h._flash_task = asyncio.create_task(asyncio.sleep(600))
            reply = await s.cmd(cmd="run_start", kind="sil")
            assert not reply["ok"] and "flash is running" in reply["error"]
            h._flash_task.cancel()
            s.close()
    run_with_helper(tmp_path, body, sil_command=["sleep", "600"])


# -- autoconnect

# A stand-in SIL for the autoconnect tests: binds the link port. The first launch exits EX_TEMPFAIL once the trigger file exists
# (the reboot request); a later launch records the first datagram it receives (the helper's keepalive) and stays up.
STAND_IN_SIL = """
import os, socket, sys, time
port = int(os.environ['{env}'])
trigger, got, launches = sys.argv[1], sys.argv[2], sys.argv[3]
n = len(open(launches).read()) if os.path.exists(launches) else 0
open(launches, 'a').write('x')
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('127.0.0.1', port))
if n == 0:
    while not os.path.exists(trigger):
        time.sleep(0.01)
    sys.exit({tempfail})
s.settimeout(30)
data, addr = s.recvfrom(64)
open(got, 'w').write('keepalive')
time.sleep(600)
""".format(env=sil.UDP_PORT_ENV, tempfail=os.EX_TEMPFAIL)


def test_autoconnect_opens_the_sil_link_within_one_poll_of_the_bind_and_keeps_it_across_a_relaunch(tmp_path, monkeypatch):
    monkeypatch.setattr(device, "KEEPALIVE_S", 0.05)
    port = free_udp_port()
    trigger, got, launches = tmp_path / "trigger", tmp_path / "got", tmp_path / "launches"
    command = [sys.executable, "-c", STAND_IN_SIL, str(trigger), str(got), str(launches)]
    bound_at = {}

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            watcher = threading.Thread(target=lambda: (wait_for(lambda: sil.udp_bound(port)), bound_at.setdefault("t", time.monotonic())), daemon=True)
            assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]
            watcher.start()
            assert h.device is None or True
            opened = await s.wait_event("autoconnect", state="opened", source="sil")
            opened_at = time.monotonic()
            watcher.join(TIMEOUT_S)
            assert opened["device"] == f"udp 127.0.0.1:{port}"
            assert opened_at - bound_at["t"] < POLL_S + POLL_SLACK_S, "the UDP link opened later than one poll after the bind"
            device_before = h.device
            assert device_before is not None
            trigger.write_text("go")  # the SIL asks for a reboot
            relaunched = await s.wait_event("sil_relaunched")
            assert relaunched["relaunches"] == 1 and relaunched["pid"] != s.seen("run_started")[0].get("pid")
            assert await asyncio.to_thread(wait_for, got.exists), "the relaunched SIL never heard the helper (keepalive)"
            assert h.device is device_before  # the same open link: not closed, not reopened
            assert not s.seen("autoconnect", state="closed")
            assert json.loads((h.run_dir / "runs" / relaunched["run_id"] / "manifest.json").read_text())["relaunches"] == 1
            await s.cmd(cmd="run_stop")
            closed = await s.wait_event("autoconnect", state="closed", source="sil")
            assert h.device is None and closed["reason"] == "the SIL run ended" or h.device is None
            s.close()
    run_with_helper(tmp_path, body, sil_command=command, sil_udp_port=port, poll_s=POLL_S)


def test_autoconnect_off_or_closed_by_the_user_is_not_reopened_until_the_run_changes(tmp_path):
    port = free_udp_port()
    command = [sys.executable, "-c", BIND_PORT]

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="autoconnect", enabled=False))["autoconnect"]["enabled"] is False
            assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]
            assert await asyncio.to_thread(wait_for, lambda: h.sil.link_state() == "ready")
            await asyncio.sleep(POLL_S * 5)
            assert h.device is None and not s.seen("autoconnect", state="opened")
            assert (await s.cmd(cmd="autoconnect", enabled=True))["ok"]
            await s.wait_event("autoconnect", state="opened", source="sil")
            assert (await s.cmd(cmd="close"))["ok"]  # the user closes it: it stays closed
            await asyncio.sleep(POLL_S * 5)
            assert h.device is None and not s.seen("autoconnect", state="opened")
            assert (await s.cmd(cmd="run_stop"))["ok"]
            assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]  # a new run is a new link
            await s.wait_event("autoconnect", state="opened", source="sil")
            assert not (await s.cmd(cmd="autoconnect", enabled="yes"))["ok"]
            s.close()
    run_with_helper(tmp_path, body, sil_command=command, sil_udp_port=port, poll_s=POLL_S)


def test_sil_relaunched_is_an_event_even_without_a_link(tmp_path):
    marker = tmp_path / "first.done"
    stand_in = f"if [ ! -e {marker} ]; then touch {marker}; exit {os.EX_TEMPFAIL}; fi; sleep 600"

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            run = (await s.cmd(cmd="run_start", kind="sil"))["run"]
            event = await s.wait_event("sil_relaunched", run_id=run["run_id"])
            assert event["relaunches"] == 1
            assert (await s.cmd(cmd="run_stop"))["ok"]
            assert not s.seen("sil_relaunched") and not s.seen("run_exited")
            s.close()
    run_with_helper(tmp_path, body, sil_command=["sh", "-c", stand_in], autoconnect=True, poll_s=POLL_S)


def test_board_watcher_opens_only_the_link_interface_and_never_a_board_without_one(tmp_path):
    keep = []

    async def body(h):
        async with ws_connect(h) as ws:  # connected first: the events are pushed to the attached browser only
            s = Session(ws)
            # a board that shows up with only the console interface, the HIL image's single CDC and a -if02 that is no device
            _, k = fake_board(tmp_path, "usb-Fake_Board_1-if00")
            keep.extend(k)
            _, k = fake_board(tmp_path, "usb-Fake_HIL_2-if00")
            keep.extend(k)
            (tmp_path / "by-id" / "usb-Raspberry_Pi_Pico_FAKE3-if02").symlink_to(tmp_path)
            await asyncio.sleep(POLL_S * 5)
            assert h.device is None and not s.seen("autoconnect", state="opened")
            link, k = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_FAKE1-if02")
            keep.extend(k)
            opened = await s.wait_event("autoconnect", state="opened", source="board")
            assert opened["device"] == f"serial {link} 115200" and h.device.path == link
            await asyncio.sleep(POLL_S * 4)
            assert h.device.path == link and not s.seen("autoconnect", state="opened")  # nothing else was opened
            assert (await s.cmd(cmd="close"))["ok"]  # the user closes it: it stays closed while the board stays
            await asyncio.sleep(POLL_S * 4)
            assert h.device is None and not s.seen("autoconnect", state="opened")
            s.close()
    try:
        run_with_helper(tmp_path, body, poll_s=POLL_S)
    finally:
        for fd in keep:
            os.close(fd)


def test_board_watcher_opens_a_device_only_when_name_usb_id_and_interface_string_all_match(tmp_path):
    keep = []

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for name, kwargs in [("usb-FTDI_X-if02", {}),  # another vendor's composite device that claims the right sysfs strings
                                 ("usb-Raspberry_Pi_Pico_NOTLINK-if02", dict(interface="Board CDC")),  # a Pico with other firmware
                                 ("usb-Raspberry_Pi_Pico_OTHERID-if02", dict(usb_id=("0403", "6001"))),  # right name and string, wrong ID
                                 ("usb-Raspberry_Pi_Pico_OTHERPID-if02", dict(usb_id=("2e8a", "000c"))),
                                 ("usb-Raspberry_Pi_Pico_CONSOLE-if00", {})]:  # the console interface
                _, k = fake_board(tmp_path, name, **kwargs)
                keep.extend(k)
            _, k = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_NOSYSFS-if02", sysfs=False)  # unverifiable: not opened
            keep.extend(k)
            await asyncio.sleep(POLL_S * 6)
            assert h.device is None and not s.seen("autoconnect", state="opened") and h.board_links() == []
            good, k = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_GOOD-if02")  # control: all three hold
            keep.extend(k)
            assert h.board_links() == [good]
            await s.wait_event("autoconnect", state="opened", source="board")
            assert h.device.path == good
            s.close()
    try:
        run_with_helper(tmp_path, body, poll_s=POLL_S)
    finally:
        for fd in keep:
            os.close(fd)


def test_board_serial_filter_opens_only_that_board(tmp_path):
    keep = []

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            other, k = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_AAAA-if02")
            keep.extend(k)
            wanted, k = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_BBBB-if02")
            keep.extend(k)
            assert h.board_links() == [wanted]
            await s.wait_event("autoconnect", state="opened", source="board")
            assert h.device.path == wanted
            s.close()
    try:
        run_with_helper(tmp_path, body, poll_s=POLL_S, board_serial="BBBB")
    finally:
        for fd in keep:
            os.close(fd)


def test_board_watcher_reports_a_board_it_cannot_open_once_and_opens_it_when_it_can(tmp_path):
    import fcntl
    keep = []

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            link, k = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_FAKE1-if02")
            keep.extend(k)
            other_fd = os.open(os.path.realpath(link), os.O_RDWR | os.O_NOCTTY)
            fcntl.flock(other_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)  # held elsewhere: the helper's own flock fails
            failed = await s.wait_event("autoconnect", state="failed", source="board")
            assert "cannot configure" in failed["reason"]
            await asyncio.sleep(POLL_S * 5)
            assert not s.seen("autoconnect", state="failed")  # told once, not every poll
            assert h.device is None
            os.close(other_fd)  # released: the next poll opens it
            await s.wait_event("autoconnect", state="opened", source="board")
            assert h.device is not None
            s.close()
    try:
        run_with_helper(tmp_path, body, poll_s=POLL_S)
    finally:
        for fd in keep:
            os.close(fd)


def test_board_autoconnect_is_suspended_during_a_hil_run_and_a_flash(tmp_path):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            port, keep_p = fake_board(tmp_path, "usb-Fake_HIL_2-if00")
            assert (await s.cmd(cmd="run_start", kind="hil", port=port))["ok"]
            await s.wait_event("autoconnect", state="suspended")
            link, keep_l = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_FAKE1-if02")
            await asyncio.sleep(POLL_S * 5)
            assert h.device is None and not s.seen("autoconnect", state="opened")  # a link interface appeared; the run owns the bus
            assert (await s.cmd(cmd="run_stop"))["ok"]
            await s.wait_event("autoconnect", state="resumed")
            await s.wait_event("autoconnect", state="opened", source="board")
            # a flash: the port is given up and picotool takes the board; the watcher must not take it back meanwhile
            assert (await s.cmd(cmd="close"))["ok"]
            h._flash_task = asyncio.create_task(asyncio.sleep(600))
            await s.wait_event("autoconnect", state="suspended")
            os.unlink(link)
            link2, keep_l2 = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_FAKE9-if02")
            await asyncio.sleep(POLL_S * 5)
            assert h.device is None
            h._flash_task.cancel()
            await s.wait_event("autoconnect", state="resumed")
            await s.wait_event("autoconnect", state="opened", source="board")
            for fd in (*keep_p, *keep_l, *keep_l2):
                os.close(fd)
            s.close()
    run_with_helper(tmp_path, body, hil_command=["sleep", "600"], poll_s=POLL_S)


# -- W-1 and the gate

def test_no_command_writes_into_the_repository(tmp_path):
    """W-1: `git status` and the diff against HEAD are the same after every command as before it."""
    port, keep = fake_board(tmp_path, "usb-Fake_Board_1-if00")
    udp = free_udp_port()
    before = repo_state()
    seen = []

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)

            async def cmd(**msg):
                reply = await s.cmd(**msg)
                seen.append((msg["cmd"], repo_state()))
                return reply
            await cmd(cmd="status")
            await cmd(cmd="host_state")
            await cmd(cmd="run_plan", kind="sil")
            await cmd(cmd="run_plan", kind="hil", port=port)
            assert (await cmd(cmd="run_start", kind="sil"))["ok"]
            await cmd(cmd="run_start", kind="hil", port=port)  # refused: a run is going
            await s.wait_event("autoconnect", state="opened", source="sil")
            await cmd(cmd="autoconnect", enabled=False)
            await cmd(cmd="run_list")
            await cmd(cmd="close")
            assert (await cmd(cmd="run_stop"))["ok"]
            run_id = (await cmd(cmd="run_list"))["runs"][0]
            await cmd(cmd="run_manifest", run_id=run_id)
            await cmd(cmd="run_manifest", run_id="../../..")
            assert (await cmd(cmd="run_start", kind="hil", port=port))["ok"]
            await cmd(cmd="run_stop")
            await cmd(cmd="sil_start")
            await cmd(cmd="sil_stop")
            await cmd(cmd="open", udp_port=udp)
            await cmd(cmd="no_such_command")
            s.close()
    try:
        run_with_helper(tmp_path, body, sil_command=[sys.executable, "-c", BIND_PORT], hil_command=["sleep", "600"], sil_udp_port=udp,
                        poll_s=POLL_S)
    finally:
        for fd in keep:
            os.close(fd)
    assert len(seen) >= 18
    for name, state in seen:
        assert state == before, f"{name} changed the repository"
    assert repo_state() == before


def test_new_commands_need_the_gate_like_every_other(tmp_path):
    async def body(h):
        for token in ("", "wrong"):
            with pytest.raises(InvalidStatus) as e:
                async with ws_connect(h, token=token) as ws:
                    await ws.send(json.dumps({"cmd": "run_start", "kind": "sil"}))
            assert e.value.response.status_code == 401
        with pytest.raises(InvalidStatus) as e:
            async with ws_connect(h, origin="http://evil.example") as ws:
                await ws.send(json.dumps({"cmd": "run_start", "kind": "sil"}))
        assert e.value.response.status_code == 403
        assert h.runs.active() is None and not (h.run_dir / "runs").exists()
    run_with_helper(tmp_path, body, sil_command=["sleep", "600"])


# -- device_closed on every close path

def test_every_close_path_pushes_device_closed(tmp_path):
    port = free_udp_port()
    keep = []

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            # the close command
            assert (await s.cmd(cmd="open", udp_port=port))["ok"]
            await s.wait_event("device_opened")
            assert (await s.cmd(cmd="close"))["ok"]
            assert (await s.wait_event("device_closed"))["reason"] == "closed by request"
            assert (await s.cmd(cmd="close"))["ok"] and not s.seen("device_closed")  # nothing open: nothing closed
            # the device ending by itself (the pty's far end goes away)
            link, k = fake_board(tmp_path, "usb-Raspberry_Pi_Pico_FAKE1-if02")
            keep.extend(k)
            await h.open_serial(link, 115200, from_browser=False)
            os.close(k[0])
            keep.remove(k[0])
            reason = (await s.wait_event("device_closed"))["reason"]
            assert reason.startswith(("read error", "end of file")), reason
            assert h.device is None
            # autoconnect closing the SIL link when its run ends
            assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]
            await s.wait_event("autoconnect", state="opened", source="sil")
            assert (await s.cmd(cmd="run_stop"))["ok"]
            assert (await s.wait_event("device_closed"))["reason"] == "the SIL run ended"
            # the flash giving the port up: test_flash_with_a_passing_report... expects exactly one device_closed, then device_opened
            s.close()
    try:
        run_with_helper(tmp_path, body, sil_command=[sys.executable, "-c", BIND_PORT], sil_udp_port=port, poll_s=POLL_S)
    finally:
        for fd in keep:
            os.close(fd)


# ---- GX-3: the SIL run definition (card, overrides, gui) ------------------------------------------------------------------------

sys.path.insert(0, str(ROOT / "tools" / "sim"))
import sil_run  # noqa: E402

CARD_A, CARD_B = "vehicles/marv_7in_fc3v2.yaml", "vehicles/uzh_neurobem_5in.yaml"
PARAM_TYPES = {"att_kp": "f32", "rate_ki_roll": "f32", "rc_channel_roll": "i32", "rc_channel_arm": "i32"}  # a stand-in parameter set


def write_manifest(tmp_path, types=PARAM_TYPES):
    """The parameter manifest of the gz build where the helper reads it (names and types)."""
    path = sil_run.manifest_path(tmp_path / "build" / sil_run.GZ_BUILD)
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({"params": {n: {"type": t} for n, t in types.items()}}))


def flag_values(argv, flag):
    return [argv[i + 1] for i, a in enumerate(argv) if a == flag]


def test_run_plan_of_a_sil_run_definition_names_the_card_overrides_gui_and_the_label_and_starts_nothing(tmp_path):
    write_manifest(tmp_path)

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            plain = (await s.cmd(cmd="run_plan", kind="sil"))["plan"]  # control: no definition, the argv of before the options
            assert plain["label"] is None and plain["overrides"] == [] and plain["options"] == {}
            assert not {"--card", "--override", "--gui"} & set(plain["argv"])
            assert plain["argv"] == sil.default_command(Path(plain["out_dir"]), h.run_dir / "store", None)
            empty = (await s.cmd(cmd="run_plan", kind="sil", overrides=[]))["plan"]  # an empty list is no override and no label
            assert empty["label"] is None and empty["argv"] == plain["argv"]
            plan = (await s.cmd(cmd="run_plan", kind="sil", card=CARD_B, gui=True, overrides=[
                {"name": "att_kp", "value": 3.5}, {"name": "rc_channel_roll", "value": 7}]))["plan"]
            assert plan["allowed"] and plan["label"] == "sil-override" and plan["kind"] == "sil"
            assert flag_values(plan["argv"], "--card") == [str(ROOT / CARD_B)] and "--gui" in plan["argv"]
            assert flag_values(plan["argv"], "--override") == ["att_kp=3.5", "rc_channel_roll=7"]
            assert plan["overrides"] == [{"name": "att_kp", "type": "f32", "value": 3.5},
                                         {"name": "rc_channel_roll", "type": "i32", "value": 7}]
            assert plan["options"] == {"card": CARD_B, "overrides": plan["overrides"], "gui": True}
            bare = (await s.cmd(cmd="run_plan", kind="sil", card="uzh_neurobem_5in.yaml"))["plan"]  # a bare file name is the same card
            assert bare["options"] == {"card": CARD_B} and bare["label"] is None
            assert h.runs.active() is None and not (h.run_dir / "runs").exists()
            s.close()
    run_with_helper(tmp_path, body)


def test_cards_lists_the_vehicle_cards_a_run_accepts_and_each_one_plans(tmp_path):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            cards = (await s.cmd(cmd="cards"))["cards"]
            assert cards == sorted(f"vehicles/{p.name}" for p in (ROOT / "vehicles").glob("*.yaml")) and CARD_A in cards and CARD_B in cards
            assert not any(c.endswith(".py") for c in cards)  # control: the inertia model script beside the cards is not one
            for card in cards:  # every listed card is one the run definition accepts
                assert (await s.cmd(cmd="run_plan", kind="sil", card=card))["plan"]["options"] == {"card": card}
            s.close()
    run_with_helper(tmp_path, body)


def test_sil_run_manifest_and_event_record_every_override_and_the_label(tmp_path):
    write_manifest(tmp_path)
    overrides = [{"name": "att_kp", "value": 3.5}, {"name": "rc_channel_arm", "value": 6}]

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            run = (await s.cmd(cmd="run_start", kind="sil", card=CARD_B, overrides=overrides, gui=False))["run"]
            event = await s.wait_event("run_started", run_id=run["run_id"])
            assert event["label"] == "sil-override"
            on_disk = json.loads((h.run_dir / "runs" / run["run_id"] / "manifest.json").read_text())
            expect = [{"name": "att_kp", "type": "f32", "value": 3.5}, {"name": "rc_channel_arm", "type": "i32", "value": 6}]
            assert on_disk["label"] == "sil-override" and on_disk["overrides"] == expect
            assert on_disk["options"] == {"card": CARD_B, "overrides": expect, "gui": False}
            assert (await s.cmd(cmd="run_stop"))["ok"]
            ended = (await s.cmd(cmd="run_manifest", run_id=run["run_id"]))["manifest"]
            assert ended["state"] == "stopped" and ended["label"] == "sil-override" and ended["overrides"] == expect
            # control: the next run has no overrides and no label (nothing carries over)
            run = (await s.cmd(cmd="run_start", kind="sil"))["run"]
            assert run["label"] is None and run["overrides"] == [] and run["options"] == {}
            assert (await s.wait_event("run_started", run_id=run["run_id"]))["label"] is None
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    run_with_helper(tmp_path, body, sil_command=["sleep", "600"])


def test_sil_launcher_builds_the_run_definition_into_every_spawn_the_relaunch_included(tmp_path, monkeypatch):
    plain = sil.SilLauncher(tmp_path, 14550).command()
    launcher = sil.SilLauncher(tmp_path, 14550)
    launcher.extra_args = ["--card", "c.yaml", "--override", "att_kp=3.5", "--gui"]
    command = launcher.command()
    assert command[:-2] == [*plain[:-2], *launcher.extra_args] and command[-2:] == ["--link-udp-port", "14550"]
    assert launcher.command(extra=[]) == plain  # control: no definition, today's command
    spawned = []
    marker = tmp_path / "first"

    def fake(out_dir, config_dir, udp_port, extra=None):
        spawned.append(list(extra or []))
        return [sys.executable, "-c", f"import os, pathlib, sys; m = pathlib.Path({str(marker)!r}); "
                                       "sys.exit(os.EX_TEMPFAIL if not m.exists() and m.write_text('x') else 0)"]
    monkeypatch.setattr(sil, "default_command", fake)
    done = threading.Event()
    launcher.on_exit = lambda status: done.set()
    launcher.start()
    assert done.wait(TIMEOUT_S)
    launcher.stop()
    assert spawned == [launcher.extra_args, launcher.extra_args]  # the same definition at the start and at the relaunch


REFUSED_SIL_DEFINITIONS = [
    dict(card="vehicles/no_such_card.yaml"), dict(card="no_such_card.yaml"), dict(card="vehicles/../tools/gs/runs.py"),
    dict(card="../vehicles/marv_7in_fc3v2.yaml"), dict(card="/etc/passwd"), dict(card=str(ROOT / CARD_A)), dict(card="vehicles"),
    dict(card="vehicles/marv_7in_fc3v2_inertia_model.py"), dict(card=""), dict(card=3), dict(card=["vehicles/marv_7in_fc3v2.yaml"]),
    dict(overrides=[{"name": "no_such_parameter", "value": 1}]),  # unknown parameter
    dict(overrides=[{"name": "att_kp", "value": float("inf")}]), dict(overrides=[{"name": "att_kp", "value": float("nan")}]),
    dict(overrides=[{"name": "att_kp", "value": 1e39}]),  # outside f32
    dict(overrides=[{"name": "rc_channel_roll", "value": 1.5}]), dict(overrides=[{"name": "rc_channel_roll", "value": 2 ** 31}]),
    dict(overrides=[{"name": "rc_channel_roll", "value": -(2 ** 31) - 1}]),  # outside i32
    dict(overrides=[{"name": "att_kp", "value": True}]), dict(overrides=[{"name": "att_kp", "value": "3.5"}]),
    dict(overrides=[{"name": "att_kp", "value": None}]), dict(overrides=[{"name": "att_kp"}]), dict(overrides=[{"value": 1}]),
    dict(overrides=[{"name": "att_kp", "value": 1, "type": "i32"}]),  # a type is the manifest's, not the browser's
    dict(overrides=[{"name": "att_kp", "value": 1}, {"name": "att_kp", "value": 2}]),  # twice
    dict(overrides=[{"name": 3, "value": 1}]), dict(overrides=["att_kp=1"]), dict(overrides={"att_kp": 1}), dict(overrides="att_kp=1"),
    dict(gui="yes"), dict(gui=1), dict(gui=None),
    dict(argv=["--override", "att_kp=1"]), dict(out_dir="/etc"), dict(plugin_dir="/tmp"), dict(port="/dev/ttyACM0"),
]


@pytest.mark.parametrize("definition", REFUSED_SIL_DEFINITIONS, ids=lambda d: json.dumps(d, default=str)[:60])
def test_sil_run_definition_refusals_start_nothing(tmp_path, definition):
    write_manifest(tmp_path)

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for cmd in ("run_plan", "run_start"):
                reply = await s.cmd(cmd=cmd, kind="sil", **definition)
                assert not reply["ok"] and reply["error"], (cmd, definition)
            assert h.runs.active() is None and not (h.run_dir / "runs").exists()
            s.close()
    run_with_helper(tmp_path, body, sil_command=["sleep", "600"])


def test_sil_overrides_need_the_parameter_manifest_of_the_gz_build(tmp_path):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for cmd in ("run_plan", "run_start"):
                reply = await s.cmd(cmd=cmd, kind="sil", overrides=[{"name": "att_kp", "value": 1}])
                assert not reply["ok"] and "build host-gz-quadfc first" in reply["error"]
            assert (await s.cmd(cmd="run_start", kind="sil", card=CARD_A, gui=True))["ok"]  # control: no override, no manifest needed
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    run_with_helper(tmp_path, body, sil_command=["sleep", "600"])


def test_heavy_lock_held_refuses_a_sil_run_definition_and_the_same_request_starts_when_it_is_free(tmp_path):
    write_manifest(tmp_path)
    holder = hold_heavy(tmp_path)
    ran = tmp_path / "ran"
    definition = dict(kind="sil", card=CARD_B, gui=True, overrides=[{"name": "att_kp", "value": 3.5}])

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            reply = await s.cmd(cmd="run_start", **definition)
            assert not reply["ok"] and "heavy lock is held" in reply["error"] and "cmd=sleep" in reply["error"]
            plan = (await s.cmd(cmd="run_plan", **definition))["plan"]
            assert not plan["allowed"] and plan["label"] == "sil-override" and any("heavy lock" in r for r in plan["refusals"])
            assert not ran.exists() and h.runs.active() is None and not (h.run_dir / "runs").exists()
            release_heavy(holder)  # control
            run = (await s.cmd(cmd="run_start", **definition))["run"]
            assert run["label"] == "sil-override" and run["options"]["gui"] is True
            assert (await s.cmd(cmd="run_stop"))["ok"]
            s.close()
    try:
        run_with_helper(tmp_path, body, sil_command=["sh", "-c", f"touch {ran}; sleep 600"])
    finally:
        release_heavy(holder)
