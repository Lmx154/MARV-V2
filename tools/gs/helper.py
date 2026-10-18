#!/usr/bin/env python3
"""The ground-segment helper (docs/spec/20-ground-segment.md section 3, decision 0053 Q5): a local server that

  - owns ONE device connection at a time, a serial port or the SIL's UDP socket (127.0.0.1:<port>), and relays raw bytes both
    ways to the browser over a localhost WebSocket at /ws. It never parses the bytes in the data path;
  - records the ground log (tools/gs/groundlog.py): every byte in each direction, timestamped per chunk;
  - serves the image library (tools/gs/artifacts.py) over HTTP GET under /api/images;
  - launches and stops a Gazebo SIL run (tools/gs/sil.py);
  - validates a staged config for an image of the library (tools/validate/validate_config.py, ground-segment unit 9, G7) and
    keeps the reports keyed by (image hash, config hash) (tools/gs/artifacts.py ReportLibrary);
  - serves the built GCS (gcs/build) at /.

  uv run python tools/gs/helper.py [--serial PATH [--baud N] | --udp PORT] [--port N] [--runs-dir DIR] [--build-root DIR]
                                   [--gcs-dir DIR] [--sil-udp-port N] [--sil-command CMD] [--reports-dir DIR]
                                   [--validate-command CMD] [--extra-origin ORIGIN ...]

Security. It binds 127.0.0.1 only. Every request must carry the Host `127.0.0.1:<port>`; an Origin header, when present, must
be exactly `http://127.0.0.1:<port>` (the WebSocket requires it); and every request needs the per-session token (random, printed
at start, never written to disk), as `?token=`, as `Authorization: Bearer`, or as the cookie the helper sets when the GCS page
is first opened with `?token=` (HttpOnly, SameSite=Strict). The served HTML carries the token in a <meta name="marv-helper">.
Anything else is refused: 405 for a method other than GET, 403 for a wrong Host or Origin, 401 for a missing or wrong token.
One WebSocket client at a time. A serial path chosen from the browser must be a tty device name (SERIAL_GLOBS).
--extra-origin ORIGIN (repeatable, default none) additionally accepts that exact origin, e.g. a vite dev server
`http://localhost:5173`: DEVELOPMENT ONLY. Such a page still needs the token (the cookie does not cross origins, so it
passes `?token=`), and the helper answers it with Access-Control-Allow-Origin for that origin.
SIGINT, SIGTERM and SIGHUP stop the helper cleanly: the SIL run is stopped and the ground log closed.

Endpoints (all need the token):
  GET  /                          the built GCS (gcs/build), 404 when it is not built
  GET  /api/status                JSON: device, client, run directory, SIL
  GET  /api/ports                 JSON: serial port candidates
  GET  /api/images                JSON: the image library (tools/gs/artifacts.py)
  GET  /api/images/<hash>/<name>  an artifact of the image with that SHA-256: image, elf, uf2, params_manifest,
                                  params_provenance, factory_mcfg
  GET  /api/reports/<image hash>/<config hash>
                                  the validation report of that pair (the bytes whose SHA-256 is the report id), 404 if none
  WS   /ws                        binary frames are raw device bytes in both directions; text frames are JSON control:
         {"cmd": "status"} {"cmd": "open", "serial": "/dev/...", "baud": N} {"cmd": "open", "udp_port": N} {"cmd": "close"}
         {"cmd": "sil_start"} {"cmd": "sil_stop"}
         {"cmd": "config_report", "image_hash": H, "view": V}: the config hash of V and whether a report of (H, that hash) is
           stored ({"config_hash", "stored", "verdict"})
         {"cmd": "validate", "image_hash": H, "view": V}: writes V as a .mcfg (tools/config/mcfg.py, the format's single
           source) and runs the validator on it for the library's image H, one validation at a time; answered at once
           ({"config_hash", "started": true}); the run pushes {"event": "validate_progress", "line"} per progress line and
           {"event": "validate_done", "image_hash", "config_hash", "verdict", "report_id", "exit_status"}.
         V is the JSON view of tools/config/mcfg.py (decode/dump: sections of records with their bits, enumerations and source
         texts); the GCS builds it from the vehicle's staged records. Each command is answered {"ok": true|false, ...} (an
         "id" field is echoed). Pushed events: {"event": "device_opened"|"device_closed", ...} and the validation's.
"""

from __future__ import annotations

import argparse
import asyncio
import fnmatch
import glob
import hashlib
import hmac
import json
import mimetypes
import os
import secrets
import shlex
import signal
import sys
import time
from datetime import datetime, timezone
from http import HTTPStatus
from http.cookies import SimpleCookie
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "tools" / "config"))
import groundlog  # noqa: E402
import mcfg  # noqa: E402
from artifacts import ARTIFACT_KEYS, ImageLibrary, ReportLibrary  # noqa: E402
from device import DEFAULT_BAUD, LOOPBACK, Device, DeviceError, SerialDevice, UdpDevice  # noqa: E402
from sil import SilLauncher  # noqa: E402

from websockets.asyncio.server import ServerConnection, serve  # noqa: E402
from websockets.datastructures import Headers  # noqa: E402
from websockets.http11 import Request, Response  # noqa: E402

DEFAULT_PORT = 8765  # a convenience default for a stable URL (unprivileged, unassigned); --port 0 picks a free port
COOKIE = "marv_gs_token"
CLIENT_QUEUE_MAX_BYTES = 4 << 20  # labelled harness value: bytes queued for a slow browser before chunks are dropped (counted, logged)
SERIAL_GLOBS = ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/serial/by-id/*")
WS_BUSY = 1013  # RFC 6455 section 7.4.2 "try again later"
VALIDATOR = ROOT / "tools" / "validate" / "validate_config.py"
VALIDATION_STOP_S = 10  # labelled harness value: how long the helper waits for a stopped validation to end
# The validator's command line; {config} {image} {build} {work} {out} are replaced (--validate-command replaces it in tests).
VALIDATE_COMMAND = (sys.executable, str(VALIDATOR), "--config", "{config}", "--image", "{image}", "--build", "{build}",
                    "--work-dir", "{work}", "--out", "{out}")


def utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


class Helper:
    def __init__(self, *, runs_dir: Path, build_root: Path, gcs_dir: Path, sil_udp_port: int | None, token: str | None = None,
                 sil_command: list[str] | None = None, extra_origins: tuple[str, ...] = (), reports_dir: Path | None = None,
                 validate_command: list[str] | None = None):
        self.token = token or secrets.token_urlsafe(32)
        self.extra_origins = frozenset(extra_origins)
        self.build_root, self.gcs_dir = Path(build_root), Path(gcs_dir)
        self.run_dir = Path(runs_dir) / utc_stamp()
        suffix = 0
        while True:
            try:
                self.run_dir.mkdir(parents=True)
                break
            except FileExistsError:
                suffix += 1
                self.run_dir = Path(runs_dir) / f"{utc_stamp()}-{suffix}"
        self.log = groundlog.GroundLog(self.run_dir / "ground.log")
        self.images = ImageLibrary(self.build_root)
        self.reports = ReportLibrary(reports_dir if reports_dir is not None else self.build_root / "gs-reports")
        self.validate_command = list(validate_command or VALIDATE_COMMAND)
        self.validation: dict | None = None
        self._validation_task: asyncio.Task | None = None
        self._validation_proc: asyncio.subprocess.Process | None = None
        self.sil = SilLauncher(self.run_dir, sil_udp_port, sil_command)
        self.port = 0
        self.device: Device | None = None
        self.client: ServerConnection | None = None
        self._queue: asyncio.Queue | None = None
        self._queued_bytes = 0
        self.dropped_bytes = 0
        self._server = None

    # ---- lifecycle ------------------------------------------------------------------------------------------------------

    async def start(self, port: int) -> None:
        self._server = await serve(self._ws_handler, LOOPBACK, port, process_request=self._process_request, compression=None)
        self.port = self._server.sockets[0].getsockname()[1]
        self.log.event(f"helper started on {self.origin}")

    @property
    def origin(self) -> str:
        return f"http://{LOOPBACK}:{self.port}"

    def url(self) -> str:
        return f"{self.origin}/?token={self.token}"

    async def stop(self) -> None:
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
        self.close_device("helper stopping")
        await self.stop_validation()
        await asyncio.to_thread(self.sil.stop)
        self.log.event("helper stopped")
        self.log.close()

    # ---- device and relay -----------------------------------------------------------------------------------------------

    def _device_rx(self, data: bytes) -> None:
        self.log.write(groundlog.RX, data)
        if self._queue is not None:
            self._push(data)

    def _device_closed(self, reason: str) -> None:
        self.log.event(f"device closed: {reason}")
        self.device = None
        self._push(json.dumps({"event": "device_closed", "reason": reason}))

    def _push(self, item: bytes | str) -> None:
        if self._queue is None:
            return
        if isinstance(item, bytes) and self._queued_bytes + len(item) > CLIENT_QUEUE_MAX_BYTES:
            self.dropped_bytes += len(item)
            self.log.event(f"relay to browser dropped {len(item)} bytes (queue full)")
            return
        if isinstance(item, bytes):
            self._queued_bytes += len(item)
        self._queue.put_nowait(item)

    def close_device(self, reason: str = "closed by request") -> None:
        if self.device is not None:
            self.device.close()
            self.log.event(f"device closed: {reason}")
            self.device = None

    async def open_serial(self, path: str, baud: int, *, from_browser: bool) -> None:
        if from_browser and not (os.path.normpath(path) == path and any(fnmatch.fnmatchcase(path, g) for g in SERIAL_GLOBS)
                                 and os.path.realpath(path).startswith("/dev/")):
            raise DeviceError("a serial path from the browser must be a tty device: " + ", ".join(SERIAL_GLOBS))
        self._require_no_device()
        self.device = SerialDevice(path, baud, self._device_rx, self._device_closed)
        self._opened()

    async def open_udp(self, port: int) -> None:
        self._require_no_device()
        self.device = await UdpDevice.open(port, self._device_rx, self._device_closed)
        self._opened()

    def _require_no_device(self) -> None:
        if self.device is not None:
            raise DeviceError("a device is already open; close it first (one connection at a time)")

    def _opened(self) -> None:
        self.log.event(f"device opened: {self.device.description}")
        self._push(json.dumps({"event": "device_opened", "device": self.device.description}))

    def write_device(self, data: bytes) -> None:
        if self.device is None:
            return
        self.log.write(groundlog.TX, data)
        self.device.write(data)

    # ---- status ---------------------------------------------------------------------------------------------------------

    def status(self) -> dict:
        return {
            "origin": self.origin,
            "run_dir": str(self.run_dir),
            "device": self.device.description if self.device else None,
            "client": self.client is not None,
            "dropped_bytes": self.dropped_bytes,
            "sil": self.sil.status(),
            "validation": self.validation,
            "gcs_built": (self.gcs_dir / "index.html").is_file(),
        }

    # ---- validation (G7) ------------------------------------------------------------------------------------------------

    @staticmethod
    def config_of_view(view) -> tuple[bytes, str]:
        """(the .mcfg bytes, the config hash hex) of a JSON view; ValueError for a view the format refuses."""
        if not isinstance(view, dict):
            raise ValueError("view must be a JSON object (tools/config/mcfg.py's JSON view)")
        try:
            cfg = mcfg.config_from_view(view)
            return mcfg.encode(cfg), mcfg.config_hash(cfg).hex()
        except (mcfg.McfgError, KeyError, TypeError, ValueError) as e:
            raise ValueError(f"the view is not a config: {type(e).__name__}: {e}") from None

    def config_report(self, msg: dict) -> dict:
        image_hash = str(msg.get("image_hash", ""))
        _, config_hash = self.config_of_view(msg.get("view"))
        report = self.reports.report(image_hash, config_hash)
        verdict = None
        if report is not None:
            verdict = json.loads(report.read_bytes()).get("verdict")
        return {"config_hash": config_hash, "stored": report is not None, "verdict": verdict}

    async def start_validation(self, msg: dict) -> dict:
        if self._validation_task is not None and not self._validation_task.done():
            raise DeviceError("a validation is running; wait for it to finish")
        image_hash = str(msg.get("image_hash", ""))
        data, config_hash = self.config_of_view(msg.get("view"))
        image = self.images.lookup(image_hash, "image")
        target = self.reports.directory(image_hash, config_hash)
        if image is None or target is None:
            raise DeviceError(f"the image {image_hash!r} is not in the image library")
        tree = self.build_root / image.relative_to(self.build_root.resolve()).parts[0]
        target.mkdir(parents=True, exist_ok=True)
        config = target / "config.mcfg"
        config.write_bytes(data)
        work = self.run_dir / "validation" / f"{image_hash[:16]}-{config_hash[:16]}-{utc_stamp()}"
        fields = {"config": str(config), "image": str(image), "build": str(tree), "work": str(work),
                  "out": str(target / "report.json")}
        cmd = []
        for part in self.validate_command:
            for key, value in fields.items():
                part = part.replace("{" + key + "}", value)
            cmd.append(part)
        self.validation = {"running": True, "image_hash": image_hash, "config_hash": config_hash}
        self.log.event(f"validation started: image {image_hash} config {config_hash}")
        self._validation_task = asyncio.create_task(self._run_validation(cmd, image_hash, config_hash))
        return {"config_hash": config_hash, "started": True}

    async def _run_validation(self, cmd: list[str], image_hash: str, config_hash: str) -> None:
        verdict, report_id, status = None, None, None
        try:
            proc = await asyncio.create_subprocess_exec(*cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
                                                        start_new_session=True)
            self._validation_proc = proc

            async def progress():
                async for raw in proc.stderr:
                    self._push(json.dumps({"event": "validate_progress", "image_hash": image_hash, "config_hash": config_hash,
                                           "line": raw.decode("utf-8", "replace").rstrip()}))

            reader = asyncio.create_task(progress())
            out = (await proc.stdout.read()).decode("utf-8", "replace")
            await reader
            status = await proc.wait()
            for line in out.splitlines():
                if line.startswith("report_id: "):
                    report_id = line[len("report_id: "):].strip()
                elif line.startswith("verdict: "):
                    verdict = line[len("verdict: "):].strip()
        except OSError as e:
            self._push(json.dumps({"event": "validate_progress", "image_hash": image_hash, "config_hash": config_hash,
                                   "line": f"the validator did not start: {e}"}))
        finally:
            self._validation_proc = None
            stored = self.reports.report(image_hash, config_hash)
            if stored is None or report_id is None or hashlib.sha256(stored.read_bytes()).hexdigest() != report_id:
                verdict, report_id = None, None  # no report of this run: the GCS shows the progress lines
            self.validation = {"running": False, "image_hash": image_hash, "config_hash": config_hash, "verdict": verdict,
                               "report_id": report_id, "exit_status": status}
            self.log.event(f"validation done: image {image_hash} config {config_hash} verdict {verdict} report {report_id}")
            self._push(json.dumps({"event": "validate_done", **self.validation}))

    async def stop_validation(self) -> None:
        proc = self._validation_proc
        if proc is not None and proc.returncode is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        if self._validation_task is not None:
            try:
                await asyncio.wait_for(self._validation_task, VALIDATION_STOP_S)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                self._validation_task.cancel()

    # ---- WebSocket ------------------------------------------------------------------------------------------------------

    async def _ws_handler(self, ws: ServerConnection) -> None:
        if self.client is not None:
            await ws.close(WS_BUSY, "another client is attached")
            return
        self.client = ws
        self._queue = queue = asyncio.Queue()
        self._queued_bytes = 0
        sender = asyncio.create_task(self._sender(ws, queue))
        try:
            async for message in ws:
                if isinstance(message, bytes):
                    self.write_device(message)
                else:
                    self._push(json.dumps(await self._control(message)))
        finally:
            self.client, self._queue = None, None
            sender.cancel()

    async def _sender(self, ws: ServerConnection, queue: asyncio.Queue) -> None:
        while True:
            item = await queue.get()
            if isinstance(item, bytes):
                self._queued_bytes -= len(item)
            await ws.send(item)

    async def _control(self, text: str) -> dict:
        try:
            msg = json.loads(text)
            cmd = msg["cmd"]
        except (ValueError, KeyError, TypeError):
            return {"ok": False, "error": "control message must be JSON with a cmd"}
        reply: dict = {"ok": True}
        if "id" in msg:
            reply["id"] = msg["id"]
        try:
            if cmd == "status":
                reply["status"] = self.status()
            elif cmd == "open" and "serial" in msg:
                await self.open_serial(str(msg["serial"]), int(msg.get("baud", DEFAULT_BAUD)), from_browser=True)
            elif cmd == "open" and "udp_port" in msg:
                await self.open_udp(int(msg["udp_port"]))
            elif cmd == "close":
                self.close_device()
            elif cmd == "sil_start":
                self.sil.start()
                self.log.event("SIL run started")
            elif cmd == "sil_stop":
                reply["exit_status"] = await asyncio.to_thread(self.sil.stop)
                self.log.event("SIL run stopped")
            elif cmd == "config_report":
                reply.update(self.config_report(msg))
            elif cmd == "validate":
                reply.update(await self.start_validation(msg))
            else:
                raise DeviceError(f"unknown command {cmd!r}")
        except (DeviceError, RuntimeError, ValueError, TypeError, OSError) as e:
            reply = {"ok": False, "error": str(e), **({"id": msg["id"]} if "id" in msg else {})}
        return reply

    # ---- HTTP and the handshake gate ------------------------------------------------------------------------------------

    def _refuse(self, status: HTTPStatus, why: str) -> Response:
        return self._response(status, why.encode() + b"\n", "text/plain; charset=utf-8")

    @staticmethod
    def _response(status: HTTPStatus, body: bytes, content_type: str, extra: list[tuple[str, str]] = ()) -> Response:
        headers = Headers([("Content-Type", content_type), ("Content-Length", str(len(body))), ("Cache-Control", "no-store"),
                           ("X-Content-Type-Options", "nosniff"), ("Referrer-Policy", "no-referrer"), ("Connection", "close"), *extra])
        return Response(status.value, status.phrase, headers, body)

    def _token_of(self, request: Request) -> tuple[str | None, bool]:
        """(token presented, whether it came in the query)."""
        query = parse_qs(urlsplit(request.path).query)
        if query.get("token"):
            return query["token"][0], True
        auth = request.headers.get("Authorization", "")
        if auth.startswith("Bearer "):
            return auth[len("Bearer "):], False
        jar = SimpleCookie()
        try:
            jar.load(request.headers.get("Cookie", ""))
        except Exception:  # a malformed Cookie header carries no token
            return None, False
        return (jar[COOKIE].value, False) if COOKIE in jar else (None, False)

    def _gate(self, request: Request, websocket: bool) -> tuple[Response | None, bool]:
        """(refusal or None, token came in the query)."""
        if request.headers.get("Host") != f"{LOOPBACK}:{self.port}":
            return self._refuse(HTTPStatus.FORBIDDEN, "wrong Host"), False
        origin = request.headers.get("Origin")
        if (origin is None and websocket) or (origin is not None and origin != self.origin and origin not in self.extra_origins):
            return self._refuse(HTTPStatus.FORBIDDEN, "foreign Origin"), False
        token, in_query = self._token_of(request)
        if token is None or not hmac.compare_digest(token.encode(), self.token.encode()):
            return self._refuse(HTTPStatus.UNAUTHORIZED, "missing or wrong token"), False
        return None, in_query

    def _process_request(self, connection: ServerConnection, request: Request) -> Response | None:
        if request.method != "GET":
            return self._response(HTTPStatus.METHOD_NOT_ALLOWED, b"only GET is served\n", "text/plain; charset=utf-8", [("Allow", "GET")])
        response = self._route(request)
        origin = request.headers.get("Origin")
        if response is not None and origin in self.extra_origins and response.status_code < HTTPStatus.BAD_REQUEST:
            response.headers["Access-Control-Allow-Origin"] = origin
            response.headers["Vary"] = "Origin"
        return response

    def _route(self, request: Request) -> Response | None:
        path = unquote(urlsplit(request.path).path)
        websocket = request.headers.get("Upgrade", "").lower() == "websocket"
        refusal, in_query = self._gate(request, websocket)
        if refusal is not None:
            return refusal
        if websocket:
            return None if path == "/ws" else self._refuse(HTTPStatus.NOT_FOUND, "no such WebSocket path")
        if path == "/ws":
            return self._refuse(HTTPStatus.UPGRADE_REQUIRED, "WebSocket only")
        try:
            return self._http_get(path, in_query)
        except Exception as e:  # a failed request must not take the helper down
            return self._refuse(HTTPStatus.INTERNAL_SERVER_ERROR, f"{type(e).__name__}: {e}")

    def _json(self, obj) -> Response:
        return self._response(HTTPStatus.OK, json.dumps(obj, indent=1).encode(), "application/json")

    def _http_get(self, path: str, set_cookie: bool) -> Response:
        if path == "/api/status":
            return self._json(self.status())
        if path == "/api/ports":
            return self._json({"serial": sorted(p for g in SERIAL_GLOBS for p in glob.glob(g))})
        if path == "/api/images":
            return self._json(self.images.scan())
        if path.startswith("/api/images/"):
            parts = path[len("/api/images/"):].split("/")
            file = self.images.lookup(parts[0], parts[1]) if len(parts) == 2 else None
            if file is None or parts[1] not in ARTIFACT_KEYS:
                return self._refuse(HTTPStatus.NOT_FOUND, "no such image or artifact")
            return self._response(HTTPStatus.OK, file.read_bytes(), "application/octet-stream",
                                  [("Content-Disposition", f'attachment; filename="{file.name}"')])
        if path.startswith("/api/reports/"):
            parts = path[len("/api/reports/"):].split("/")
            report = self.reports.report(parts[0], parts[1]) if len(parts) == 2 else None
            if report is None:
                return self._refuse(HTTPStatus.NOT_FOUND, "no report of that image and config")
            return self._response(HTTPStatus.OK, report.read_bytes(), "application/json")
        if path.startswith("/api/"):
            return self._refuse(HTTPStatus.NOT_FOUND, "no such endpoint")
        return self._static(path, set_cookie)

    def _static(self, path: str, set_cookie: bool) -> Response:
        root = self.gcs_dir.resolve()
        rel = path.lstrip("/")
        candidates = [root / rel] if rel else []
        if not rel or path.endswith("/"):
            candidates = [root / rel / "index.html"]
        else:
            candidates += [root / (rel + ".html"), root / rel / "index.html"]
        for c in candidates:
            c = c.resolve()
            if c.is_file() and root in c.parents:
                body = c.read_bytes()
                ctype = mimetypes.guess_type(c.name)[0] or "application/octet-stream"
                if c.suffix == ".html":
                    meta = f'<meta name="marv-helper" content=\'{json.dumps({"token": self.token, "ws": "/ws"})}\'>'.encode()
                    head = body.find(b"</head>")
                    body = body[:head] + meta + body[head:] if head >= 0 else meta + body
                extra = [("Set-Cookie", f"{COOKIE}={self.token}; Path=/; HttpOnly; SameSite=Strict")] if set_cookie else []
                return self._response(HTTPStatus.OK, body, ctype, extra)
        return self._refuse(HTTPStatus.NOT_FOUND, "not found (is the GCS built? cd gcs && npm run build)")


async def run(args: argparse.Namespace) -> int:
    helper = Helper(runs_dir=args.runs_dir, build_root=args.build_root, gcs_dir=args.gcs_dir, sil_udp_port=args.sil_udp_port,
                    sil_command=shlex.split(args.sil_command) if args.sil_command else None, extra_origins=tuple(args.extra_origin),
                    reports_dir=args.reports_dir,
                    validate_command=shlex.split(args.validate_command) if args.validate_command else None)
    await helper.start(args.port)
    try:
        if args.serial:
            await helper.open_serial(args.serial, args.baud, from_browser=False)
        elif args.udp:
            await helper.open_udp(args.udp)
    except DeviceError as e:
        print(f"helper: {e}", file=sys.stderr)
        await helper.stop()
        return 1
    print(f"GCS:     {helper.url()}", flush=True)
    print(f"run dir: {helper.run_dir}", flush=True)
    print(f"device:  {helper.device.description if helper.device else 'none (open one from the GCS)'}", flush=True)
    stopping = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        asyncio.get_running_loop().add_signal_handler(sig, stopping.set)
    try:
        await stopping.wait()
    finally:
        await helper.stop()
    return 0


def origin_arg(text: str) -> str:
    u = urlsplit(text)
    if u.scheme not in ("http", "https") or not u.hostname or u.path or u.query or u.fragment or u.username or text.endswith("/"):
        raise argparse.ArgumentTypeError(f"{text!r} is not an origin (scheme://host[:port], no path)")
    return text


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    dev = ap.add_mutually_exclusive_group()
    dev.add_argument("--serial", help="open this serial path at start (the board's link CDC)")
    dev.add_argument("--udp", type=int, help="open the SIL's UDP socket 127.0.0.1:<port> at start (a scenario value)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="serial baud rate, ignored by USB CDC (default %(default)s)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="helper port on 127.0.0.1; 0 picks a free one (default %(default)s)")
    ap.add_argument("--runs-dir", type=Path, default=ROOT / "build" / "gs-runs", help="parent of the per-start run directories")
    ap.add_argument("--build-root", type=Path, default=ROOT / "build", help="directory of the build trees the image library scans")
    ap.add_argument("--gcs-dir", type=Path, default=ROOT / "gcs" / "build", help="the built GCS served at /")
    ap.add_argument("--sil-udp-port", type=int, default=None, help="UDP port the launched SIL run is to serve (a scenario value)")
    ap.add_argument("--sil-command", help="the command line `sil_start` runs instead of tools/sim/run_quad_fc.py (shell-style quoting; for tests)")
    ap.add_argument("--reports-dir", type=Path, default=None,
                    help="the validation reports, keyed by image and config hash (default <build-root>/gs-reports)")
    ap.add_argument("--validate-command", help="the validator command line with {config} {image} {build} {work} {out} "
                                               "(shell-style quoting; for tests; default tools/validate/validate_config.py)")
    ap.add_argument("--extra-origin", action="append", default=[], type=origin_arg, metavar="ORIGIN",
                    help="also accept this exact origin (repeatable), e.g. http://localhost:5173 for a vite dev server: DEVELOPMENT ONLY")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    try:
        return asyncio.run(run(parse_args(argv)))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
