#!/usr/bin/env python3
"""The ground-segment helper (docs/spec/20-ground-segment.md section 3, decision 0053 Q5): a local server that

  - owns ONE device connection at a time, a serial port or the SIL's UDP socket (127.0.0.1:<port>), and relays raw bytes both
    ways to the browser over a localhost WebSocket at /ws. It never parses the bytes in the data path;
  - records the ground log (tools/gs/groundlog.py): every byte in each direction, timestamped per chunk;
  - serves the image library (tools/gs/artifacts.py) over HTTP GET under /api/images;
  - supervises one simulation run at a time (tools/gs/runs.py, tools/gs/sil.py): the interactive Gazebo SIL, relaunched on a reboot
    request, or a HIL run (tools/sim/run_hil_s4.py), each in its own process group with a manifest JSON per run;
  - opens the SIL's UDP link once its port is bound (and keeps it across relaunches) and the link interface of a board that shows
    up under /dev/serial/by-id (autoconnect), and reports the host (load, free memory, the heavy-lock holder, ci/heavy.sh);
  - validates a staged config for an image of the library (tools/validate/validate_config.py, ground-segment unit 9, G7) and
    keeps the reports keyed by (image hash, config hash) (tools/gs/artifacts.py ReportLibrary);
  - flashes a board image through picotool (tools/gs/flash.py, ground-segment unit 15, G7): only with a passing validation report
    for the pair (image hash, the config the board will run); the helper checks that gate, runs picotool and opens the serial
    port again; the maintenance grant before it and the image hash read back after it are the GCS's (the link is its codec's);
  - serves the built GCS (gcs/build) at /.

  uv run python tools/gs/helper.py [--serial PATH [--baud N] | --udp PORT] [--port N] [--runs-dir DIR] [--build-root DIR]
                                   [--gcs-dir DIR] [--sil-udp-port N] [--sil-command CMD] [--reports-dir DIR]
                                   [--validate-command CMD] [--picotool PATH] [--extra-origin ORIGIN ...]
                                   [--hil-command CMD] [--heavy-lock FILE] [--byid-dir DIR] [--no-autoconnect] [--board-serial SERIAL]

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
         {"cmd": "sil_start"} {"cmd": "sil_stop"}: run_start / run_stop of kind sil
         {"cmd": "run_plan", "kind": "sil"|"hil", ...options}: the argv, run id and directory a start would use, and the reasons it
           would be refused ({"plan": {"kind", "run_id", "argv", "options", "out_dir", "udp_port", "allowed", "refusals"}}).
           The options of a hil run: "port" (required, under the by-id directory), "seconds", "expect" (s4a|late|none), "gui";
           a sil run has none; any other key is refused.
         {"cmd": "run_start", "kind": ..., ...options}: refused ("refusals": [...], ok false) while a run is going (one simulation
           at a time), a validation or flash runs, the heavy lock is held, or (hil) the helper has a serial port open or the port is
           absent; else starts the run and answers {"run": manifest}. {"cmd": "run_stop"}: stops the run's process group,
           answers {"run": manifest, "exit_status"}. {"cmd": "run_list"}: {"runs": [run id]}. {"cmd": "run_manifest", "run_id"}:
           {"manifest"} (the id must name a run under <run dir>/runs).
         {"cmd": "host_state"}: {"host": {"loadavg", "cpus", "mem_total_bytes", "mem_available_bytes", "heavy_lock": {"held",
           "holder", "path"}}}. {"cmd": "autoconnect", "enabled": bool}: {"autoconnect": {"enabled", "suspended", "owner"}}.
         {"cmd": "config_report", "image_hash": H, "view": V}: the config hash of V and whether a report of (H, that hash) is
           stored ({"config_hash", "stored", "verdict"})
         {"cmd": "validate", "image_hash": H, "view": V}: writes V as a .mcfg (tools/config/mcfg.py, the format's single
           source) and runs the validator on it for the library's image H, one validation at a time; answered at once
           ({"config_hash", "started": true}); the run pushes {"event": "validate_progress", "line"} per progress line and
           {"event": "validate_done", "image_hash", "config_hash", "verdict", "report_id", "exit_status"}.
         "view" may be replaced by "config": "factory" (the image's factory .mcfg) or a config hash the report store holds a
         config.mcfg of (for the flash gate: a config the vehicle does not stage here); a board image is validated with the SIL
         library of a build tree of the same parameter schema ("--build"), and its ELF is written as the .bin the validator takes.
         {"cmd": "flash_plan", "image_hash": H, "board": B}: what flashing H onto the board would run and whether the gate passes
           ({"plan": {"image", "config", "gate", "can_validate", "allowed", "refusal"}}); reads the report store only.
         {"cmd": "flash", "image_hash": H, "board": B}: plans again and refuses (ok false, "picotool_started": false) unless the
           plan is allowed; else answers {"started": true, "expected": {...}} at once and runs the flash, one at a time: the UF2
           is checked to hold exactly the image's flash bytes, the serial port is given up (event device_closed), picotool
           load -f -v -x runs, the port is opened again (event device_opened). Pushed: {"event": "flash_progress", "stage",
           "line"} and {"event": "flash_done", "image_hash", "ok", "stage", "error", "picotool_started", "picotool_status",
           "port_reopened", "expected"}. B = {"schema_hash": "0x..", "config_hash": H, "config_source": "factory"|"active",
           "armed": bool} from the vehicle's identity and heartbeat.
         V is the JSON view of tools/config/mcfg.py (decode/dump: sections of records with their bits, enumerations and source
         texts); the GCS builds it from the vehicle's staged records. Each command is answered {"ok": true|false, ...} (an
         "id" field is echoed). Pushed events: {"event": "device_opened"|"device_closed", ...}, the validation's and the flash's, and
         {"event": "run_started"|"run_stopped"|"run_exited", "run_id", "kind", ...}, {"event": "sil_relaunched", "run_id", "relaunches",
         "pid"}, {"event": "autoconnect", "state": "opened"|"closed"|"failed"|"suspended"|"resumed", "source": "sil"|"board",
         "device", "reason"} and {"event": "host_state", ...} (every few polls, at once when the heavy-lock holder changes).
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
import flash  # noqa: E402
import groundlog  # noqa: E402
import image_hash as board_image  # noqa: E402
import mcfg  # noqa: E402
from artifacts import ARTIFACT_KEYS, ImageLibrary, ReportLibrary  # noqa: E402
from device import DEFAULT_BAUD, LOOPBACK, Device, DeviceError, SerialDevice, UdpDevice  # noqa: E402
import hoststate  # noqa: E402
from runs import KINDS, RunRefused, RunSupervisor, parse_options  # noqa: E402
from sil import SilLauncher, udp_bound  # noqa: E402

from websockets.asyncio.server import ServerConnection, serve  # noqa: E402
from websockets.datastructures import Headers  # noqa: E402
from websockets.http11 import Request, Response  # noqa: E402

DEFAULT_PORT = 8765  # a convenience default for a stable URL (unprivileged, unassigned); --port 0 picks a free port
COOKIE = "marv_gs_token"
CLIENT_QUEUE_MAX_BYTES = 4 << 20  # labelled harness value: bytes queued for a slow browser before chunks are dropped (counted, logged)
SERIAL_GLOBS = ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/serial/by-id/*")
WS_BUSY = 1013  # RFC 6455 section 7.4.2 "try again later"
BYID_DIR = Path("/dev/serial/by-id")
# A board's link interface is opened only when all three hold. The by-id name is the Pico SDK's product string with the link CDC,
# USB interface 2 (fw/hal/target/CMakeLists.txt); sysfs says the device is 2e8a:0009 (Raspberry Pi's vendor ID and the Pico SDK CDC
# product ID: fw/prim/include/marv/prim/constants.hpp kUsbVendorIdRaspberryPi, kUsbProductIdPicoSdkCdc) and the interface's string
# is "MARV link" (tests/regression/quad/L10/results/usb/README.md, sysfs_after.txt). The HIL image has one CDC (not "MARV link").
BOARD_LINK_GLOB = "usb-Raspberry_Pi_Pico_*-if02"
BOARD_USB_ID = ("2e8a", "0009")
BOARD_LINK_INTERFACE = "MARV link"
SYSFS = Path("/sys")
POLL_S = 0.5  # labelled harness value: the autoconnect and host-state poll period; SIL UDP is opened within one poll of the bind
HOST_PUSH_POLLS = 4  # labelled harness value: a host_state event every this many polls (at once when the heavy-lock holder changes)
VALIDATOR = ROOT / "tools" / "validate" / "validate_config.py"
VALIDATION_STOP_S = 10  # labelled harness value: how long the helper waits for a stopped validation to end
# The validator's command line; {config} {image} {build} {work} {out} are replaced (--validate-command replaces it in tests).
VALIDATE_COMMAND = (sys.executable, str(VALIDATOR), "--config", "{config}", "--image", "{image}", "--build", "{build}",
                    "--work-dir", "{work}", "--out", "{out}")


def utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


class Refusal(DeviceError):
    """A command the helper refuses: `extra` is merged into the reply (the list of reasons)."""

    def __init__(self, message: str, extra: dict | None = None):
        super().__init__(message)
        self.extra = extra or {}


class Helper:
    def __init__(self, *, runs_dir: Path, build_root: Path, gcs_dir: Path, sil_udp_port: int | None, token: str | None = None,
                 sil_command: list[str] | None = None, extra_origins: tuple[str, ...] = (), reports_dir: Path | None = None,
                 validate_command: list[str] | None = None, picotool: str | None = None, hil_command: list[str] | None = None,
                 heavy_lock: Path | None = None, byid_dir: Path | None = None, autoconnect: bool = True, poll_s: float = POLL_S,
                 board_serial: str | None = None, sysfs_dir: Path | None = None):
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
        self.picotool = picotool or os.environ.get("MARV_PICOTOOL") or flash.PICOTOOL
        self.flash_state: dict | None = None
        self.flash_timing: dict = {}  # tests shorten flash.run_flash's waits (load_s, port_wait_s, settle_s)
        self._flash_task: asyncio.Task | None = None
        self.sil = SilLauncher(self.run_dir, sil_udp_port, sil_command)
        self.runs = RunSupervisor(self.run_dir, self.sil, self._emit_threadsafe, hil_command_override=hil_command)
        self.heavy_lock = Path(heavy_lock) if heavy_lock is not None else None  # None: $MARV_HEAVY_LOCK or the default (hoststate.py)
        self.byid_dir = Path(byid_dir) if byid_dir is not None else BYID_DIR
        self.autoconnect_enabled = autoconnect
        self.board_serial = board_serial  # when set, only the board with this USB serial string is opened
        self.sysfs_dir = Path(sysfs_dir) if sysfs_dir is not None else SYSFS
        self.poll_s = poll_s
        self._ac_owner: str | None = None  # "sil" or "board" while the open device was opened by autoconnect
        self._ac_key: str | None = None
        self._ac_declined: str | None = None  # the key whose device the user closed: not opened again until the key changes
        self._ac_suspended: str | None = None
        self._ac_failed: dict = {}
        self._host_last: dict | None = None
        self._monitor_task: asyncio.Task | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self.port = 0
        self.device: Device | None = None
        self.client: ServerConnection | None = None
        self._queue: asyncio.Queue | None = None
        self._queued_bytes = 0
        self.dropped_bytes = 0
        self._server = None

    # ---- lifecycle ------------------------------------------------------------------------------------------------------

    async def start(self, port: int) -> None:
        self._loop = asyncio.get_running_loop()
        self._server = await serve(self._ws_handler, LOOPBACK, port, process_request=self._process_request, compression=None)
        self.port = self._server.sockets[0].getsockname()[1]
        self.log.event(f"helper started on {self.origin}")
        self._monitor_task = asyncio.create_task(self._monitor())

    @property
    def origin(self) -> str:
        return f"http://{LOOPBACK}:{self.port}"

    def url(self) -> str:
        return f"{self.origin}/?token={self.token}"

    async def stop(self) -> None:
        if self._monitor_task is not None:
            self._monitor_task.cancel()
            await asyncio.gather(self._monitor_task, return_exceptions=True)
            self._monitor_task = None
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
        self.close_device("helper stopping")
        await self.stop_validation()
        await self.stop_flash()
        await asyncio.to_thread(self.runs.shutdown)
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
        self._ac_owner = self._ac_key = None
        self._push(json.dumps({"event": "device_closed", "reason": reason}))

    def _emit_threadsafe(self, event: dict) -> None:
        """Logs and pushes an event from any thread (the run watcher's)."""
        loop = self._loop
        if loop is None or loop.is_closed():
            return

        def deliver() -> None:
            self.log.event(f"{event['event']}: " + json.dumps({k: v for k, v in event.items() if k != "event"}))
            self._push(json.dumps(event))
        try:
            loop.call_soon_threadsafe(deliver)
        except RuntimeError:  # the loop closed between the check and the call
            pass

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

    def close_device(self, reason: str = "closed by request", *, by_user: bool = False) -> None:
        if self.device is not None:
            self.device.close()
            self.log.event(f"device closed: {reason}")
            self.device = None
            self._push(json.dumps({"event": "device_closed", "reason": reason}))
        if by_user and self._ac_owner is not None:
            self._ac_declined = self._ac_key
        self._ac_owner = self._ac_key = None

    async def open_serial(self, path: str, baud: int, *, from_browser: bool) -> None:
        if from_browser and not (os.path.normpath(path) == path and any(fnmatch.fnmatchcase(path, g) for g in SERIAL_GLOBS)
                                 and os.path.realpath(path).startswith("/dev/")):
            raise DeviceError("a serial path from the browser must be a tty device: " + ", ".join(SERIAL_GLOBS))
        self._require_no_device()
        if self.runs.active() is not None and self.runs.active()["kind"] == "hil":
            raise DeviceError("a HIL run is going: it owns the board port; stop the run first (H-1)")
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

    # ---- runs (H-1 to H-3) ----------------------------------------------------------------------------------------------

    def run_options(self, msg: dict) -> tuple[str, dict]:
        kind = msg.get("kind", "sil")
        try:
            return kind, parse_options(kind, msg, device_globs=(f"{self.byid_dir}/*",))
        except (ValueError, TypeError) as e:
            raise DeviceError(str(e)) from None

    def run_refusals(self, kind: str, opts: dict) -> list[str]:
        """Every reason a run of this kind cannot start now; empty when it can."""
        reasons = []
        busy = self.runs.busy()
        if busy is not None:
            reasons.append(f"run {busy} is still going: one simulation at a time (H-1)")
        if self._validation_task is not None and not self._validation_task.done():
            reasons.append("a validation is running; wait for it to finish")
        if self._flash_task is not None and not self._flash_task.done():
            reasons.append("a flash is running; wait for it to finish")
        lock = hoststate.heavy_lock_state(self.heavy_lock)
        if lock["held"]:
            reasons.append(f"the heavy lock is held ({lock['holder'] or 'unknown holder'}): heavy work waits for it (H-3)")
        if kind == "hil":
            if isinstance(self.device, SerialDevice):
                reasons.append(f"the helper has a serial port open ({self.device.path}): close it first, a HIL run owns the board "
                               "port (H-1)")
            if not os.path.exists(opts["port"]):
                reasons.append(f"the board port {opts['port']} is not present")
        return reasons

    def run_plan(self, msg: dict) -> dict:
        kind, opts = self.run_options(msg)
        plan = self.runs.plan(kind, opts)
        refusals = self.run_refusals(kind, opts)
        return {**plan, "allowed": not refusals, "refusals": refusals}

    def run_start(self, msg: dict) -> dict:
        kind, opts = self.run_options(msg)
        refusals = self.run_refusals(kind, opts)
        if refusals:
            self.log.event(f"run refused ({kind}): {'; '.join(refusals)}")
            raise Refusal(refusals[0], {"refusals": refusals})
        try:
            manifest = self.runs.start(kind, opts)
        except RunRefused as e:
            raise Refusal(str(e), {"refusals": [str(e)]}) from None
        return {"run": manifest}

    def run_stop(self) -> dict:
        try:
            manifest = self.runs.stop()
        except RunRefused as e:
            raise DeviceError(str(e)) from None
        return {"run": manifest, "exit_status": manifest["exit_status"]}

    # ---- autoconnect and host state -------------------------------------------------------------------------------------

    def autoconnect_status(self) -> dict:
        return {"enabled": self.autoconnect_enabled, "suspended": self._ac_suspended, "owner": self._ac_owner,
                "byid_dir": str(self.byid_dir)}

    def _ac_event(self, state: str, **fields) -> None:
        event = {"event": "autoconnect", "state": state, **fields}
        self.log.event("autoconnect " + json.dumps({k: v for k, v in event.items() if k != "event"}))
        self._push(json.dumps(event))

    def _read_sysfs(self, path: Path) -> str | None:
        try:
            return path.read_text(encoding="utf-8", errors="replace").strip()
        except OSError:
            return None

    def board_links(self) -> list[str]:
        """The by-id entries of a board's link interface, sorted. An entry is listed only when its name is BOARD_LINK_GLOB (and, with
        --board-serial, carries that serial), its target is a tty device node under /dev, and sysfs gives that tty's USB device
        BOARD_USB_ID and its interface the string BOARD_LINK_INTERFACE. Anything else (another vendor's composite device, a Pico
        running other firmware, the HIL image's single CDC, the console interface) is never listed."""
        try:
            names = sorted(os.listdir(self.byid_dir))
        except OSError:
            return []
        out = []
        for name in names:
            if not fnmatch.fnmatchcase(name, BOARD_LINK_GLOB):
                continue
            if self.board_serial is not None and name != f"usb-Raspberry_Pi_Pico_{self.board_serial}-if02":
                continue
            path = os.path.join(self.byid_dir, name)
            real = os.path.realpath(path)
            if not (real.startswith("/dev/") and os.path.exists(real)):
                continue
            interface = Path(os.path.realpath(self.sysfs_dir / "class" / "tty" / os.path.basename(real) / "device"))
            usb_id = (self._read_sysfs(interface.parent / "idVendor"), self._read_sysfs(interface.parent / "idProduct"))
            if usb_id == BOARD_USB_ID and self._read_sysfs(interface / "interface") == BOARD_LINK_INTERFACE:
                out.append(path)
        return out

    async def _monitor(self) -> None:
        polls = 0
        while True:
            try:
                await self.poll()
            except asyncio.CancelledError:
                raise
            except Exception as e:  # a failed poll must not end the monitor
                self.log.event(f"monitor poll failed: {type(e).__name__}: {e}")
            polls += 1
            host = hoststate.host_state(self.heavy_lock)
            if self._host_last is None or polls % HOST_PUSH_POLLS == 0 or host["heavy_lock"] != self._host_last["heavy_lock"]:
                self._push(json.dumps({"event": "host_state", **host}))
            self._host_last = host
            await asyncio.sleep(self.poll_s)

    async def poll(self) -> None:
        """One autoconnect step: the SIL's UDP link once its port is bound, else a board's link interface; both suspended while a
        HIL run or a flash owns the board."""
        active = self.runs.active()
        suspend = None
        if active is not None and active["kind"] == "hil":
            suspend = "a HIL run owns the board"
        elif self._flash_task is not None and not self._flash_task.done():
            suspend = "a flash is running"
        if suspend != self._ac_suspended:
            self._ac_suspended = suspend
            self._ac_event("suspended" if suspend else "resumed", reason=suspend)
        if not self.autoconnect_enabled or suspend:
            return
        sil_run = active is not None and active["kind"] == "sil"
        if self._ac_owner == "sil" and not sil_run:
            self.close_device("the SIL run ended")
            self._ac_event("closed", source="sil", reason="the SIL run ended")
        if self.device is not None:
            return
        port = self.sil.udp_port
        if sil_run and port is not None:
            key = f"sil:{active['id']}"
            if key != self._ac_declined and udp_bound(port):
                try:
                    await self.open_udp(port)
                except DeviceError as e:
                    self._ac_failure("sil", str(e))
                else:
                    self._ac_owner, self._ac_key, self._ac_failed = "sil", key, {}
                    self._ac_event("opened", source="sil", device=self.device.description)
                return
        links = self.board_links()
        if self._ac_declined is not None and self._ac_declined.startswith("board:") and self._ac_declined[len("board:"):] not in links:
            self._ac_declined = None
        for path in links:
            key = f"board:{path}"
            if key == self._ac_declined:
                continue
            try:
                await self.open_serial(path, DEFAULT_BAUD, from_browser=False)
            except DeviceError as e:
                self._ac_failure("board", str(e), path)
                continue
            self._ac_owner, self._ac_key, self._ac_failed = "board", key, {}
            self._ac_event("opened", source="board", device=self.device.description)
            return
        self._ac_failed = {k: v for k, v in self._ac_failed.items() if k in links or k == "sil"}

    def _ac_failure(self, source: str, reason: str, path: str = "") -> None:
        if self._ac_failed.get(path or source) != reason:
            self._ac_failed[path or source] = reason
            self._ac_event("failed", source=source, reason=reason)

    # ---- status ---------------------------------------------------------------------------------------------------------

    def status(self) -> dict:
        return {
            "origin": self.origin,
            "run_dir": str(self.run_dir),
            "device": self.device.description if self.device else None,
            "client": self.client is not None,
            "dropped_bytes": self.dropped_bytes,
            "sil": self.sil.status(),
            "runs": self.runs.status(),
            "host": hoststate.host_state(self.heavy_lock),
            "autoconnect": self.autoconnect_status(),
            "validation": self.validation,
            "flash": self.flash_state,
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

    def validation_inputs(self, image_hash: str) -> tuple[Path, Path, dict]:
        """(the image file the validator takes, the build tree it runs in, the library entry). A SIL image validates in its own tree. A
        board image is validated as its .bin (written from the ELF when the tree has none: the report's image hash is the board's)
        in the tree of a SIL image of the same parameter schema, which holds the SIL library and the table tool."""
        listing = self.images.scan()["images"]
        entry = next((e for e in listing if e["hash"] == image_hash), None)
        if entry is None:
            raise DeviceError(f"the image {image_hash!r} is not in the image library")
        image = (self.build_root / entry["artifacts"]["image"]).resolve()
        if entry["kind"] == "sil":
            return image, self.build_root / entry["tree"], entry
        schema = mcfg.manifest_schema_hash(mcfg.load_manifest(self.build_root / entry["artifacts"]["params_manifest"])) \
            if "params_manifest" in entry["artifacts"] else None
        tree = None
        for other in listing:
            if other["kind"] == "sil" and other["params_set"] == entry["params_set"] and "params_manifest" in other["artifacts"] \
                    and mcfg.manifest_schema_hash(mcfg.load_manifest(self.build_root / other["artifacts"]["params_manifest"])) == schema:
                tree = self.build_root / other["tree"]
                break
        if tree is None:
            raise DeviceError("no build tree with a SIL image of this board image's parameter schema: build the host-gz-quadfc preset")
        if image.suffix != ".bin":
            image_dir = self.run_dir / "images"
            image_dir.mkdir(exist_ok=True)
            image = image_dir / f"{image_hash}.bin"
            image.write_bytes(board_image.flash_image(self.build_root / entry["artifacts"]["image"]).data)
        return image, tree, entry

    def config_for_validation(self, msg: dict, image_hash: str, entry: dict) -> tuple[bytes, str]:
        """(the .mcfg bytes, the config hash hex) of the "view" or of the "config" reference of a validate message."""
        ref = msg.get("config")
        if ref is None:
            return self.config_of_view(msg.get("view"))
        if "params_manifest" not in entry["artifacts"]:
            raise DeviceError("the image has no parameter manifest")
        schema = mcfg.manifest_schema_hash(mcfg.load_manifest(self.build_root / entry["artifacts"]["params_manifest"]))
        if ref == "factory":
            path = self.build_root / entry["artifacts"]["factory_mcfg"] if "factory_mcfg" in entry["artifacts"] else None
        else:
            path = self.reports.find_config(str(ref))
        if path is None:
            raise DeviceError(f"the helper holds no copy of config {str(ref)[:16]}: stage and validate it from the Parameters page")
        try:
            data = path.read_bytes()
            return data, flash.config_hash_of(mcfg.decode(data), schema)
        except (OSError, mcfg.McfgError, flash.FlashRefused) as e:
            raise DeviceError(f"the config cannot be used: {e}") from None

    async def start_validation(self, msg: dict) -> dict:
        if self._validation_task is not None and not self._validation_task.done():
            raise DeviceError("a validation is running; wait for it to finish")
        image_hash = str(msg.get("image_hash", ""))
        image, tree, entry = self.validation_inputs(image_hash)
        data, config_hash = self.config_for_validation(msg, image_hash, entry)
        target = self.reports.directory(image_hash, config_hash)
        if target is None:
            raise DeviceError(f"the image {image_hash!r} is not in the image library")
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

    # ---- flashing (G7) --------------------------------------------------------------------------------------------------

    def flash_plan(self, msg: dict) -> dict:
        try:
            return flash.make_plan(self.images, self.reports, str(msg.get("image_hash", "")), msg.get("board"))
        except flash.FlashRefused as e:
            raise DeviceError(str(e)) from None

    async def start_flash(self, msg: dict) -> dict:
        """Plans again from the stored reports, refuses unless the plan is allowed (nothing has been touched then), else starts the
        flash and answers at once."""
        if self._flash_task is not None and not self._flash_task.done():
            raise DeviceError("a flash is running; wait for it to finish")
        if self._validation_task is not None and not self._validation_task.done():
            raise DeviceError("a validation is running; wait for it to finish")
        plan = self.flash_plan(msg)
        image_hash = plan["image"]["hash"]
        if not plan["allowed"]:
            self.log.event(f"flash refused before picotool: image {image_hash} config {plan['config']['hash']}: {plan['refusal']}")
            raise DeviceError(f"not flashed, picotool was not started: {plan['refusal']}")
        if not isinstance(self.device, SerialDevice):
            raise DeviceError("not flashed, picotool was not started: the board's serial port is not open here (Connection page)")
        entry = flash.board_entry(self.images, image_hash)
        expected = {"image_hash": image_hash, "config_hash": plan["config"]["hash"], "config_source": plan["config"]["expected_source"]}
        self.flash_state = {"running": True, "image_hash": image_hash, "expected": expected}
        self.log.event(f"flash started: image {image_hash} config {plan['config']['hash']} ({plan['config']['which']}), "
                       f"report {plan['gate']['report_id']}")
        self._flash_task = asyncio.create_task(self._run_flash(plan, entry))
        return {"started": True, "expected": expected, "plan": plan}

    async def _run_flash(self, plan: dict, entry: dict) -> None:
        image_hash = plan["image"]["hash"]

        def progress(stage: str, line: str) -> None:
            self.log.event(f"flash {stage}: {line}")
            self._push(json.dumps({"event": "flash_progress", "image_hash": image_hash, "stage": stage, "line": line}))

        def release() -> tuple[str, int]:
            if not isinstance(self.device, SerialDevice):
                raise flash.FlashRefused("the serial port is no longer open")
            path, baud = self.device.path, self.device.baud
            self.close_device("flashing")
            return path, baud

        async def reopen(path: str, baud: int) -> None:
            await self.open_serial(path, baud, from_browser=False)

        try:
            result = await flash.run_flash(plan, entry, self.build_root, self.picotool, self.run_dir / "flash",
                                           flash.Hooks(progress, release, reopen), **self.flash_timing)
        except asyncio.CancelledError:
            raise
        except Exception as e:  # a failed flash must not take the helper down
            result = {"ok": False, "stage": "internal", "error": f"{type(e).__name__}: {e}", "picotool_started": False,
                      "picotool_status": None, "port_reopened": False, "expected": self.flash_state["expected"] if self.flash_state else None}
        self.flash_state = {"running": False, "image_hash": image_hash, **result}
        self.log.event(f"flash done: image {image_hash} ok {result['ok']} stage {result['stage']} error {result['error']}")
        self._push(json.dumps({"event": "flash_done", "image_hash": image_hash, **result}))

    async def stop_flash(self) -> None:
        if self._flash_task is not None and not self._flash_task.done():
            self._flash_task.cancel()
            try:
                await self._flash_task
            except (asyncio.CancelledError, Exception):
                pass

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
                self.close_device(by_user=True)
            elif cmd == "sil_start":
                reply.update(self.run_start({"cmd": "run_start", "kind": "sil"}))
            elif cmd == "sil_stop":
                active = self.runs.active()
                if active is not None and active["kind"] == "sil":
                    reply.update(await asyncio.to_thread(self.run_stop))
                else:
                    reply["exit_status"] = await asyncio.to_thread(self.sil.stop)
            elif cmd == "run_plan":
                reply["plan"] = self.run_plan(msg)
            elif cmd == "run_start":
                reply.update(self.run_start(msg))
            elif cmd == "run_stop":
                reply.update(await asyncio.to_thread(self.run_stop))
            elif cmd == "run_list":
                reply["runs"] = self.runs.list_runs()
            elif cmd == "run_manifest":
                reply["manifest"] = self.runs.manifest(msg.get("run_id"))
            elif cmd == "host_state":
                reply["host"] = hoststate.host_state(self.heavy_lock)
            elif cmd == "autoconnect":
                if not isinstance(msg.get("enabled"), bool):
                    raise DeviceError("autoconnect needs enabled: true or false")
                self.autoconnect_enabled = msg["enabled"]
                self._ac_declined = None
                reply["autoconnect"] = self.autoconnect_status()
            elif cmd == "config_report":
                reply.update(self.config_report(msg))
            elif cmd == "validate":
                reply.update(await self.start_validation(msg))
            elif cmd == "flash_plan":
                reply["plan"] = self.flash_plan(msg)
            elif cmd == "flash":
                reply.update(await self.start_flash(msg))
            else:
                raise DeviceError(f"unknown command {cmd!r}")
        except (DeviceError, RuntimeError, ValueError, TypeError, OSError) as e:
            reply = {"ok": False, "error": str(e), **getattr(e, "extra", {}), **({"id": msg["id"]} if "id" in msg else {})}
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
                    validate_command=shlex.split(args.validate_command) if args.validate_command else None,
                    picotool=args.picotool, hil_command=shlex.split(args.hil_command) if args.hil_command else None,
                    heavy_lock=args.heavy_lock, byid_dir=args.byid_dir, autoconnect=not args.no_autoconnect,
                    board_serial=args.board_serial)
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
    ap.add_argument("--hil-command", help="the command line a `run_start` of kind hil runs instead of tools/sim/run_hil_s4.py "
                                          "(shell-style quoting; for tests)")
    ap.add_argument("--heavy-lock", type=Path, default=None,
                    help="the heavy-lock file (default $MARV_HEAVY_LOCK, else %s; ci/heavy.sh takes the same lock)" % hoststate.HEAVY_LOCK_DEFAULT)
    ap.add_argument("--byid-dir", type=Path, default=None,
                    help=f"the directory the board watcher scans for link interfaces (default {BYID_DIR}); also the root of a HIL run's port")
    ap.add_argument("--board-serial", default=None, metavar="SERIAL",
                    help="autoconnect opens only the board whose USB serial string is SERIAL (default: any board that passes the checks)")
    ap.add_argument("--no-autoconnect", action="store_true", help="do not open the SIL's UDP link or a board's link interface by itself")
    ap.add_argument("--picotool", help=f"the picotool executable the flash runs (default $MARV_PICOTOOL, else `{flash.PICOTOOL}` on PATH)")
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
