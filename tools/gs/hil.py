"""HIL from the GCS (ground-segment unit GX-10): the preflight of an S4b run, the launch, the verdict form and the SWD flash of the
bench images. The helper's control commands hil_preflight, hil_hold, hil_start, hil_verdict and flash_swd are answered here;
tools/gs/helper.py only dispatches them and asks this module whether autoconnect is held.

Preflight (hil_preflight, and again inside hil_start): a launch is refused, each reason named, unless all six hold:
  probe        a Raspberry Pi Debug Probe is on USB with the serial MARV_PROBE_SERIAL (the probe tools/target/flash.sh and
               tools/sim/run_hil_s4.py reboot the board with)
  board        a board (USB 2e8a:0009) is on USB
  hil_image    the board's serial port shows the HIL image's USB descriptors: the Pico SDK's single CDC, interface string "Board CDC"
               (every other image of the board has the composite device of fw/hal/target/src/usb.cpp, "MARV console"). This is read
               from sysfs, with no contact with the board; the plugin's own checks of the board's hello and receiver-boot record
               (sim/gz/plugin/src/hil_source.cpp) stay the authority on the exact image
  esc_battery  the message carries esc_battery_unplugged: true (the operator's acknowledgement, given again for every launch)
  host_idle    no run, validation or flash is going and the heavy lock is free (Helper.idle_refusals: H-1, H-3)
  ports        the helper has no serial port open, no other process holds the board's port (flock, read from /proc/locks without
               opening the port), and autoconnect is suspended or off (hil_hold closes the helper's serial port and suspends it)
hil_start runs the preflight, then tools/sim/run_hil_s4.py --port <port> --expect none --gui as a supervised run (tools/gs/runs.py:
its own process group, a manifest, H-2) and never builds (H-3). Autoconnect stays suspended while the run lives (Helper.poll) and the
hold ends with the run ("restored after it").

Verdict (hil_verdict): the flight mode flown (ANGLE or ACRO) and free text, written once to <run dir>/runs/<run id>/out/verdict.json,
next to the run's *.hil_report.txt.

SWD flash (flash_swd): an image kind (hil, bench-log, bench-profile) flashed with tools/target/flash.sh through the debug probe,
outside the G7 gate (bench images never fly). Only the hil kind, which drives no motor pad, is flashed from the GCS: the motor-capable
kinds are refused. The ELF is the one a build left in the fixed tree of the kind; it is never built here (H-3).
"""

from __future__ import annotations

import asyncio
import fcntl
import hashlib
import json
import os
import re
import signal
import time
from pathlib import Path

from device import DeviceError, SerialDevice
from runs import resolve_under, utc_iso

# USB identities. The board's and the probe's were read from this host's sysfs (/sys/bus/usb/devices, 2026-10-19): the board 2e8a:0009
# "Pico" (also fw/prim/include/marv/prim/constants.hpp kUsbVendorIdRaspberryPi, kUsbProductIdPicoSdkCdc; helper.py BOARD_USB_ID), the
# probe 2e8a:000c "Debugprobe on Pico (CMSIS-DAP)", serial 900678EF479C3A0E (the MARV_PROBE_SERIAL of the bench READMEs).
BOARD_USB_ID = ("2e8a", "0009")
PROBE_USB_ID = ("2e8a", "000c")
# The by-id name of a board's interface 0 (the Pico SDK's product string "Pico" and the interface's number).
BOARD_PORT_GLOB = "usb-Raspberry_Pi_Pico_*-if00"
BOARD_NAME_PREFIX = "usb-Raspberry_Pi_Pico_"
# The interface string of the SDK's default descriptors, which the HIL images keep (fw/hal/target/CMakeLists.txt: "every image but the
# two HIL ones" has its own); read for the older single-CDC images in tests/regression/quad/L10/results/usb/README.md (a).
# INFERRED for the HIL receiver image until a run of this preflight on the flashed board shows it.
HIL_INTERFACE = "Board CDC"
PROC_LOCKS = Path("/proc/locks")
SYS_BUS_USB = ("bus", "usb", "devices")

VERDICT_MODES = ("ANGLE", "ACRO")
VERDICT_FILE = "verdict.json"
VERDICT_SCHEMA = 1
# PLACEHOLDER (harness value): bounds the verdict form's free text so that a request cannot fill the disk; decides no result.
VERDICT_TEXT_MAX = 4000
# The S4b length: tools/sim/run_hil_s4.py --seconds 300 in the S4b runbook (tests/regression/quad/L10/results/s4/README.md). A scenario value.
S4B_SECONDS = 300.0
HIL_OPTIONS = ("esc_battery_unplugged", "port", "seconds")

MOTOR_CAPABLE_REFUSAL = ("only after its props-off TB on the exact image and Luis's presence acknowledgement: {kind} is a motor-capable "
                         "bench image (it links the flight composition's motor outputs), so it is not flashed from the GCS in this unit")
# Bench image kinds. A kind flashed from the GCS names the tree a build left its ELF in (the helper never builds) and the CMake cache
# entries that make that tree the kind's image; a motor-capable kind has no tree here: it is refused (BENCH_KINDS).
HIL_TREE = "m33-hil"  # `cmake --preset m33 -B build/m33-hil -DMARV_FC3V2_HIL=ON -DMARV_FC3V2_HIL_RECEIVER=ON` (tools/sim/run_hil_s4.py)
HIL_ELF = Path("fw") / "hal" / "target" / "marv_fc3v2.elf"
HIL_CACHE = {"MARV_FC3V2_HIL": "ON", "MARV_FC3V2_HIL_RECEIVER": "ON"}
BENCH_KINDS = {"hil": False, "bench-log": True, "bench-profile": True}  # kind -> motor-capable
SWD_SCRIPT = Path(__file__).resolve().parents[2] / "tools" / "target" / "flash.sh"
# PLACEHOLDER (harness value): bounds a hung flash.sh (OpenOCD program + verify + reboot took about 14 s on the bench, tests/regression/
# quad/L10/results/telemetry/README.md); tools/sim/run_hil_s4.py allows the same 120 s for flash.sh --probe-only. Decides no result.
SWD_TIMEOUT_S = 120.0
# PLACEHOLDER (harness value): how long a stopped flash.sh gets after SIGTERM before SIGKILL.
SWD_STOP_S = 10.0
# PLACEHOLDER (harness value): the last lines of flash.sh's output the status keeps for the GCS; the whole output is in the ground log.
SWD_TAIL_LINES = 40


def _read(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return None


def usb_devices(sysfs_dir: Path) -> list[dict]:
    """The USB devices sysfs lists: [{"vendor", "product", "serial"}] (the strings of idVendor, idProduct and serial)."""
    root = Path(sysfs_dir).joinpath(*SYS_BUS_USB)
    out = []
    try:
        entries = sorted(root.iterdir())
    except OSError:
        return []
    for d in entries:
        vendor, product = _read(d / "idVendor"), _read(d / "idProduct")
        if vendor is not None and product is not None:
            out.append({"vendor": vendor, "product": product, "serial": _read(d / "serial")})
    return out


def probe_check(sysfs_dir: Path, serial: str | None) -> tuple[bool, str]:
    if not serial:
        return False, ("no debug probe: MARV_PROBE_SERIAL is not set in the helper's environment (tools/target/flash.sh and "
                       "tools/sim/run_hil_s4.py reboot the board through the probe it names)")
    for d in usb_devices(sysfs_dir):
        if (d["vendor"], d["product"]) == PROBE_USB_ID and d["serial"] == serial:
            return True, f"debug probe {serial} is on USB"
    return False, f"no debug probe: no USB device {':'.join(PROBE_USB_ID)} with serial {serial} (plug the Debug Probe in, or fix MARV_PROBE_SERIAL)"


def boards_on_usb(sysfs_dir: Path, serial: str | None) -> list[str]:
    """The serial strings of the boards on USB (with `serial`, only that board)."""
    found = [d["serial"] or "" for d in usb_devices(sysfs_dir) if (d["vendor"], d["product"]) == BOARD_USB_ID]
    return [s for s in found if serial is None or s == serial]


def board_ports(byid_dir: Path, sysfs_dir: Path, serial: str | None) -> list[dict]:
    """The by-id entries of a board's interface 0: [{"path", "interface"}], sorted by path. An entry is listed when its target is a device
    under /dev and sysfs gives its USB device the board's identity and the interface its string (anything sysfs cannot vouch for is not
    listed)."""
    try:
        names = sorted(os.listdir(byid_dir))
    except OSError:
        return []
    out = []
    for name in names:
        if not (name.startswith(BOARD_NAME_PREFIX) and name.endswith("-if00")):
            continue
        if serial is not None and name != f"{BOARD_NAME_PREFIX}{serial}-if00":
            continue
        path = os.path.join(byid_dir, name)
        real = os.path.realpath(path)
        if not (real.startswith("/dev/") and os.path.exists(real)):
            continue
        interface = Path(os.path.realpath(Path(sysfs_dir) / "class" / "tty" / os.path.basename(real) / "device"))
        usb_id = (_read(interface.parent / "idVendor"), _read(interface.parent / "idProduct"))
        text = _read(interface / "interface")
        if usb_id == BOARD_USB_ID and text is not None:
            out.append({"path": path, "interface": text})
    return out


def flock_holders(device_path: str, locks: Path = PROC_LOCKS) -> list[int]:
    """The pids that hold an flock(2) lock on the device node (the kernel's /proc/locks; the port is not opened). [] where the table
    cannot be read."""
    try:
        st = os.stat(os.path.realpath(device_path))
        rows = locks.read_text(encoding="ascii").splitlines()
    except OSError:
        return []
    want = f"{os.major(st.st_dev):02x}:{os.minor(st.st_dev):02x}:{st.st_ino}"
    pids = []
    for row in rows:
        f = row.split()
        if len(f) > 5 and f[1] == "FLOCK" and f[5] == want:
            pids.append(int(f[4]))
    return pids


def cache_values(tree: Path) -> dict[str, str]:
    """NAME -> value of the CMake cache of a build tree (`NAME:TYPE=value` lines); {} when it has none."""
    text = _read(tree / "CMakeCache.txt")
    if text is None:
        return {}
    return {m.group(1): m.group(2) for m in re.finditer(r"^([A-Za-z0-9_]+):[A-Z]+=(.*)$", text, re.M)}


class HilGcs:
    """The GCS's HIL commands over a Helper (duck-typed: run_options, run_start, idle_refusals, runs, device, autoconnect_enabled,
    byid_dir, sysfs_dir, board_serial, build_root, log, _push, close_device)."""

    def __init__(self, helper, swd_script: Path | None = None):
        self.h = helper
        self.hold = False  # autoconnect and the board's ports are held for a HIL run
        self.hold_run: str | None = None  # the run started under the hold; the hold ends with it
        self.swd_script = Path(swd_script) if swd_script is not None else SWD_SCRIPT
        self.swd: dict | None = None
        self._swd_task: asyncio.Task | None = None
        self._swd_proc: asyncio.subprocess.Process | None = None

    # ---- the hold -------------------------------------------------------------------------------------------------------------

    def suspension(self, active: dict | None) -> str | None:
        """Why autoconnect is held now (None: not held), called by Helper.poll each step. A hold taken for a run ends when that run
        is no longer the active one."""
        if self.hold and self.hold_run is not None and (active is None or active["id"] != self.hold_run):
            self.hold, self.hold_run = False, None
            self.h.log.event("HIL hold released: the run ended")
        return "the board is held for a HIL run (hil_hold)" if self.hold else None

    def set_hold(self, msg: dict) -> dict:
        if set(msg) - {"cmd", "id", "on"} or not isinstance(msg.get("on"), bool):
            raise DeviceError("hil_hold needs on: true or false")
        if msg["on"]:
            if isinstance(self.h.device, SerialDevice):
                self.h.close_device("held for a HIL run", by_user=True)
            self.hold = True
        else:
            self.hold, self.hold_run = False, None
        self.h.log.event(f"HIL hold {'taken' if self.hold else 'released'}")
        return {"hold": self.hold}

    # ---- preflight ------------------------------------------------------------------------------------------------------------

    def options(self, msg: dict) -> tuple[bool, str | None, float]:
        unknown = sorted(set(msg) - {"cmd", "id"} - set(HIL_OPTIONS))
        if unknown:
            raise DeviceError(f"not an option of a HIL launch: {', '.join(unknown)}")
        ack = msg.get("esc_battery_unplugged", False)
        if not isinstance(ack, bool):
            raise DeviceError("esc_battery_unplugged must be true or false")
        port = msg.get("port")
        if port is not None and not isinstance(port, str):
            raise DeviceError("port must be a device path")
        seconds = msg.get("seconds", S4B_SECONDS)
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or seconds <= 0 or seconds != seconds or seconds == float("inf"):
            raise DeviceError("seconds must be a positive finite number")
        return ack, port, float(seconds)

    def preflight(self, msg: dict) -> dict:
        """{"allowed", "refusals", "checks": [{"id", "state": ok|refused|skipped, "detail"}], "port", "seconds", "argv"}. Starts nothing
        and touches no device."""
        h = self.h
        ack, want_port, seconds = self.options(msg)
        checks: list[dict] = []

        def add(check_id: str, reasons: list[str], ok_detail: str, skipped: str | None = None) -> None:
            if skipped is not None:
                checks.append({"id": check_id, "state": "skipped", "detail": skipped, "reasons": []})
            else:
                checks.append({"id": check_id, "state": "refused" if reasons else "ok", "detail": "; ".join(reasons) or ok_detail,
                               "reasons": reasons})

        ok, detail = probe_check(h.sysfs_dir, os.environ.get("MARV_PROBE_SERIAL"))
        add("probe", [] if ok else [detail], detail)

        boards = boards_on_usb(h.sysfs_dir, h.board_serial)
        add("board", [] if boards else ["no board: no USB device " + ":".join(BOARD_USB_ID) + (f" with serial {h.board_serial}" if h.board_serial else "")
                                        + " (plug the board in)"],
            f"board {', '.join(boards)} is on USB")

        port = None
        if not boards:
            add("hil_image", [], "", skipped="no board to look at")
        else:
            ports = board_ports(h.byid_dir, h.sysfs_dir, h.board_serial)
            hil_ports = [p for p in ports if p["interface"] == HIL_INTERFACE]
            reasons: list[str] = []
            if want_port is not None:
                chosen = [p for p in hil_ports if p["path"] == want_port]
                if not chosen:
                    reasons.append(f"the port {want_port} is not a board port running the HIL image")
                else:
                    port = chosen[0]["path"]
            elif len(hil_ports) == 1:
                port = hil_ports[0]["path"]
            elif len(hil_ports) > 1:
                reasons.append("several board ports run the HIL image: name the port")
            if not hil_ports and not reasons:
                shown = ", ".join(f"'{p['interface']}'" for p in ports) or "no serial port under " + str(h.byid_dir)
                reasons.append(f"the board is not running the HIL image: its interface 0 reads {shown}, not '{HIL_INTERFACE}' "
                               "(flash the HIL image first)")
            add("hil_image", reasons, f"the board's port {port} shows the HIL image's descriptors")

        add("esc_battery", [] if ack else ["the ESC battery acknowledgement is not ticked: unplug the ESC battery, then tick it"],
            "acknowledged: the ESC battery is unplugged")

        add("host_idle", h.idle_refusals(), "no run, validation or flash is going and the heavy lock is free")

        reasons = []
        if isinstance(h.device, SerialDevice):
            reasons.append(f"the helper has a serial port open ({h.device.path}): close it, a HIL run owns the board's port (H-1)")
        if port is not None and not (isinstance(h.device, SerialDevice) and os.path.realpath(h.device.path) == os.path.realpath(port)):
            holders = flock_holders(port)
            if holders:
                reasons.append(f"another process holds the board's port {port} (pid {', '.join(str(p) for p in holders)}): close it, "
                               "a HIL run owns the port (H-1)")
        if h.autoconnect_enabled and not self.hold and not h._ac_suspended:
            reasons.append("autoconnect is not suspended: it would open the board's ports again (take the hold)")
        add("ports", reasons, "the board's ports are closed and autoconnect is suspended")

        refusals = [r for c in checks for r in c["reasons"]]
        argv = None
        if port is not None:
            try:
                kind, opts = h.run_options({"cmd": "run_plan", "kind": "hil", "port": port, "seconds": seconds, "expect": "none", "gui": True})
                argv = h.runs.plan(kind, opts)["argv"]
            except DeviceError:
                argv = None
        return {"allowed": not refusals, "refusals": refusals, "checks": [{k: v for k, v in c.items() if k != "reasons"} for c in checks],
                "port": port, "seconds": seconds, "argv": argv}

    def start(self, msg: dict, refusal) -> dict:
        """The preflight, then the run. `refusal(text, extra)` builds the helper's refusal exception."""
        pre = self.preflight(msg)
        if not pre["allowed"]:
            self.h.log.event("HIL launch refused: " + "; ".join(pre["refusals"]))
            raise refusal(pre["refusals"][0], {"refusals": pre["refusals"], "preflight": pre})
        reply = self.h.run_start({"cmd": "run_start", "kind": "hil", "port": pre["port"], "seconds": pre["seconds"], "expect": "none", "gui": True})
        self.hold, self.hold_run = True, reply["run"]["run_id"]
        return {**reply, "preflight": pre}

    # ---- the verdict ----------------------------------------------------------------------------------------------------------

    def verdict_path(self, manifest: dict) -> Path:
        return resolve_under(Path(manifest["out_dir"]) / VERDICT_FILE, (self.h.runs.runs_dir,))

    def verdict(self, msg: dict) -> dict:
        unknown = sorted(set(msg) - {"cmd", "id", "run_id", "mode", "text"})
        if unknown:
            raise DeviceError(f"not a field of the verdict: {', '.join(unknown)}")
        manifest = self.h.runs.manifest(msg.get("run_id"))
        if manifest["kind"] != "hil":
            raise DeviceError(f"run {manifest['run_id']} is not a HIL run: a verdict is a HIL run's")
        if manifest["state"] == "running":
            raise DeviceError(f"run {manifest['run_id']} is still going: give the verdict after it ends")
        if msg.get("mode") not in VERDICT_MODES:
            raise DeviceError(f"mode must be one of {', '.join(VERDICT_MODES)}: the flight mode flown")
        text = msg.get("text")
        if not isinstance(text, str) or not text.strip():
            raise DeviceError("the verdict needs text: say what happened")
        if len(text) > VERDICT_TEXT_MAX:
            raise DeviceError(f"the verdict text is {len(text)} characters, above {VERDICT_TEXT_MAX}")
        path = self.verdict_path(manifest)
        path.parent.mkdir(parents=True, exist_ok=True)
        reports = sorted(p.name for p in path.parent.glob("*.hil_report.txt"))
        record = {"schema": VERDICT_SCHEMA, "run_id": manifest["run_id"], "mode": msg["mode"], "text": text, "saved_utc": utc_iso(),
                  "report": reports[0] if reports else None}
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            raise DeviceError(f"run {manifest['run_id']} already has a verdict ({path}): it is evidence and is not overwritten") from None
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(json.dumps(record, indent=1) + "\n")
        self.h.log.event(f"HIL verdict saved: run {manifest['run_id']} {msg['mode']} {path}")
        return {"verdict": record, "path": str(path)}

    def last_verdict(self) -> dict | None:
        last = self.h.runs.status()["last"]
        if last is None or last["kind"] != "hil":
            return None
        try:
            return json.loads(self.verdict_path(last).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None

    # ---- SWD flash of the bench images ----------------------------------------------------------------------------------------

    def swd_busy(self) -> bool:
        return self._swd_task is not None and not self._swd_task.done()

    def swd_plan(self, msg: dict) -> tuple[str, Path]:
        """(kind, the ELF) of a flash_swd message, or DeviceError naming why it is refused. Nothing is run."""
        h = self.h
        unknown = sorted(set(msg) - {"cmd", "id", "kind"})
        if unknown:
            raise DeviceError(f"not an option of an SWD flash: {', '.join(unknown)}")
        kind = msg.get("kind")
        if not isinstance(kind, str) or kind not in BENCH_KINDS:
            raise DeviceError(f"unknown image kind {kind!r}: {', '.join(BENCH_KINDS)}")
        if BENCH_KINDS[kind]:
            raise DeviceError(MOTOR_CAPABLE_REFUSAL.format(kind=kind))
        ok, detail = probe_check(h.sysfs_dir, os.environ.get("MARV_PROBE_SERIAL"))
        if not ok:
            raise DeviceError(detail)
        reasons = h.idle_refusals(heavy=False)
        if isinstance(h.device, SerialDevice):
            reasons.append(f"the helper has a serial port open ({h.device.path}): close it, the flash reboots the board")
        if reasons:
            raise DeviceError("; ".join(reasons))
        tree = Path(h.build_root) / HIL_TREE
        cache = cache_values(tree)
        wrong = {k: cache.get(k) for k, v in HIL_CACHE.items() if cache.get(k) != v}
        if wrong:
            raise DeviceError(f"{tree} is not a {kind} build tree (CMake cache: " + ", ".join(f"{k}={v or 'absent'}" for k, v in wrong.items())
                              + "; needs " + ", ".join(f"{k}={v}" for k, v in HIL_CACHE.items()) + "): the helper never builds")
        elf = tree / HIL_ELF
        if not elf.is_file():
            raise DeviceError(f"no image at {elf}: build it first, the helper never builds")
        return kind, elf

    async def start_swd(self, msg: dict) -> dict:
        kind, elf = self.swd_plan(msg)
        digest = hashlib.sha256(elf.read_bytes()).hexdigest()
        self.swd = {"running": True, "kind": kind, "elf": str(elf), "elf_sha256": digest, "ok": None, "exit_status": None, "tail": []}
        self.h.log.event(f"SWD flash started: {kind} {elf} sha256 {digest}")
        self._swd_task = asyncio.create_task(self._run_swd(kind, elf))
        return {"started": True, "kind": kind, "elf": str(elf), "elf_sha256": digest}

    async def _run_swd(self, kind: str, elf: Path) -> None:
        state = self.swd
        tail: list[str] = []

        def progress(line: str) -> None:
            tail.append(line)
            del tail[:-SWD_TAIL_LINES]
            state["tail"] = list(tail)
            self.h.log.event(f"SWD flash {kind}: {line}")
            self.h._push(json.dumps({"event": "flash_swd_progress", "kind": kind, "line": line}))

        status = None
        error = None
        try:
            proc = await asyncio.create_subprocess_exec(str(self.swd_script), str(elf), env=dict(os.environ), stdout=asyncio.subprocess.PIPE,
                                                        stderr=asyncio.subprocess.STDOUT, start_new_session=True)
            self._swd_proc = proc

            async def read() -> None:
                async for raw in proc.stdout:
                    progress(raw.decode("utf-8", "replace").rstrip())

            try:
                await asyncio.wait_for(asyncio.gather(read(), proc.wait()), SWD_TIMEOUT_S)
            except asyncio.TimeoutError:
                error = f"flash.sh did not end within {SWD_TIMEOUT_S} s"
                await self._kill_swd()
                await proc.wait()
            status = proc.returncode
        except OSError as e:
            error = f"flash.sh did not start: {e}"
        finally:
            self._swd_proc = None
        ok = status == 0 and error is None
        if not ok and error is None:
            error = f"flash.sh exited with status {status}"
        self.swd = {**state, "running": False, "ok": ok, "exit_status": status, "error": error}
        self.h.log.event(f"SWD flash done: {kind} ok {ok} status {status} {error or ''}")
        self.h._push(json.dumps({"event": "flash_swd_done", "kind": kind, "ok": ok, "exit_status": status, "error": error,
                                 "elf": str(elf), "elf_sha256": state["elf_sha256"]}))

    async def _kill_swd(self) -> None:
        proc = self._swd_proc
        if proc is None:
            return
        for sig, wait in ((signal.SIGTERM, SWD_STOP_S), (signal.SIGKILL, SWD_STOP_S)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                return
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline:
                try:
                    os.killpg(proc.pid, 0)
                except ProcessLookupError:
                    return
                await asyncio.sleep(0.05)

    async def stop(self) -> None:
        """The helper is stopping: a flash.sh that is still going is stopped with its whole process group (H-2)."""
        if self._swd_proc is not None:
            await self._kill_swd()
        if self._swd_task is not None and not self._swd_task.done():
            try:
                await asyncio.wait_for(self._swd_task, SWD_STOP_S)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                self._swd_task.cancel()

    def status(self) -> dict:
        return {"hold": self.hold, "hold_run": self.hold_run, "verdict": self.last_verdict(), "swd_flash": self.swd}

