"""Bench only (ground-segment unit 15): the GCS's side of a flash, driven from Python. Never run by CI.

`Gcs` plays the GUI (gcs/src/lib/state/flash_state.ts) against the real helper (tools/gs/helper.py on the board's link CDC, the
real tools/gs/flash.py, picotool) and the real board: one WebSocket carries both the raw link bytes (decoded here with the MARV
dialect) and the helper's JSON control messages. The steps of the GUI's flow are the same ones, in the same order: the arming
state from the heartbeat, the identity, `flash_plan`, the maintenance grant over the link, `flash`, the helper's flash_done, then
the identity read back over the link and compared with the plan's expectation.
"""
from __future__ import annotations

import json
import os
import re
import signal
import stat
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "link_board"))
sys.path.insert(0, str(HERE.parent / "slots"))
import link_common as lc  # noqa: E402
import slots_common as sc  # noqa: E402

ROOT = lc.ROOT
RAW = HERE / "raw"
BUILD_ROOT = ROOT / "build"
PICOTOOL = lc.PICOTOOL
WRAPPER = HERE / "picotool_logged.sh"
PICOTOOL_LOG = RAW / "picotool_invocations.log"
FLASH_DONE_S = 400  # labelled harness value: the GUI's flashDoneMs (gcs/src/lib/state/flash_state.ts)
IDENTITY_S = 60  # labelled harness value: the GUI's identityMs
VALIDATE_S = 3600  # labelled harness value: bounds a hung validation only
CONTROL_S = 10  # labelled harness value: the GUI's CONTROL_TIMEOUT_MS


class FlashHelper(lc.HelperProcess):
    """The helper on the link port with the image library at build/, the logged picotool and a reports directory."""

    def __init__(self, runs_dir: Path, reports_dir: Path):
        runs_dir.mkdir(parents=True, exist_ok=True)
        self.proc = subprocess.Popen([sys.executable, str(lc.HELPER), "--serial", lc.LINK, "--port", "0", "--runs-dir", str(runs_dir),
                                      "--build-root", str(BUILD_ROOT), "--reports-dir", str(reports_dir), "--picotool", str(WRAPPER)],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        self.lines = []
        deadline = time.monotonic() + lc.HELPER_START_S
        self.url = self.run_dir = None
        while time.monotonic() < deadline and (self.url is None or self.run_dir is None):
            line = self.proc.stdout.readline()
            if not line:
                break
            self.lines.append(line.rstrip("\n"))
            if line.startswith("GCS:"):
                self.url = line.split(None, 1)[1].strip()
            elif line.startswith("run dir:"):
                self.run_dir = Path(line.split(":", 1)[1].strip())
        if self.url is None:
            self.stop()
            raise RuntimeError("the helper did not start:\n" + "\n".join(self.lines))
        m = re.match(r"http://127\.0\.0\.1:(\d+)/\?token=(\S+)", self.url)
        self.port, self.token = int(m.group(1)), m.group(2)
        self.origin = f"http://127.0.0.1:{self.port}"


class MixedPipe:
    """lc.Client's pipe over the helper's WebSocket: binary frames are link bytes, text frames are the helper's JSON (replies and events)."""

    def __init__(self, helper: FlashHelper):
        from websockets.sync.client import connect
        self.ws = connect(f"ws://127.0.0.1:{helper.port}/ws?token={helper.token}", origin=helper.origin, compression=None,
                          max_size=None)
        self.texts: list[dict] = []

    def write(self, data: bytes):
        self.ws.send(data)

    def send_json(self, obj: dict):
        self.ws.send(json.dumps(obj))

    def read(self, timeout: float) -> bytes:
        try:
            msg = self.ws.recv(timeout=timeout)
        except TimeoutError:
            return b""
        if isinstance(msg, bytes):
            return msg
        self.texts.append(json.loads(msg))
        return b""

    def close(self):
        self.ws.close()


class Gcs:
    def __init__(self, runs_dir: Path, reports_dir: Path):
        self.d = lc.load_dialect()
        self.helper = FlashHelper(runs_dir, reports_dir)
        self.pipe = MixedPipe(self.helper)
        self.client = lc.Client(self.d, self.pipe)
        self._id = 0
        self.log: list[str] = []

    def close(self, ground_log: Path | None = None):
        try:
            self.pipe.close()
        except Exception:  # noqa: BLE001 - the port may be gone after a reboot
            pass
        self.helper.stop()
        if ground_log is not None and self.helper.run_dir is not None and (self.helper.run_dir / "ground.log").exists():
            ground_log.write_bytes((self.helper.run_dir / "ground.log").read_bytes())

    # ---- the helper's control messages ----------------------------------------------------------------------------------

    def control(self, cmd: str, timeout: float = CONTROL_S, **args) -> dict:
        self._id += 1
        mine = self._id
        self.pipe.send_json({"cmd": cmd, "id": mine, **args})
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.client.pump()
            for t in self.pipe.texts:
                if t.get("id") == mine:
                    return t
        raise TimeoutError(f"the helper did not answer {cmd}")

    def wait_event(self, name: str, timeout: float, **match) -> dict:
        deadline = time.monotonic() + timeout
        seen = 0
        while time.monotonic() < deadline:
            self.client.pump()
            for t in self.pipe.texts[seen:]:
                if t.get("event") == name and all(t.get(k) == v for k, v in match.items()):
                    return t
            seen = len(self.pipe.texts)
        raise TimeoutError(f"no {name} event within {timeout} s")

    def events(self) -> list[dict]:
        return [t for t in self.pipe.texts if "event" in t]

    # ---- the vehicle ------------------------------------------------------------------------------------------------------

    def armed(self) -> bool:
        hb = self.client.heartbeat()
        return bool(hb.base_mode & self.d.MAV_MODE_FLAG_SAFETY_ARMED)

    def identity(self) -> dict:
        i = self.client.identity()
        d = self.d
        source = {d.MARV_CONFIG_SOURCE_FACTORY: "factory", d.MARV_CONFIG_SOURCE_ACTIVE: "active"}[i.config_source]
        state = {d.MARV_CONFIG_STATE_OK: "ok", d.MARV_CONFIG_STATE_REFUSED: "refused"}[i.config_state]
        return {"image_sha256": bytes(i.image_sha256).hex(), "config_sha256": bytes(i.config_sha256).hex(),
                "schema_hash": f"{i.param_schema_hash:#018x}", "config_source": source, "config_state": state,
                "config_refusal": i.config_refusal, "report_id": bytes(i.validation_report_id).hex()}

    def board_facts(self) -> dict:
        ident = self.identity()
        return {"schema_hash": ident["schema_hash"], "config_hash": ident["config_sha256"], "config_source": ident["config_source"],
                "armed": self.armed()}, ident

    # ---- the GUI's flow ---------------------------------------------------------------------------------------------------

    def validate(self, image_hash: str, config: str = "factory") -> dict:
        """The Flash page's Validate: the helper's validation of the config the board will run for the image."""
        start = self.control("validate", image_hash=image_hash, config=config)
        if not start.get("ok"):
            return {"started": start}
        done = self.wait_event("validate_done", VALIDATE_S, image_hash=image_hash, config_hash=start["config_hash"])
        progress = [t["line"] for t in self.pipe.texts if t.get("event") == "validate_progress" and t.get("image_hash") == image_hash]
        return {"started": start, "done": done, "progress": progress}

    def gui_flash(self, image_hash: str, res: dict) -> bool:
        """The Flash button: the same gates and steps as FlashController.flash. Returns True when the read-back equals the plan."""
        facts, ident_before = self.board_facts()
        res["identity_before"], res["board_facts"] = ident_before, facts
        plan = self.control("flash_plan", image_hash=image_hash, board=facts)
        res["plan"] = plan
        if not plan.get("ok"):
            res["stopped"] = f"the gate could not be checked: {plan.get('error')}"
            return False
        if facts["armed"] is not False:
            res["stopped"] = "not sent: the vehicle is not reported disarmed"
            return False
        if not plan["plan"]["gate"]["passes"]:
            res["stopped"] = f"not sent: {plan['plan']['gate']['reason']}"
            return False
        acks, at = self.client.reboot(lc.REBOOT_TO_BOOTLOADER)
        res["grant"] = {"acks": acks, "texts": self.client.texts(at)}
        if acks[-1] != self.d.MAV_RESULT_ACCEPTED:
            res["stopped"] = f"the vehicle did not grant maintenance (ack {acks})"
            return False
        start = self.control("flash", image_hash=image_hash, board=facts)
        res["flash_start"] = start
        if not start.get("ok"):
            res["stopped"] = f"the helper refused to flash: {start.get('error')}"
            return False
        done = self.wait_event("flash_done", FLASH_DONE_S, image_hash=image_hash)
        res["flash_done"] = done
        res["flash_events"] = [t for t in self.pipe.texts if t.get("event") in ("device_closed", "device_opened", "flash_progress", "flash_done")]
        if not done["ok"]:
            res["stopped"] = f"the flash failed at {done['stage']}: {done['error']}"
            return False
        deadline = time.monotonic() + IDENTITY_S
        ident = None
        while time.monotonic() < deadline:
            try:
                ident = self.identity()
                break
            except TimeoutError:
                continue
        res["identity_after"] = ident
        if ident is None:
            res["stopped"] = f"the vehicle did not report its identity within {IDENTITY_S} s"
            return False
        exp = done["expected"]
        res["readback"] = {"image hash": [exp["image_hash"], ident["image_sha256"]], "config hash": [exp["config_hash"], ident["config_sha256"]],
                           "config source": [exp["config_source"], ident["config_source"]]}
        return all(a == b for a, b in res["readback"].values())


def usb_devnum(link_path: str = lc.LINK) -> int:
    """The USB device number of the board (it changes whenever the board enumerates again)."""
    tty = os.path.basename(os.path.realpath(link_path))
    return int(Path(f"/sys/class/tty/{tty}/device/../devnum").read_text())


def usb_nodes() -> dict:
    """inode and ctime of the board's serial nodes: new nodes after an enumeration."""
    out = {}
    for name in ("if00", "if02"):
        p = os.path.realpath(f"/dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-{name}")
        st = os.stat(p)
        out[name] = {"node": p, "ino": st.st_ino, "ctime": st.st_ctime}
    return out


def uptime_us(console_log: Path, seconds: float = sc.CONSOLE_S) -> dict:
    """The console's newest stats line: `t=` is the microseconds since boot."""
    return sc.boot_console(console_log, seconds)


def dump(name: str, doc: dict) -> None:
    (RAW / f"{name}.json").write_text(json.dumps(doc, indent=1, default=str) + "\n", encoding="utf-8")


def check(results: dict, name: str, ok: bool, **detail) -> bool:
    results.setdefault("checks", {})[name] = {"pass": bool(ok), **detail}
    print(f"{'PASS' if ok else 'FAIL'} {name}: " + ", ".join(f"{k}={v}" for k, v in detail.items()), flush=True)
    return bool(ok)
