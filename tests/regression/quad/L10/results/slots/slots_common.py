"""Bench only (ground-segment unit 14): the PC side of the FC3v2 board's config slots. Never run by CI.

The board's boot lines (console if00: `image: sha256 ...` and `config: source ...`, fw/hal/target/src/main.cpp print_config), its
link identity through the helper (tools/gs/helper.py on if02, ../link_board/link_common.py), staging and the config commands,
the validator (tools/validate/validate_config.py) for a real report id, and the flashing paths: tools/target/flash.sh (OpenOCD
over SWD) and picotool (UF2 over USB, after the maintenance grant).
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "link_board"))
import link_common as lc  # noqa: E402
from link_common import mcfg  # noqa: E402

ROOT = lc.ROOT
RAW = HERE / "raw"
ELF = lc.BUILD / "fw" / "hal" / "target" / "marv_fc3v2.elf"
IMAGE_HASH = ROOT / "tools" / "config" / "image_hash.py"
VALIDATOR = ROOT / "tools" / "validate" / "validate_config.py"
GZ_BUILD = ROOT / "build" / "host-gz-quadfc"
FLASH_SH = ROOT / "tools" / "target" / "flash.sh"
OPENOCD = Path.home() / "pico" / "openocd-install" / "bin" / "openocd"
PROBE = os.environ.get("MARV_PROBE_SERIAL", "900678EF479C3A0E")
BOOT_WAIT_S = 12  # labelled harness value: the wait after a reboot before the console is opened (ModemManager, enumeration)
CONSOLE_S = 3  # labelled harness value: how long the console is read after it is opened (the boot lines, then a stats line)
REBOOT_S = 30  # labelled harness value: how long an accepted activation may take to reboot and come back
DTR_DROP_S = 0.5  # labelled harness value: how long DTR stays low before the console is opened again
VALIDATE_TIMEOUT_S = 3600  # labelled harness value: bounds a hung validation only

IMAGE_RE = re.compile(r"^image: sha256 ([0-9a-f]{64}) \((\d+) bytes, ([^,]+), (\d+) us\)")
CONFIG_RE = re.compile(r"^config: source (factory|slot A|slot B)(?: sequence (\d+) config_sha256 ([0-9a-f]{64})  ?bound_image "
                       r"([0-9a-f]{64}))?, status (ok|refused: [a-z ]+?)(?: \((.*?)\))?, boot request (\w+(?: \w+)?)(?: \((.*)\))?$")
BOOT_RE = re.compile(r"^boot: IMU start at (\d+) us")


def manifest():
    return json.loads(lc.MANIFEST.read_text(encoding="utf-8"))


def host_image(elf: Path = ELF, bin_out: Path | None = None) -> dict:
    args = [sys.executable, str(IMAGE_HASH), str(elf)] + (["--bin-out", str(bin_out)] if bin_out else [])
    out = subprocess.run(args, capture_output=True, text=True, check=True).stdout
    return {"elf": str(elf.relative_to(ROOT)) if elf.is_relative_to(ROOT) else str(elf),
            "elf_file_sha256": hashlib.sha256(elf.read_bytes()).hexdigest(),
            "sha256": re.search(r"^sha256 (\w+)", out, re.M).group(1), "text": out.strip()}


def boot_console(log: Path, seconds: float = CONSOLE_S) -> dict:
    """Opens the console (the board prints its boot lines on open), reads `seconds`, parses the image, config and boot lines
    and the newest stats line. Writes the raw bytes to `log`. The board prints its boot lines when a host raises DTR; the reader
    (../usb/usb_common.py open_raw) clears HUPCL, so DTR is first dropped here (HUPCL set, one open and close)."""
    subprocess.run(["stty", "-F", lc.CONSOLE, "hupcl"], check=False)
    os.close(os.open(lc.CONSOLE, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK))
    time.sleep(DTR_DROP_S)
    r = lc.ConsoleReader()
    r.start()
    try:
        time.sleep(seconds)
        stats = lc.wait_console(r, lc.REPLY_S)
    finally:
        r.stop()
        log.write_bytes(bytes(r.raw))
    out = {"stats": {k: v for k, v in stats.items() if k != "line"}}
    for _, raw in r.lines:
        line = raw.decode(errors="replace").strip()
        if m := IMAGE_RE.match(line):
            out["image"] = {"sha256": m[1], "bytes": int(m[2]), "how": m[3], "us": int(m[4])}
        elif m := CONFIG_RE.match(line):
            out["config"] = {"source": m[1], "sequence": int(m[2]) if m[2] else None, "config_sha256": m[3],
                             "bound_image": m[4], "status": m[5].strip(), "fault": m[6], "request": m[7],
                             "request_result": m[8], "line": line}
        elif m := BOOT_RE.match(line):
            out["imu_start_us"] = int(m[1])
        elif line.startswith("marv_fc3v2 "):
            out["version"] = line.split(" ", 1)[1]
    return out


class Link:
    """The helper on the link port and a client on its WebSocket."""

    def __init__(self, runs_dir: Path):
        self.d = lc.load_dialect()
        self.helper = lc.HelperProcess(runs_dir)
        self.pipe = lc.WsPipe(self.helper)
        self.client = lc.Client(self.d, self.pipe)

    def close(self, ground_log: Path | None = None):
        try:
            self.pipe.close()
        except Exception:  # noqa: BLE001 - the port may be gone after a reboot
            pass
        self.helper.stop()
        if ground_log is not None and self.helper.run_dir is not None and (self.helper.run_dir / "ground.log").exists():
            ground_log.write_bytes((self.helper.run_dir / "ground.log").read_bytes())

    def identity(self) -> dict:
        i = self.client.identity()
        return {"image_sha256": bytes(i.image_sha256).hex(), "config_sha256": bytes(i.config_sha256).hex(),
                "schema_hash": hex(i.param_schema_hash), "config_source": i.config_source, "config_state": i.config_state,
                "config_refusal": i.config_refusal, "last_activation_result": i.last_activation_result,
                "report_id": bytes(i.validation_report_id).hex(), "staged_differs": i.staged_differs}


def staged_with(factory, man, changes: dict[str, int]):
    """The factory config with each named f32 parameter's value bits replaced as a plain link write stages it."""
    names = list(man["params"])
    recs = {names.index(n): mcfg.Record("f32", bits, 0, "Choice", "scenario", "Manual", False, "set over the link; no source stated")
            for n, bits in changes.items()}
    return lc.staged_config(factory, man, recs)


def validate(config, image_bin: Path, work: Path) -> dict:
    """tools/validate/validate_config.py on `config` (an mcfg.Config) for the image whose flash bytes are `image_bin`: the
    report's image hash is the .bin's SHA-256, the board's image hash. Returns the verdict and the report id."""
    work.mkdir(parents=True, exist_ok=True)
    cfg_file, report = work / "staged.mcfg", work / "report.json"
    cfg_file.write_bytes(mcfg.encode(config))
    t0 = time.monotonic()
    p = subprocess.run([sys.executable, str(VALIDATOR), "--config", str(cfg_file), "--image", str(image_bin), "--build",
                        str(GZ_BUILD), "--work-dir", str(work / "work"), "--out", str(report)],
                       capture_output=True, text=True, timeout=VALIDATE_TIMEOUT_S, check=False)
    (work / "validate.log").write_text(p.stdout + "\n--- stderr ---\n" + p.stderr, encoding="utf-8")
    rid = re.search(r"^report_id: ([0-9a-f]{64})$", p.stdout, re.M)
    verdict = re.search(r"^verdict: (\w+)$", p.stdout, re.M)
    doc = json.loads(report.read_text(encoding="utf-8")) if report.exists() else {}
    return {"returncode": p.returncode, "verdict": verdict[1] if verdict else None, "report_id": rid[1] if rid else None,
            "report_image_sha256": doc.get("image", {}).get("sha256"), "report_config_hash": doc.get("config", {}).get("config_hash"),
            "seconds": round(time.monotonic() - t0, 1)}


def flash_sh(elf: Path, log: Path) -> dict:
    p = subprocess.run([str(FLASH_SH), str(elf)], env={**os.environ, "MARV_PROBE_SERIAL": PROBE}, capture_output=True, text=True,
                       timeout=180, check=False)
    log.write_text(p.stdout + p.stderr, encoding="utf-8")
    erase = re.findall(r"Adding extra erase range.*", p.stdout + p.stderr)
    return {"returncode": p.returncode, "verified": "** Verified OK **" in p.stdout + p.stderr, "erase_notes": erase}


def openocd(commands: list[str], log: Path) -> str:
    """OpenOCD on the Debug Probe with the board halted, then the board rebooted through the watchdog as flash.sh does."""
    reboot = ["targets rp2350.cm0", "halt", "write_memory 0x40018008 32 [expr {0x01ffffff & ~0xc}]",
              "write_memory 0x400d8000 32 [expr {[read_memory 0x400d8000 32 1] | 0x80000000}]", "shutdown"]
    args = [str(OPENOCD), "-f", "interface/cmsis-dap.cfg", "-c", f"adapter serial {PROBE}", "-f", "target/rp2350.cfg", "-c",
            "adapter speed 5000", "-c", "init", "-c", "reset halt"]
    for c in commands + reboot:
        args += ["-c", c]
    p = subprocess.run(args, capture_output=True, text=True, timeout=180, check=False)
    log.write_text(p.stdout + p.stderr, encoding="utf-8")
    return p.stdout + p.stderr


def dump(name: str, doc: dict) -> None:
    (RAW / f"{name}.json").write_text(json.dumps(doc, indent=1, default=str) + "\n", encoding="utf-8")


def check(results: dict, name: str, ok: bool, **detail) -> bool:
    results.setdefault("checks", {})[name] = {"pass": bool(ok), **detail}
    print(f"{'PASS' if ok else 'FAIL'} {name}: " + ", ".join(f"{k}={v}" for k, v in detail.items()), flush=True)
    return bool(ok)
