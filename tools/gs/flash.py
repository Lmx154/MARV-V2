"""Flashing a board image (ground-segment unit 15, decision 0053 Q2 G7 and Q6).

The gate. An image is flashed only when a PASSING validation report is stored for the pair (image hash, the config the board
will run with that image): the report's verdict is PASS, it names that image hash and that config hash, and its id is the
SHA-256 of its bytes (tools/validate/validate_config.py). The config the board will run:

  - the board's stored config (config source `active`) when the board's parameter schema equals the image's: the board keeps it
    across a flash (unit 14), so its config hash is the pair's second element;
  - else the image's own factory config (tools/config/mcfg.py factory file of the image's parameter set): a board on its factory
    table boots the new image's compiled table, and a stored config of another schema is refused at boot.

The gate reads only files: a refusal happens before the UF2 is made, before picotool is started and before the board is touched.

What the helper does and does not do. The helper relays bytes and never parses the link (GS-3), so the two steps that need the
link stay in the GCS (its codec): the maintenance grant before the flash and the image hash read back after it. The helper
checks the gate, makes sure the UF2 holds exactly the image's flash bytes, runs `picotool load -f -v -x` (the board's USB reset
interface honours it only after the grant, and it is not armed), gives the serial port up while the board reboots, and opens it
again when the port is back. It states the identity the board is expected to report (image hash, config hash, config source).
"""

from __future__ import annotations

import asyncio
import hashlib
import json
import os
import signal
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Awaitable, Callable

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "config"))
import image_hash as board_image  # noqa: E402
import mcfg  # noqa: E402
from artifacts import HASH_RE, ImageLibrary, ReportLibrary  # noqa: E402

PICOTOOL = "picotool"  # on PATH unless --picotool says otherwise
CONVERT_S = 60  # labelled harness value: bounds a hung `picotool uf2 convert`
LOAD_S = 180  # labelled harness value: bounds a hung `picotool load` (the unit 14 bench used the same bound)
KILL_S = 5  # labelled harness value: how long a killed process group may take to end
PORT_WAIT_S = 60  # labelled harness value: how long the helper waits for the serial port to come back after the load
PORT_POLL_S = 0.25  # labelled harness value: the poll period of that wait
PORT_SETTLE_S = 3  # labelled harness value: the pause between the port appearing and opening it (USB enumeration and the host's modem probing)
OPEN_RETRIES = 5  # labelled harness value: attempts to open the returned port
OPEN_RETRY_S = 1  # labelled harness value: the pause between them

# UF2 file format (https://github.com/microsoft/uf2, "File format"): 512-byte blocks, a 32-byte header of eight little-endian
# 32-bit words, up to 476 payload bytes, the end magic. Flag 0x1 is "not main flash".
UF2_BLOCK = 512
UF2_HEADER = struct.Struct("<8I")
UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_PAYLOAD_MAX = 476
UF2_FLAG_NOT_MAIN_FLASH = 0x00000001
UF2_END_OFFSET = UF2_BLOCK - 4
# picotool pads the last UF2 block of an image to the flash page (RP2350 datasheet, QSPI flash: 256-byte pages); the board hashes only
# [__flash_binary_start, __flash_binary_end), so up to one page of trailing zeros is not part of the image.
FLASH_PAGE = 256


class FlashRefused(Exception):
    """The flash is not done; the message is the reason shown to the operator."""


# ---- the UF2 -----------------------------------------------------------------------------------------------------------

def uf2_flash_bytes(data: bytes) -> bytes:
    """The flash bytes a UF2 writes, from the XIP window's start: blocks in any order must tile that range with no gap or overlap
    and carry no payload outside the window. FlashRefused otherwise."""
    if not data or len(data) % UF2_BLOCK:
        raise FlashRefused(f"the UF2 is {len(data)} bytes, not a whole number of {UF2_BLOCK}-byte blocks")
    blocks = []
    for at in range(0, len(data), UF2_BLOCK):
        block = data[at:at + UF2_BLOCK]
        m0, m1, flags, target, size, _no, _count, _family = UF2_HEADER.unpack_from(block)
        (end,) = struct.unpack_from("<I", block, UF2_END_OFFSET)
        if (m0, m1, end) != (UF2_MAGIC_START0, UF2_MAGIC_START1, UF2_MAGIC_END):
            raise FlashRefused(f"UF2 block {at // UF2_BLOCK} has bad magic numbers")
        if flags & UF2_FLAG_NOT_MAIN_FLASH:
            raise FlashRefused(f"UF2 block {at // UF2_BLOCK} is flagged not-main-flash")
        if size > UF2_PAYLOAD_MAX:
            raise FlashRefused(f"UF2 block {at // UF2_BLOCK} claims {size} payload bytes")
        blocks.append((target, block[UF2_HEADER.size:UF2_HEADER.size + size]))
    blocks.sort(key=lambda b: b[0])
    out = bytearray()
    expect = board_image.XIP_BASE
    for target, payload in blocks:
        if target != expect:
            raise FlashRefused(f"the UF2 has a gap or an overlap at {expect:#010x} (next block at {target:#010x}); "
                               f"the flash bytes must start at {board_image.XIP_BASE:#010x} and tile without a gap")
        out += payload
        expect += len(payload)
    return bytes(out)


def uf2_matches(path: Path, image_hash: str) -> str | None:
    """None when the UF2 holds exactly the flash bytes whose SHA-256 is `image_hash`, else why not."""
    try:
        flash = uf2_flash_bytes(path.read_bytes())
    except OSError as e:
        return f"cannot read the UF2: {e.strerror}"
    except FlashRefused as e:
        return str(e)
    got = hashlib.sha256(flash).hexdigest()
    if got == image_hash:
        return None
    for pad in range(1, min(FLASH_PAGE, len(flash) + 1)):
        if flash[len(flash) - pad:].strip(b"\0") or hashlib.sha256(flash[:len(flash) - pad]).hexdigest() != image_hash:
            continue
        return None  # the image, then fewer than a flash page of zero padding
    return f"the UF2 holds image {got[:12]}, not {image_hash[:12]}"


# ---- the plan and the gate ---------------------------------------------------------------------------------------------

def schema_int(text) -> int:
    try:
        return int(str(text), 16)
    except ValueError:
        raise FlashRefused(f"{text!r} is not a schema hash (0x and hex digits)") from None


def config_hash_of(cfg: "mcfg.Config", schema: int) -> str:
    """The config hash (core IF-1) of the section of `schema`: what a report names and what the board reports."""
    for s in cfg.sections:
        if s.schema_hash == schema:
            return mcfg.config_hash(cfg, (s.kind,)).hex()
    raise FlashRefused("the config has no section of the image's parameter schema")


def read_report(reports: ReportLibrary, image_hash: str, config_hash: str) -> dict:
    """The gate for one pair: {"passes", "reason", "report_id", "verdict"}. Reads the report store only."""
    path = reports.report(image_hash, config_hash)
    if path is None:
        return {"passes": False, "reason": f"no validation report for image {image_hash[:12]} with config {config_hash[:12]}",
                "report_id": None, "verdict": None}
    data = path.read_bytes()
    report_id = hashlib.sha256(data).hexdigest()
    try:
        doc = json.loads(data)
        verdict = doc.get("verdict")
        named_image = (doc.get("image") or {}).get("sha256")
        named_config = (doc.get("config") or {}).get("config_hash")
    except (ValueError, AttributeError):
        return {"passes": False, "reason": "the stored report is not readable", "report_id": report_id, "verdict": None}
    reason = None
    if verdict != "PASS":
        reason = f"the validation report for image {image_hash[:12]} with config {config_hash[:12]} says {verdict}, not PASS"
    elif named_image != image_hash or named_config != config_hash:
        reason = "the stored report names another image or config than the pair it is stored under"
    return {"passes": reason is None, "reason": reason, "report_id": report_id, "verdict": verdict}


def parse_board(board) -> dict:
    """The board facts the GCS sends from the vehicle's identity: schema_hash '0x...', config_hash hex, config_source
    'factory'|'active', armed bool. FlashRefused when one is missing or malformed."""
    if not isinstance(board, dict):
        raise FlashRefused("the vehicle's identity is missing: flashing needs a connected board")
    schema = schema_int(board.get("schema_hash"))
    config_hash = board.get("config_hash")
    if not isinstance(config_hash, str) or not HASH_RE.match(config_hash):
        raise FlashRefused("the vehicle's config hash is missing or malformed")
    source = board.get("config_source")
    if source not in ("factory", "active"):
        raise FlashRefused(f"the vehicle's config source {source!r} is neither factory nor active")
    return {"schema_hash": schema, "config_hash": config_hash, "config_source": source, "armed": board.get("armed")}


def board_entry(images: ImageLibrary, image_hash: str) -> dict:
    if not isinstance(image_hash, str) or not HASH_RE.match(image_hash):
        raise FlashRefused("the image hash must be 64 lower-case hex digits")
    for entry in images.scan()["images"]:
        if entry["hash"] == image_hash:
            if entry["kind"] != "board":
                raise FlashRefused(f"{entry['name']} is not a board image (it is {entry['kind']})")
            return entry
    raise FlashRefused(f"the image {image_hash[:12]} is not in the image library")


def make_plan(images: ImageLibrary, reports: ReportLibrary, image_hash: str, board_msg) -> dict:
    """What flashing this image onto this board would run, and whether the gate passes. FlashRefused for an image or a board the
    flash cannot even be planned for; otherwise {"allowed", "refusal", ...} (the gate and the arming state decide `allowed`)."""
    entry = board_entry(images, image_hash)
    board = parse_board(board_msg)
    art = entry["artifacts"]
    for key in ("params_manifest", "factory_mcfg"):
        if key not in art:
            raise FlashRefused(f"the image has no {key}: its parameter schema and factory config are unknown")
    if "elf" not in art and "uf2" not in art:
        raise FlashRefused("the image has neither an ELF nor a UF2 beside it to flash")
    root = images.root
    image_schema = mcfg.manifest_schema_hash(mcfg.load_manifest(root / art["params_manifest"]))
    keep = board["config_source"] == "active" and board["schema_hash"] == image_schema
    if keep:
        which, config_hash = "active", board["config_hash"]
        why = f"the board's stored config has the image's schema {image_schema:#018x}: it keeps it across the flash"
    else:
        try:
            factory = mcfg.decode((root / art["factory_mcfg"]).read_bytes())
        except (mcfg.McfgError, OSError) as e:
            raise FlashRefused(f"the image's factory config cannot be read: {e}") from None
        which, config_hash = "factory", config_hash_of(factory, image_schema)
        why = ("the board runs its factory table, which the new image brings" if board["config_source"] == "factory"
               else f"the board's stored config has schema {board['schema_hash']:#018x}, the image's is {image_schema:#018x}: "
                    "the image boots its factory table")
    gate = read_report(reports, image_hash, config_hash)
    if board["armed"] is not False:
        refusal = ("the vehicle is armed: never flash while armed" if board["armed"] is True
                   else "the vehicle's arming state is unknown (no heartbeat): never flash without knowing it is disarmed")
    else:
        refusal = gate["reason"]
    return {
        "image": {"hash": image_hash, "tree": entry["tree"], "name": entry["name"], "schema_hash": f"{image_schema:#018x}"},
        "config": {"which": which, "hash": config_hash, "expected_source": "active" if keep else "factory", "why": why},
        "gate": gate,
        "can_validate": which == "factory" or reports.find_config(config_hash) is not None,
        "allowed": refusal is None,
        "refusal": refusal,
    }


# ---- running picotool --------------------------------------------------------------------------------------------------

async def run_command(cmd: list[str], timeout_s: float, on_line: Callable[[str], None]) -> tuple[int | None, str | None]:
    """(return code, None), or (None, why) when the command did not start or was killed at the timeout. Output lines (picotool ends
    progress bars with a carriage return) go to on_line as they come."""
    try:
        proc = await asyncio.create_subprocess_exec(*cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
                                                    start_new_session=True)
    except OSError as e:
        return None, f"{cmd[0]} did not start: {e.strerror}"

    async def pump() -> None:
        buf = b""
        while chunk := await proc.stdout.read(4096):
            buf += chunk
            *lines, buf = buf.replace(b"\r", b"\n").split(b"\n")
            for line in lines:
                if line.strip():
                    on_line(line.decode("utf-8", "replace").rstrip())
        if buf.strip():
            on_line(buf.decode("utf-8", "replace").rstrip())

    reader = asyncio.create_task(pump())
    try:
        status = await asyncio.wait_for(proc.wait(), timeout_s)
    except asyncio.TimeoutError:
        await kill_group(proc)
        reader.cancel()
        return None, f"{Path(cmd[0]).name} did not finish in {timeout_s:g} s and was killed"
    except asyncio.CancelledError:
        await kill_group(proc)
        reader.cancel()
        raise
    await reader
    return status, None


async def kill_group(proc: asyncio.subprocess.Process) -> None:
    for sig in (signal.SIGTERM, signal.SIGKILL):
        if proc.returncode is not None:
            return
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            await asyncio.wait_for(proc.wait(), KILL_S)
        except asyncio.TimeoutError:
            continue


@dataclass
class Hooks:
    """What the helper gives the flash: progress lines, giving up the serial port (its path and baud rate, or FlashRefused when
    the open device is not a serial port) and opening it again."""
    progress: Callable[[str, str], None]
    release_device: Callable[[], tuple[str, int]]
    reopen: Callable[[str, int], Awaitable[None]]


async def prepare_uf2(entry: dict, root: Path, picotool: str, work: Path, hooks: Hooks) -> Path:
    """A UF2 whose flash bytes hash to the image's hash: the image's own UF2 when it holds exactly those bytes, else the ELF
    converted with `picotool uf2 convert` (and checked the same way)."""
    art = entry["artifacts"]
    image_hash = entry["hash"]
    problems = []
    if "uf2" in art:
        why = uf2_matches(root / art["uf2"], image_hash)
        if why is None:
            return root / art["uf2"]
        problems.append(f"the library's UF2: {why}")
    if "elf" in art:
        elf = root / art["elf"]
        try:
            from_elf = hashlib.sha256(board_image.flash_image(elf).data).hexdigest()
        except (board_image.ImageError, OSError) as e:
            raise FlashRefused(f"the ELF has no flash image: {e}") from None
        if from_elf != image_hash:
            raise FlashRefused(f"the ELF holds image {from_elf[:12]}, not {image_hash[:12]}")
        work.mkdir(parents=True, exist_ok=True)
        out = work / f"{image_hash}.uf2"
        out.unlink(missing_ok=True)
        status, why = await run_command([picotool, "uf2", "convert", str(elf), str(out)], CONVERT_S,
                                        lambda line: hooks.progress("uf2", line))
        if why is not None or status != 0:
            raise FlashRefused(f"the ELF could not be converted to a UF2: {why or f'picotool exit status {status}'}")
        why = uf2_matches(out, image_hash)
        if why is not None:
            raise FlashRefused(f"the converted UF2 does not hold the image: {why}")
        return out
    raise FlashRefused("no usable UF2: " + "; ".join(problems))


async def wait_for_port(path: str, timeout_s: float, poll_s: float = PORT_POLL_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while True:
        if os.path.exists(path):
            return True
        if time.monotonic() >= deadline:
            return False
        await asyncio.sleep(poll_s)


async def run_flash(plan: dict, entry: dict, root: Path, picotool: str, work: Path, hooks: Hooks, *,
                    load_s: float = LOAD_S, port_wait_s: float = PORT_WAIT_S, settle_s: float = PORT_SETTLE_S) -> dict:
    """Everything after the gate (the caller made the plan and it is `allowed`; the maintenance grant is given). Returns the result
    {"ok", "stage", "error", "picotool_started", "picotool_status", "port_reopened", "expected"}; a failure is a result, not an
    exception, and the serial port is opened again whenever it was given up."""
    expected = {"image_hash": plan["image"]["hash"], "config_hash": plan["config"]["hash"], "config_source": plan["config"]["expected_source"]}
    result = {"ok": False, "stage": "uf2", "error": None, "picotool_started": False, "picotool_status": None,
              "port_reopened": False, "expected": expected}
    try:
        uf2 = await prepare_uf2(entry, root, picotool, work, hooks)
    except FlashRefused as e:
        result["error"] = str(e)
        return result
    result["stage"] = "picotool"
    try:
        path, baud = hooks.release_device()
    except FlashRefused as e:
        result["error"] = str(e)
        return result
    result["picotool_started"] = True
    try:
        status, why = await run_command([picotool, "load", "-f", "-v", "-x", str(uf2)], load_s, lambda line: hooks.progress("picotool", line))
        result["picotool_status"] = status
        if why is not None or status != 0:
            result["error"] = why or f"picotool load exited with status {status} (does the board have the maintenance grant?)"
    finally:
        result["stage"] = "port"
        come_back = await wait_for_port(path, port_wait_s)
        if come_back:
            await asyncio.sleep(settle_s)
            for attempt in range(OPEN_RETRIES):
                try:
                    await hooks.reopen(path, baud)
                    result["port_reopened"] = True
                    break
                except Exception as e:  # the port may not accept an open yet; the last reason is kept
                    last = str(e)
                    await asyncio.sleep(OPEN_RETRY_S if attempt + 1 < OPEN_RETRIES else 0)
            else:
                result["error"] = result["error"] or f"the serial port {path} came back but could not be opened: {last}"
        elif result["error"] is None:
            result["error"] = f"the serial port {path} did not come back within {port_wait_s:g} s"
    if result["error"] is None:
        result["ok"], result["stage"] = True, "done"
    return result
