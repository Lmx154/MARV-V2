#!/usr/bin/env python3
"""Bench only (ground-segment unit 14): the FC3v2 board's config slots, step by step. Never run by CI.

  uv run python tests/regression/quad/L10/results/slots/bench_slots.py <step> <name> [options]

Board disarmed, ESC unpowered; nothing here arms or commands a motor. Each step writes raw/<name>.json (its checks, PASS or
FAIL each), raw/<name>.console*.log (console bytes) and raw/<name>.ground.log (the helper's ground log) where it uses them.
Steps:
  hash      (e) the boot line `image: sha256` and the link identity's image hash equal tools/config/image_hash.py of --elf
            and the SHA-256 of `arm-none-eabi-objcopy -O binary`.
  activate  (a) stage --param at the factory value + --ulps (a plain link write), validate the staged config with
            tools/validate/validate_config.py for --elf's flash bytes (a PASS report; its id), activate with that id; the
            board reboots through the maintenance grant; after the reboot the boot line says source slot, request activate
            done, status ok, the staged config hash and the board's image hash as the binding, and --expect-sequence; the
            identity says active, ok, the same hash and report id; the parameter's active value is the staged one.
  revert    revert to the last good config: the store copies the older intact slot over the newest one (store.hpp,
            boot.cpp reactivate), so after the reboot the same slot holds it with sequence + 1 (request revert done,
            --expect-hash).
  factory   reset to factory: after the reboot source factory, status ok, the factory config hash.
  flash     (b) flash --elf with tools/target/flash.sh (OpenOCD): afterwards the config hash is --expect-hash and the status is
            --expect-status (not validated for an image the slot is not bound to); console arming as expected.
  uf2       (b) the maintenance grant over the link (reboot command param1 3), then picotool load -f -x of --elf converted to
            UF2: afterwards as for flash.
  corrupt   (c) flip one byte of the newest slot's config file over SWD (OpenOCD reads the slot's first sector, the script
            inverts the byte at --offset, OpenOCD writes the sector back), then reboot: source factory, status refused
            corrupt, console arming config_refused.
  look      the boot lines and identity only.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import slots_common as sc  # noqa: E402
from slots_common import lc, mcfg  # noqa: E402

PICOTOOL = lc.PICOTOOL
SLOT_BYTES = 98304  # design/budget.yaml config_slot_bytes (the bench reads the value it was built with; checked below)
FLASH_BYTES = 2097152  # PICO_FLASH_SIZE_BYTES of boards/madflight_fc3v2_revc.h
SECTOR = 4096  # FLASH_SECTOR_SIZE of the pinned Pico SDK
SLOT_OFFSET = {"slot A": FLASH_BYTES - 2 * SLOT_BYTES, "slot B": FLASH_BYTES - SLOT_BYTES}
XIP_BASE = 0x10000000  # RP2350 address map; tools/config/image_hash.py


def slot_bytes_of_build() -> int:
    import re
    text = (lc.BUILD / "CMakeCache.txt").read_text(encoding="utf-8")
    return int(re.search(r"^MARV_CONFIG_SLOT_BYTES:\w+=(\d+)$", text, re.M).group(1))


def after_reboot(res, name):
    time.sleep(sc.BOOT_WAIT_S)
    boot = sc.boot_console(sc.RAW / f"{name}.console_after.log")
    res["boot_after"] = boot
    return boot


def look(res, name, link_too=True):
    boot = sc.boot_console(sc.RAW / f"{name}.console.log")
    res["boot"] = boot
    if link_too:
        link = sc.Link(sc.RAW / "gs-runs")
        try:
            res["identity"] = link.identity()
        finally:
            link.close(sc.RAW / f"{name}.ground.log")
    return boot


def step_hash(a, res):
    boot = look(res, a.name)
    host = sc.host_image(Path(a.elf), sc.RAW.parent / "images" / f"{a.name}.bin")
    objcopy_bin = sc.RAW.parent / "images" / f"{a.name}.objcopy.bin"
    subprocess.run(["arm-none-eabi-objcopy", "-O", "binary", a.elf, str(objcopy_bin)], check=True)
    objcopy_sha = sc.hashlib.sha256(objcopy_bin.read_bytes()).hexdigest()
    res["host"] = host
    res["objcopy_sha256"] = objcopy_sha
    sc.check(res, "(e) image hash", boot["image"]["sha256"] == host["sha256"] == objcopy_sha == res["identity"]["image_sha256"],
             board=boot["image"]["sha256"], identity=res["identity"]["image_sha256"], host_elf=host["sha256"], objcopy_bin=objcopy_sha,
             bytes=boot["image"]["bytes"], hash_us=boot["image"]["us"], imu_start_us=boot.get("imu_start_us"))


def step_activate(a, res):
    man = sc.manifest()
    factory = mcfg.decode(lc.FACTORY_MCFG.read_bytes())
    names = list(man["params"])
    idx = names.index(a.param)
    base_bits = mcfg.section_records(factory.section("quad"))[idx].value_bits
    new_bits = base_bits + a.ulps
    staged = sc.staged_with(factory, man, {a.param: new_bits})
    expected = mcfg.config_hash(staged)
    image_bin = sc.RAW.parent / "images" / f"{a.name}.bin"
    host = sc.host_image(Path(a.elf), image_bin)
    res["host_image"] = host
    before = look(res, a.name + ".before", link_too=False)
    link = sc.Link(sc.RAW / "gs-runs")
    d = link.d
    try:
        res["identity_before"] = link.identity()
        sc.check(res, "board runs --elf", res["identity_before"]["image_sha256"] == host["sha256"],
                 board=res["identity_before"]["image_sha256"], host=host["sha256"])
        echo, at = link.client.set(man["params"][a.param]["link_id"], new_bits, d.MAV_PARAM_TYPE_REAL32)
        res["stage"] = {"param": a.param, "factory_bits": f"{base_bits:#010x}", "new_bits": f"{new_bits:#010x}",
                        "echo_bits": f"{lc.value_bits(d, echo):#010x}", "texts": link.client.texts(at)}
        ident = link.identity()
        sc.check(res, "staged", lc.value_bits(d, echo) == new_bits and ident["staged_differs"] == (0 if a.rebind else 1),
                 echo=res["stage"]["echo_bits"], staged_differs=ident["staged_differs"])
        res["validation"] = sc.validate(staged, image_bin, sc.RAW.parent / "validation" / a.name)
        v = res["validation"]
        sc.check(res, "validated", v["verdict"] == "PASS" and v["report_image_sha256"] == host["sha256"]
                 and v["report_config_hash"] == expected.hex(), **v)
        if v["verdict"] != "PASS" or v["report_id"] is None:
            return  # the G7 gate (the GCS's side): no activation without a passing report for this image and config
        schema = mcfg.manifest_schema_hash(man)
        ack, at = link.client.config(d.MARV_CONFIG_ACTION_ACTIVATE, bytes.fromhex(v["report_id"]), expected, schema)
        res["activate_ack"] = {"result": ack.result, "texts": link.client.texts(at)}
        sc.check(res, "activate accepted", ack.result == d.MARV_CONFIG_RESULT_ACCEPTED, **res["activate_ack"])
    finally:
        link.close(sc.RAW / f"{a.name}.ground.log")
    boot = after_reboot(res, a.name)
    c = boot["config"]
    link = sc.Link(sc.RAW / "gs-runs")
    try:
        ident = link.identity()
        value, record = link.client.read(idx)
        res["identity_after"] = ident
        res["param_after"] = {"value_bits": f"{lc.value_bits(d, value):#010x}", "active_bits": f"{record.active_bits:#010x}",
                              "staged_bits": f"{record.staged_bits:#010x}"}
    finally:
        link.close(sc.RAW / f"{a.name}.after.ground.log")
    sc.check(res, "(a) activated", c["source"].startswith("slot") and c["request"] == "activate" and c["request_result"] == "done"
             and c["status"] == "ok" and c["config_sha256"] == expected.hex() and c["bound_image"] == host["sha256"]
             and (a.expect_sequence is None or c["sequence"] == a.expect_sequence)
             and ident["config_source"] == d.MARV_CONFIG_SOURCE_ACTIVE and ident["config_state"] == d.MARV_CONFIG_STATE_OK
             and ident["config_sha256"] == expected.hex() and ident["report_id"] == v["report_id"]
             and ident["image_sha256"] == host["sha256"] and record.active_bits == new_bits and boot["stats"]["est_valid"] == 1,
             config_line=c["line"], identity=ident, param=res["param_after"], expected_config_sha256=expected.hex(),
             arming=boot["stats"]["arming"], est_valid=boot["stats"]["est_valid"], before=before["config"]["line"])


def step_command(a, res, action, request):
    link = sc.Link(sc.RAW / "gs-runs")
    d = link.d
    before = sc.boot_console(sc.RAW / f"{a.name}.console.log")
    res["boot"] = before
    try:
        res["identity_before"] = link.identity()
        ack, at = link.client.config(getattr(d, action), bytes(mcfg.DIGEST), bytes(mcfg.DIGEST),
                                     mcfg.manifest_schema_hash(sc.manifest()))
        res["ack"] = {"result": ack.result, "texts": link.client.texts(at)}
    finally:
        link.close(sc.RAW / f"{a.name}.ground.log")
    sc.check(res, f"{request} accepted", ack.result == d.MARV_CONFIG_RESULT_ACCEPTED, **res["ack"])
    boot = after_reboot(res, a.name)
    link = sc.Link(sc.RAW / "gs-runs")
    try:
        res["identity_after"] = link.identity()
    finally:
        link.close(sc.RAW / f"{a.name}.after.ground.log")
    return d, before, boot


def step_revert(a, res):
    d, before, boot = step_command(a, res, "MARV_CONFIG_ACTION_REVERT_LAST_GOOD", "revert")
    c, b0, ident = boot["config"], before["config"], res["identity_after"]
    sc.check(res, "(a) reverted", c["request"] == "revert" and c["request_result"] == "done" and c["status"] == "ok"
             and c["source"] == b0["source"] and c["sequence"] == b0["sequence"] + 1
             and c["config_sha256"] == a.expect_hash and ident["config_sha256"] == a.expect_hash
             and ident["config_state"] == d.MARV_CONFIG_STATE_OK,
             before=b0["line"], after=c["line"], identity=ident)


def step_factory(a, res):
    d, before, boot = step_command(a, res, "MARV_CONFIG_ACTION_RESET_FACTORY", "factory reset")
    c, ident = boot["config"], res["identity_after"]
    factory_hash = mcfg.config_hash(mcfg.decode(lc.FACTORY_MCFG.read_bytes())).hex()
    sc.check(res, "(a) factory reset", c["request"] == "factory reset" and c["request_result"] == "done" and c["source"] == "factory"
             and c["status"] == "ok" and ident["config_sha256"] == factory_hash
             and ident["config_source"] == d.MARV_CONFIG_SOURCE_FACTORY and ident["config_state"] == d.MARV_CONFIG_STATE_OK
             and boot["stats"]["arming"] != "config_refused",
             before=before["config"]["line"], after=c["line"], identity=ident, factory_hash=factory_hash,
             arming=boot["stats"]["arming"])


def expect_after_flash(a, res, boot, label):
    c = boot["config"]
    link = sc.Link(sc.RAW / "gs-runs")
    try:
        ident = link.identity()
    finally:
        link.close(sc.RAW / f"{a.name}.after.ground.log")
    res["identity_after"] = ident
    host = sc.host_image(Path(a.elf))
    refused = a.expect_status != "ok"
    sc.check(res, label, c["config_sha256"] == a.expect_hash and ident["config_sha256"] == a.expect_hash
             and c["status"] == a.expect_status and ident["image_sha256"] == host["sha256"] == boot["image"]["sha256"]
             and (boot["stats"]["arming"] == "config_refused") == refused,
             config_line=c["line"], identity=ident, image=host["sha256"], arming=boot["stats"]["arming"],
             est_valid=boot["stats"]["est_valid"])


def step_flash(a, res):
    res["before"] = sc.boot_console(sc.RAW / f"{a.name}.console.log")
    res["flash"] = sc.flash_sh(Path(a.elf), sc.RAW / f"{a.name}.flash.log")
    sc.check(res, "flash.sh verified", res["flash"]["returncode"] == 0 and res["flash"]["verified"], **res["flash"])
    boot = after_reboot(res, a.name)
    expect_after_flash(a, res, boot, "(b) flash.sh keeps the config")


def step_uf2(a, res):
    res["before"] = sc.boot_console(sc.RAW / f"{a.name}.console.log")
    uf2 = sc.RAW.parent / "images" / f"{a.name}.uf2"
    conv = subprocess.run([str(PICOTOOL), "uf2", "convert", a.elf, str(uf2)], capture_output=True, text=True, check=False)
    res["uf2_convert"] = {"returncode": conv.returncode, "out": (conv.stdout + conv.stderr).strip()}
    link = sc.Link(sc.RAW / "gs-runs")
    try:
        acks, at = link.client.reboot(lc.REBOOT_TO_BOOTLOADER)
        res["grant"] = {"acks": acks, "texts": link.client.texts(at)}
    finally:
        link.close(sc.RAW / f"{a.name}.ground.log")
    sc.check(res, "maintenance granted", acks[-1] == link.d.MAV_RESULT_ACCEPTED, **res["grant"])
    load = subprocess.run([str(PICOTOOL), "load", "-f", "-v", "-x", str(uf2)], capture_output=True, text=True, timeout=180,
                          check=False)
    res["picotool_load"] = {"returncode": load.returncode, "out": (load.stdout + load.stderr).strip()}
    (sc.RAW / f"{a.name}.picotool.log").write_text(load.stdout + load.stderr, encoding="utf-8")
    sc.check(res, "picotool load", load.returncode == 0, **res["picotool_load"])
    boot = after_reboot(res, a.name)
    expect_after_flash(a, res, boot, "(b) picotool UF2 load keeps the config")


def step_corrupt(a, res):
    before = sc.boot_console(sc.RAW / f"{a.name}.console.log")
    res["before"] = before
    c0 = before["config"]
    sc.check(res, "a slot is active", c0["source"].startswith("slot"), line=c0["line"])
    offset = SLOT_OFFSET[c0["source"]]
    sector = sc.RAW.parent / "images" / f"{a.name}.sector.bin"
    sc.openocd([f"flash read_bank 0 {sector} {offset} {SECTOR}"], sc.RAW / f"{a.name}.openocd_read.log")
    data = bytearray(sector.read_bytes())
    res["flipped"] = {"slot": c0["source"], "flash_offset": hex(offset + a.offset), "before": f"{data[a.offset]:#04x}"}
    data[a.offset] ^= 0xFF
    res["flipped"]["after"] = f"{data[a.offset]:#04x}"
    sector.write_bytes(bytes(data))
    sc.openocd([f"program {sector} {XIP_BASE + offset:#x} verify"], sc.RAW / f"{a.name}.openocd_write.log")
    boot = after_reboot(res, a.name)
    c = boot["config"]
    link = sc.Link(sc.RAW / "gs-runs")
    try:
        ident = link.identity()
        d = link.d
    finally:
        link.close(sc.RAW / f"{a.name}.after.ground.log")
    res["identity_after"] = ident
    sc.check(res, "(c) corrupt slot refused", c["source"] == "factory" and c["status"] == "refused: corrupt" and c["fault"]
             and boot["stats"]["arming"] == "config_refused" and ident["config_state"] == d.MARV_CONFIG_STATE_REFUSED,
             config_line=c["line"], identity=ident, arming=boot["stats"]["arming"], flipped=res["flipped"])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=["hash", "activate", "revert", "factory", "flash", "uf2", "corrupt", "look"])
    ap.add_argument("name")
    ap.add_argument("--elf", default=str(sc.ELF))
    ap.add_argument("--param", default="att_kp")
    ap.add_argument("--ulps", type=int, default=1)
    ap.add_argument("--expect-sequence", type=int)
    ap.add_argument("--rebind", action="store_true", help="activate: the running config already holds the value (re-binding the "
                    "same config to this image), so the staged copy does not differ from it")
    ap.add_argument("--expect-hash")
    ap.add_argument("--expect-status", default="ok")
    ap.add_argument("--offset", type=int, default=200, help="byte of the slot's file to invert (corrupt)")
    a = ap.parse_args()
    sc.RAW.mkdir(parents=True, exist_ok=True)
    (sc.RAW.parent / "images").mkdir(exist_ok=True)
    res = {"step": a.step, "elf": a.elf, "elf_sha256": sc.hashlib.sha256(Path(a.elf).read_bytes()).hexdigest()}
    if slot_bytes_of_build() != SLOT_BYTES:
        print("the build's slot size is not this script's", file=sys.stderr)
        return 2
    try:
        {"hash": step_hash, "activate": step_activate, "revert": step_revert, "factory": step_factory, "flash": step_flash,
         "uf2": step_uf2, "corrupt": step_corrupt, "look": lambda a, r: look(r, a.name)}[a.step](a, res)
    finally:
        sc.dump(a.name, res)
    ok = all(c["pass"] for c in res.get("checks", {}).values())
    print("PASS" if ok else "FAIL" if res.get("checks") else "DONE", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
