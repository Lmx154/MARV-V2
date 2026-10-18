#!/usr/bin/env python3
"""Bench only (ground-segment unit 15): flashing the FC3v2 board through the helper's gate. Never run by CI.

  uv run python tests/regression/quad/L10/results/flash_panel/bench_flash.py <step> <name> --elf <ELF> [--tree-note TEXT]

Board disarmed, ESC unpowered; nothing here arms or commands a motor. The helper's image library is build/ (the ELF given must be in
a tree there: its image hash is tools/config/image_hash.py of it), the report store is raw/reports/. Each step writes raw/<name>.json
(its checks, PASS or FAIL each), raw/<name>.*console*.log and raw/<name>.*ground.log. Steps:
  look      the boot lines, the link identity, the board's USB device number and serial nodes.
  validate  (a)/(c) the Flash page's Validate: the helper validates the config the board will run (factory) for the image with the
            real validator (tools/validate/validate_config.py, a Gazebo SIL run), report stored under (image hash, config hash).
  flash     (a)/(c) the Flash button: arming state, identity, flash_plan, the maintenance grant over the link, flash, flash_done, the
            identity read back over the link; PASS when the image hash, config hash and config source read back are the plan's.
  refuse    (b) no report for the image: the page's gate stops before the grant (the button is off), and the helper, asked to flash
            directly with no grant, refuses before picotool; the console's uptime goes on, the USB device number and nodes are
            unchanged and picotool_invocations.log has no new line.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import flash_common as fc  # noqa: E402
from flash_common import check, lc, sc  # noqa: E402

REPORTS = fc.RAW / "reports"
UPTIME_TOLERANCE_S = 3  # labelled harness value: how far the board's uptime may differ from the host's clock between two looks
INVOCATIONS = fc.PICOTOOL_LOG


def invocation_count() -> int:
    return len(INVOCATIONS.read_text().splitlines()) if INVOCATIONS.exists() else 0


def new_gcs() -> fc.Gcs:
    return fc.Gcs(fc.RAW / "gs-runs", REPORTS)


def image_hash_of(elf: Path) -> str:
    return sc.host_image(elf)["sha256"]


def step_look(a, res):
    boot = fc.uptime_us(fc.RAW / f"{a.name}.console.log")
    res["boot"] = boot
    res["usb"] = {"devnum": fc.usb_devnum(), "nodes": fc.usb_nodes()}
    gcs = new_gcs()
    try:
        res["identity"] = gcs.identity()
        res["armed"] = gcs.armed()
    finally:
        gcs.close(fc.RAW / f"{a.name}.ground.log")
    check(res, "looked", True, image=res["identity"]["image_sha256"], config=res["identity"]["config_sha256"],
          source=res["identity"]["config_source"], state=res["identity"]["config_state"], armed=res["armed"],
          arming=boot["stats"]["arming"], est_valid=boot["stats"]["est_valid"], devnum=res["usb"]["devnum"])


def step_validate(a, res):
    image = image_hash_of(Path(a.elf))
    res["image_hash"] = image
    gcs = new_gcs()
    try:
        out = gcs.validate(image)
    finally:
        gcs.close(fc.RAW / f"{a.name}.ground.log")
    res["validation"] = out
    done = out.get("done", {})
    report = REPORTS / image / (out.get("started", {}).get("config_hash") or "-") / "report.json"
    check(res, "validated", done.get("verdict") == "PASS" and report.is_file(), image_hash=image,
          config_hash=out.get("started", {}).get("config_hash"), verdict=done.get("verdict"), report_id=done.get("report_id"),
          exit_status=done.get("exit_status"))


def step_flash(a, res):
    image = image_hash_of(Path(a.elf))
    res["image_hash"] = image
    before = invocation_count()
    gcs = new_gcs()
    try:
        ok = gcs.gui_flash(image, res)
    finally:
        gcs.close(fc.RAW / f"{a.name}.ground.log")
    res["picotool_invocations"] = INVOCATIONS.read_text().splitlines()[before:] if INVOCATIONS.exists() else []
    time.sleep(sc.BOOT_WAIT_S / 2)
    res["boot_after"] = fc.uptime_us(fc.RAW / f"{a.name}.console_after.log")
    res["usb_after"] = {"devnum": fc.usb_devnum(), "nodes": fc.usb_nodes()}
    done = res.get("flash_done", {})
    check(res, "flashed", ok and done.get("ok") is True, image_hash=image, readback=res.get("readback"), stopped=res.get("stopped"),
          picotool_status=done.get("picotool_status"), invocations=len(res["picotool_invocations"]))
    host = sc.host_image(Path(a.elf))
    boot = res["boot_after"]
    check(res, "boot line image hash equals the host's", boot.get("image", {}).get("sha256") == host["sha256"],
          boot=boot.get("image", {}).get("sha256"), host=host["sha256"], arming=boot["stats"]["arming"], est_valid=boot["stats"]["est_valid"])


def step_refuse(a, res):
    image = image_hash_of(Path(a.elf))
    res["image_hash"] = image
    before = invocation_count()
    first = fc.uptime_us(fc.RAW / f"{a.name}.console_before.log")
    t0 = time.time()
    res["usb_before"] = {"devnum": fc.usb_devnum(), "nodes": fc.usb_nodes()}
    gcs = new_gcs()
    try:
        facts, ident = gcs.board_facts()
        res["board_facts"], res["identity_before"] = facts, ident
        plan = gcs.control("flash_plan", image_hash=image, board=facts)
        res["plan"] = plan
        gate = plan["plan"]["gate"]
        check(res, "the page's gate is closed", plan["ok"] and gate["passes"] is False and plan["plan"]["allowed"] is False,
              reason=gate["reason"], config=plan["plan"]["config"]["hash"], can_validate=plan["plan"]["can_validate"])
        sent_before = gcs.client.mark()
        direct = gcs.control("flash", image_hash=image, board=facts)
        res["direct_flash"] = direct
        check(res, "the helper refuses before picotool", direct["ok"] is False and "picotool was not started" in direct.get("error", ""),
              error=direct.get("error"))
        res["helper_status"] = gcs.control("status")["status"]
        check(res, "no flash was started", res["helper_status"]["flash"] is None and res["helper_status"]["device"] is not None,
              flash=res["helper_status"]["flash"], device=res["helper_status"]["device"])
        grants = [m for m in gcs.client.messages(sent_before) if m.get_type() == "COMMAND_ACK"]
        res["acks_after_direct_flash"] = len(grants)
    finally:
        gcs.close(fc.RAW / f"{a.name}.ground.log")
    second = fc.uptime_us(fc.RAW / f"{a.name}.console_after.log")
    t1 = time.time()  # each look ends with the board's newest stats line, so both clocks are read at the end of a look
    res["usb_after"] = {"devnum": fc.usb_devnum(), "nodes": fc.usb_nodes()}
    res["picotool_invocations"] = INVOCATIONS.read_text().splitlines()[before:] if INVOCATIONS.exists() else []
    dt_host = t1 - t0
    dt_board = (second["stats"]["t_us"] - first["stats"]["t_us"]) / 1e6
    check(res, "picotool was not invoked", res["picotool_invocations"] == [], new_lines=res["picotool_invocations"])
    check(res, "board uptime continued", 0 < dt_board and abs(dt_board - dt_host) < UPTIME_TOLERANCE_S, board_s=round(dt_board, 2),
          host_s=round(dt_host, 2), t_before_us=first["stats"]["t_us"], t_after_us=second["stats"]["t_us"])
    check(res, "no USB re-enumeration", res["usb_before"] == res["usb_after"], before=res["usb_before"], after=res["usb_after"])


STEPS = {"look": step_look, "validate": step_validate, "flash": step_flash, "refuse": step_refuse}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("step", choices=sorted(STEPS))
    ap.add_argument("name")
    ap.add_argument("--elf", required=False, help="the image's ELF in a tree of build/ (not needed for look)")
    a = ap.parse_args()
    fc.RAW.mkdir(parents=True, exist_ok=True)
    res = {"step": a.step, "name": a.name, "elf": a.elf, "date": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    try:
        STEPS[a.step](a, res)
    except Exception as e:  # noqa: BLE001 - a bench step records its failure
        import traceback
        res["exception"] = traceback.format_exc()
        check(res, "step completed", False, error=f"{type(e).__name__}: {e}")
    fc.dump(a.name, res)
    return 0 if all(c["pass"] for c in res.get("checks", {}).values()) and res.get("checks") else 1


if __name__ == "__main__":
    sys.exit(main())
