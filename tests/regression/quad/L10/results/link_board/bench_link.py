#!/usr/bin/env python3
"""Bench only (ground-segment unit 12b): the FC3v2 flight image's ground link through the helper, disarmed. Never run by CI.

  uv run python tests/regression/quad/L10/results/link_board/bench_link.py <out_prefix> [--picotool]

The board runs the flight image of the m33 build (build/m33), disarmed, ESC unpowered; nothing here arms or commands a motor.
The helper (tools/gs/helper.py --serial <if02>) owns the link port; this script is its WebSocket client and reads the console
(if00) alongside. Steps, each printed PASS or FAIL:
  link        heartbeat (disarmed), AUTOPILOT_VERSION (byte-wise capability), MARV_IDENTITY (factory, OK, schema hash of the
              manifest, config hash of the factory .mcfg, staged_differs 0)
  (a) list    PARAM_REQUEST_LIST: every parameter of the manifest, link id, type and count as the manifest says, value bits equal
              the factory .mcfg; no sequence gap
  (b) stage   an unlocked f32 one ulp up: echoed staged; read by index shows it staged with the active value unchanged;
              identity staged_differs 1; a locked one: refused with its status text, value unchanged
  (c) activate  zero report id: DENIED_REPORT; a nonzero report id with the staged config's hash: refused with its reason
              (the board's image hash is unknown until unit 14, so nothing can be bound); revert: DENIED_UNAVAILABLE; factory
              reset: refused (no boot request store until unit 14); afterwards the board keeps running (heartbeats, console t)
  (d) reset   with --picotool: `picotool reboot -f` without a grant is refused (usb_reset_refused + 1), the board keeps running
  (d) grant   with --maintenance (last; the board reboots): MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1 1 is IN_PROGRESS then FAILED
              with its status text (no reboot on the board until unit 14) and leaves the grant held (console arming=maintenance);
              param1 3 is IN_PROGRESS then ACCEPTED; `picotool reboot -f` is then honoured: the console port goes away and the
              board comes back with a fresh clock, usb_reset_refused 0 and est_valid=1
Writes <out_prefix>.json (results), <out_prefix>.console.log (console bytes) and <out_prefix>.ground.log (the helper's log).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import link_common as lc  # noqa: E402
from link_common import mcfg  # noqa: E402

ACCEPTED_PARAM = "att_kp"  # an unlocked f32 parameter (test_link_sil.py's); one ulp up, which the validator accepts
LOCKED_PARAM = "rotor_speed_max"  # a locked f32 parameter (Luis-signed lock list, decision 0053 Q2)
ALIVE_S = 3  # labelled harness value: how long the board is watched after the refusals
PICOTOOL_SETTLE_S = 3  # labelled harness value: the wait after picotool before the console is read again
REBOOT_WAIT_S = 15  # labelled harness value: the wait for the console after a reboot (12 s for ModemManager, then a stats line)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_prefix")
    ap.add_argument("--picotool", action="store_true", help="also send `picotool reboot -f` (refused without a grant)")
    ap.add_argument("--maintenance", action="store_true", help="last, take the maintenance grant and reboot with picotool")
    args = ap.parse_args()
    out = Path(args.out_prefix)
    out.parent.mkdir(parents=True, exist_ok=True)
    d = lc.load_dialect()
    manifest = json.loads(lc.MANIFEST.read_text(encoding="utf-8"))
    factory = mcfg.decode(lc.FACTORY_MCFG.read_bytes())
    names = list(manifest["params"])
    records = mcfg.section_records(factory.section("quad"))
    schema = mcfg.manifest_schema_hash(manifest)
    res = {"elf_sha256": lc.sha256_hex(lc.BUILD / "fw" / "hal" / "target" / "marv_fc3v2.elf"), "steps": {}}
    verdicts = {}

    def step(name, ok, **detail):
        verdicts[name] = bool(ok)
        res["steps"][name] = {"pass": bool(ok), **detail}
        print(f"{'PASS' if ok else 'FAIL'} {name}: " + ", ".join(f"{k}={v}" for k, v in detail.items()), flush=True)

    console = lc.ConsoleReader()
    console.start()
    helper = lc.HelperProcess(out.parent / "gs-runs")
    pipe = client = None
    try:
        before = lc.wait_console(console)
        res["console_before"] = before
        pipe = lc.WsPipe(helper)
        client = lc.Client(d, pipe)

        hb = client.heartbeat()
        version = client.version()
        ident = client.identity()
        step("link", hb.get_srcSystem() == lc.VEHICLE[0] and not hb.base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED
             and bool(version.capabilities & d.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)
             and ident.config_source == d.MARV_CONFIG_SOURCE_FACTORY and ident.config_state == d.MARV_CONFIG_STATE_OK
             and ident.param_schema_hash == schema and bytes(ident.config_sha256) == mcfg.config_hash(factory)
             and ident.staged_differs == 0,
             heartbeat_armed=bool(hb.base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED), system_status=hb.system_status,
             capabilities=hex(version.capabilities), config_source=ident.config_source, config_state=ident.config_state,
             config_refusal=ident.config_refusal, schema_hash=hex(ident.param_schema_hash),
             config_sha256=bytes(ident.config_sha256).hex(), image_sha256=bytes(ident.image_sha256).hex(),
             staged_differs=ident.staged_differs)

        # (a) the list
        values, recs, elapsed, at = client.list_all(manifest["count"])
        mismatch = []
        for i, name in enumerate(names):
            m = values.get(i)
            want_type = d.MAV_PARAM_TYPE_REAL32 if records[i].type == "f32" else d.MAV_PARAM_TYPE_INT32
            if (m is None or m.param_count != manifest["count"] or m.param_id != manifest["params"][name]["link_id"]
                    or m.param_type != want_type or lc.value_bits(d, m) != records[i].value_bits
                    or recs[i].active_bits != records[i].value_bits or recs[i].staged_bits != records[i].value_bits
                    or recs[i].lock != int(manifest["params"][name]["lock"])):
                mismatch.append(name)
        gaps = client.seq_gaps(at)
        frames = len(client.messages(at))
        step("(a) list", len(values) == len(recs) == manifest["count"] == len(records) and not mismatch and gaps == 0,
             manifest_count=manifest["count"], values=len(values), records=len(recs), mismatched=mismatch, seq_gaps=gaps,
             frames=frames, seconds=round(elapsed, 3))
        res["list"] = {names[i]: {"link_id": values[i].param_id, "type": values[i].param_type,
                                  "bits": f"{lc.value_bits(d, values[i]):#010x}", "factory_bits": f"{records[i].value_bits:#010x}",
                                  "lock": recs[i].lock} for i in sorted(values)}

        # (b) staging
        accepted, locked = names.index(ACCEPTED_PARAM), names.index(LOCKED_PARAM)
        new_bits = records[accepted].value_bits + 1
        link_id = manifest["params"][ACCEPTED_PARAM]["link_id"]
        echo, at = client.set(link_id, new_bits, d.MAV_PARAM_TYPE_REAL32)
        accepted_texts = client.texts(at)
        value, record = client.read(accepted)
        ident_b = client.identity()
        locked_id = manifest["params"][LOCKED_PARAM]["link_id"]
        lecho, lat = client.set(locked_id, records[locked].value_bits + 1, d.MAV_PARAM_TYPE_REAL32)
        lvalue, lrecord = client.read(locked)
        locked_texts = client.texts(lat)
        step("(b) stage", lc.value_bits(d, echo) == new_bits and not accepted_texts and lc.value_bits(d, value) == new_bits
             and record.staged_bits == new_bits and record.active_bits == records[accepted].value_bits
             and ident_b.staged_differs == 1 and lc.value_bits(d, lecho) == records[locked].value_bits
             and any(t.startswith(f"{locked_id}: ") for t in locked_texts) and lrecord.lock == 1
             and lrecord.staged_bits == lrecord.active_bits == records[locked].value_bits,
             param=ACCEPTED_PARAM, factory_bits=f"{records[accepted].value_bits:#010x}", echo_bits=f"{lc.value_bits(d, echo):#010x}",
             read_bits=f"{lc.value_bits(d, value):#010x}", record_staged=f"{record.staged_bits:#010x}",
             record_active=f"{record.active_bits:#010x}", origin=record.origin, method=record.method, sigma_kind=record.sigma_kind,
             staged_differs=ident_b.staged_differs, locked_param=LOCKED_PARAM, locked_echo=f"{lc.value_bits(d, lecho):#010x}",
             locked_texts=locked_texts)

        # (c) activation and the other config commands
        staged = lc.staged_config(factory, manifest, {accepted: mcfg.Record("f32", new_bits, 0, "Choice", "scenario", "Manual",
                                                                            False, "set over the link; no source stated")})
        expected = mcfg.config_hash(staged)
        report_id = hashlib.sha256(b"bench_link.py: a report id no validation produced").digest()
        acks = {}
        for label, action, rid, exp in (("activate_no_report", d.MARV_CONFIG_ACTION_ACTIVATE, bytes(mcfg.DIGEST), expected),
                                        ("activate", d.MARV_CONFIG_ACTION_ACTIVATE, report_id, expected),
                                        ("revert", d.MARV_CONFIG_ACTION_REVERT_LAST_GOOD, bytes(mcfg.DIGEST), bytes(mcfg.DIGEST)),
                                        ("factory_reset", d.MARV_CONFIG_ACTION_RESET_FACTORY, bytes(mcfg.DIGEST), bytes(mcfg.DIGEST))):
            ack, at = client.config(action, rid, exp, schema)
            client.listen(0.3)
            acks[label] = {"result": ack.result, "texts": client.texts(at)}
        t_before = lc.console_fields(console)["t_us"]
        at = client.mark()
        client.listen(ALIVE_S)
        heartbeats = len(client.since(at, "HEARTBEAT"))
        after = lc.console_fields(console)
        ident_c = client.identity()
        step("(c) activate", acks["activate_no_report"]["result"] == d.MARV_CONFIG_RESULT_DENIED_REPORT
             and acks["activate"]["result"] == d.MARV_CONFIG_RESULT_FAILED and acks["activate"]["texts"]
             and acks["revert"]["result"] == d.MARV_CONFIG_RESULT_DENIED_UNAVAILABLE
             and acks["factory_reset"]["result"] == d.MARV_CONFIG_RESULT_FAILED and acks["factory_reset"]["texts"]
             and heartbeats >= ALIVE_S - 1 and after["t_us"] > t_before and after["est_valid"] == 1
             and ident_c.config_source == d.MARV_CONFIG_SOURCE_FACTORY and ident_c.last_activation_result == d.MARV_ACTIVATION_RESULT_NONE,
             acks=acks, expected_config_sha256=expected.hex(), heartbeats_in_window=heartbeats, console_t_before=t_before,
             console_t_after=after["t_us"], est_valid=after["est_valid"], arming=after["arming"],
             staged_differs=ident_c.staged_differs, maintenance_requested="no (every command was refused before the store asks for it)")

        # (d) a USB reset without the grant
        if args.picotool:
            r0 = lc.console_fields(console)["usb_reset_refused"]
            p = subprocess.run([str(lc.PICOTOOL), "reboot", "-f"], capture_output=True, text=True, timeout=30, check=False)
            time.sleep(PICOTOOL_SETTLE_S)
            at = client.mark()
            client.listen(2)
            f = lc.console_fields(console)
            step("(d) picotool reboot -f, no grant", f["usb_reset_refused"] == r0 + 1 and f["t_us"] > after["t_us"]
                 and len(client.since(at, "HEARTBEAT")) >= 1,
                 picotool_rc=p.returncode, picotool_out=(p.stdout + p.stderr).strip(), usb_reset_refused_before=r0,
                 usb_reset_refused_after=f["usb_reset_refused"], console_t=f["t_us"], arming=f["arming"])

        res["console_after"] = lc.console_fields(console)
        res["seq_gaps_total"] = client.seq_gaps()
        res["frames_total"] = len(client.got)

        # (d) the maintenance grant over the link, then a USB reset honoured
        if args.maintenance:
            acks1, at1 = client.reboot(lc.REBOOT_AUTOPILOT)
            texts1 = client.texts(at1)
            acks3, _ = client.reboot(lc.REBOOT_TO_BOOTLOADER)
            time.sleep(PICOTOOL_SETTLE_S)
            f = lc.console_fields(console)
            step("(d) grant", acks1 == [d.MAV_RESULT_IN_PROGRESS, d.MAV_RESULT_FAILED] and bool(texts1)
                 and acks3 == [d.MAV_RESULT_IN_PROGRESS, d.MAV_RESULT_ACCEPTED] and f["arming"] == "maintenance",
                 reboot_acks=acks1, reboot_texts=texts1, bootloader_acks=acks3, arming=f["arming"], console_t=f["t_us"])
            r_before, t_before = f["usb_reset_refused"], f["t_us"]
            p = subprocess.run([str(lc.PICOTOOL), "reboot", "-f"], capture_output=True, text=True, timeout=30, check=False)
            time.sleep(REBOOT_WAIT_S)
            console2 = lc.ConsoleReader()
            console2.start()
            try:
                g = lc.wait_console(console2, REBOOT_WAIT_S)
            finally:
                console2.stop()
                Path(str(out) + ".after_reboot.console.log").write_bytes(bytes(console2.raw))
            step("(d) picotool reboot -f, granted", p.returncode == 0 and g["t_us"] < t_before and g["usb_reset_refused"] == 0
                 and g["est_valid"] == 1,
                 picotool_rc=p.returncode, picotool_out=(p.stdout + p.stderr).strip(), usb_reset_refused_before=r_before,
                 console_t_before=t_before, usb_reset_refused_after_reboot=g["usb_reset_refused"], console_t_after_reboot=g["t_us"],
                 est_valid=g["est_valid"], arming=g["arming"])
            res["console_after_reboot"] = g
    finally:
        if pipe is not None:
            try:
                pipe.close()
            except Exception:  # noqa: BLE001 - the port may be gone after a reboot
                pass
        helper.stop()
        console.stop()
        Path(str(out) + ".console.log").write_bytes(bytes(console.raw))
        res["helper_output"] = helper.lines
        if helper.run_dir is not None and (helper.run_dir / "ground.log").exists():
            shutil.copy(helper.run_dir / "ground.log", str(out) + ".ground.log")
        Path(str(out) + ".json").write_text(json.dumps(res, indent=1, default=str) + "\n", encoding="utf-8")
    ok = all(verdicts.values())
    print("PASS" if ok else "FAIL", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
