#!/usr/bin/env python3
"""Bench only (ground-segment unit 12b): the PC side of the armed link step, for the lead with Luis. Never run by CI.

  uv run python tests/regression/quad/L10/results/link_board/armed_runbook.py <out_prefix>

Preconditions (the operator confirms each at the prompt): props off, ESC unpowered, the bench-stats image of the flight image
flashed (cmake --preset m33 -B build/m33-bs -DMARV_FC3V2_BENCH_STATS=ON; tools/target/flash.sh), the radio on with the arm
switch low. This script never arms, disarms or commands a motor: Luis arms and disarms on the radio when it asks. It prints
PASS, FAIL or BLOCKED per step and writes <out_prefix>.json plus the console capture (<out_prefix>.console.log,
<out_prefix>.phases.json) that `bench_load.py analyze <out_prefix>` turns into the EMB-3 numbers.

Steps:
  1 disarmed     heartbeat not armed, console arming is not armed; usb_reset_refused r0
  2 arm          [Luis arms on the radio] heartbeat armed and console arming=armed
  3 armed refusals  a parameter write is refused (status text, value unchanged); an activation is DENIED_ARMED
  4 maintenance armed  BLOCKED: the link has no maintenance request yet (a lead decision, see README); when it exists: refused
  5 picotool armed  `picotool reboot -f` is refused: usb_reset_refused r0 + 1, the board still armed and running
  6 link at the cap  bench_load.load_phase for 60 s while armed (the console records bench_stats for EMB-3)
  7 disarm       [Luis disarms on the radio] heartbeat not armed
  8 maintenance disarmed  BLOCKED likewise; when it exists: granted, console arming=maintenance, then `picotool reboot -f` is
                 honoured (the board re-enumerates)
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bench_load  # noqa: E402
import link_common as lc  # noqa: E402
from link_common import mcfg  # noqa: E402

ARM_WAIT_S = 120  # labelled harness value: how long the script waits for Luis to arm or disarm
SETTLE_S = 3  # labelled harness value: the wait after picotool before the console is read again
ARMED_STATES = ("armed", "armed_hold", "armed_failsafe_centred")
PROBE_PARAM = "att_kp"  # an unlocked f32 parameter


def main() -> int:
    out = Path(sys.argv[1])
    out.parent.mkdir(parents=True, exist_ok=True)
    for q in ("props are off", "the ESC is unpowered", "the bench-stats flight image is flashed", "the arm switch is low"):
        if input(f"confirm: {q} [yes/no] ").strip() != "yes":
            print("stopped: precondition not confirmed")
            return 1
    d = lc.load_dialect()
    manifest = json.loads(lc.MANIFEST.read_text(encoding="utf-8"))
    names = list(manifest["params"])
    factory = mcfg.decode(lc.FACTORY_MCFG.read_bytes())
    records = mcfg.section_records(factory.section("quad"))
    schema = mcfg.manifest_schema_hash(manifest)
    res, phases, load = {"steps": {}}, {}, {}
    verdicts = []

    def step(name, verdict, **detail):
        verdicts.append(verdict)
        res["steps"][name] = {"verdict": verdict, **detail}
        print(f"{verdict} {name}: " + ", ".join(f"{k}={v}" for k, v in detail.items()), flush=True)

    def armed_hb(client):
        return bool(client.heartbeat().base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED)

    def wait_arming(client, console, want_armed):
        deadline = time.monotonic() + ARM_WAIT_S
        while time.monotonic() < deadline:
            f = lc.console_fields(console)
            if armed_hb(client) == want_armed and (f["arming"] in ARMED_STATES) == want_armed:
                return f
        return lc.console_fields(console)

    console = lc.ConsoleReader()
    console.start()
    pipe = lc.SerialPipe()
    client = lc.Client(d, pipe)
    try:
        f = lc.wait_console(console)
        r0 = f["usb_reset_refused"]
        hb = armed_hb(client)
        step("1 disarmed", "PASS" if not hb and f["arming"] not in ARMED_STATES else "FAIL", heartbeat_armed=hb,
             arming=f["arming"], usb_reset_refused=r0, est_valid=f["est_valid"])

        print(">>> Luis: arm on the radio now", flush=True)
        f = wait_arming(client, console, True)
        step("2 arm", "PASS" if f["arming"] in ARMED_STATES else "FAIL", arming=f["arming"])

        i = names.index(PROBE_PARAM)
        link_id = manifest["params"][PROBE_PARAM]["link_id"]
        echo, at = client.set(link_id, records[i].value_bits + 1, d.MAV_PARAM_TYPE_REAL32)
        texts = client.texts(at)
        ack, at2 = client.config(d.MARV_CONFIG_ACTION_ACTIVATE, bytes(range(1, 33)), bytes(mcfg.DIGEST), schema)
        ok = (lc.value_bits(d, echo) == records[i].value_bits and any("disarmed" in t for t in texts)
              and ack.result == d.MARV_CONFIG_RESULT_DENIED_ARMED)
        step("3 armed refusals", "PASS" if ok else "FAIL", set_echo=f"{lc.value_bits(d, echo):#010x}", set_texts=texts,
             activate_result=ack.result, activate_texts=client.texts(at2))

        step("4 maintenance armed", "BLOCKED", why="no maintenance request on the link yet (lead decision)")

        p = subprocess.run([str(lc.PICOTOOL), "reboot", "-f"], capture_output=True, text=True, timeout=30, check=False)
        time.sleep(SETTLE_S)
        f = lc.console_fields(console)
        step("5 picotool armed", "PASS" if f["usb_reset_refused"] == r0 + 1 and f["arming"] in ARMED_STATES else "FAIL",
             picotool=(p.stdout + p.stderr).strip(), usb_reset_refused=f["usb_reset_refused"], arming=f["arming"])

        phases["load"], load = bench_load.load_phase(pipe, d, bench_load.LOAD_S)
        f = lc.console_fields(console)
        step("6 link at the cap", "PASS" if load["seq_gaps"] == 0 and f["arming"] in ARMED_STATES else "FAIL",
             arming_after=f["arming"], **{k: load[k] for k in ("frames_per_s", "bytes_per_s", "seq_gaps")},
             emb3="run: bench_load.py analyze " + str(out))

        print(">>> Luis: disarm on the radio now", flush=True)
        f = wait_arming(client, console, False)
        step("7 disarm", "PASS" if f["arming"] not in ARMED_STATES else "FAIL", arming=f["arming"])

        step("8 maintenance disarmed", "BLOCKED", why="no maintenance request on the link yet (lead decision)")
    finally:
        pipe.close()
        console.stop()
        bench_load.write_capture(out, console, phases, load)
        Path(str(out) + ".json").write_text(json.dumps(res, indent=1, default=str) + "\n", encoding="utf-8")
    print("FAIL" if "FAIL" in verdicts else ("BLOCKED" if "BLOCKED" in verdicts else "PASS"), flush=True)
    return 1 if "FAIL" in verdicts else 0


if __name__ == "__main__":
    sys.exit(main())
