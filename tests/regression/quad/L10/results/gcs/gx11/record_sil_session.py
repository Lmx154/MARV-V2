#!/usr/bin/env python3
"""Bench evidence (ground-segment unit GX-11, GS-8): records a ground log of a real Gazebo SIL session, and the values the live client decoded.

  ci/heavy.sh -- uv run python tests/regression/quad/L10/results/gcs/gx11/record_sil_session.py <out_dir> [--seconds N]

Needs the host-gz-quadfc build (`cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc`) and gz-sim 8. It starts the
helper (tools/gs/helper.py) with the SIL launcher, the way the GCS's Simulation page does (the sil_start command of /ws), and is the helper's
WebSocket client as the GCS is: it reads the heartbeat, asks for the identity, asks for the nine telemetry streams at the rates of the live view
(gcs/src/lib/link/telemetry.ts DISPLAY_RATES_HZ), reads for N seconds, stops the streams and the run. It never arms: the stick script holds the
arm switch low.

Writes to <out_dir>:
  ground.log               the helper's ground log of the session (tools/gs/groundlog.py): the SIL's bytes and the client's requests
  live.decoded.jsonl       what the live client decoded from the bytes it received over the WebSocket (tests/tools/replay_decode.py), one
                           frame per line, in order
  record.json              the helper's start-up lines, the counts, and the check that the WebSocket bytes equal the log's received bytes
The helper's own heavy-lock file is private to this run (the script is run under ci/heavy.sh, which holds the host's), as tools/gs/e2e.sh does.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[6]
sys.path.insert(0, str(ROOT / "tests" / "tools"))
sys.path.insert(0, str(ROOT / "tools" / "gs"))
import groundlog  # noqa: E402
import replay_decode  # noqa: E402

SECONDS = 8  # labelled harness value: how long the streams run
HELPER_START_S = 30  # labelled harness value: how long the helper may take to print its URL
RUN_START_S = 180  # labelled harness value: how long a simulator run may take to bind its socket and answer (bounds a hung run only)
GCS_SYSTEM, GCS_COMPONENT = 255, 190  # the ground-station convention
# The live view's request rates (gcs/src/lib/link/telemetry.ts DISPLAY_RATES_HZ), by message name.
RATES_HZ = {"SYS_STATUS": 2, "ATTITUDE": 20, "ATTITUDE_QUATERNION": 10, "RC_CHANNELS": 10, "ATTITUDE_TARGET": 20, "HIGHRES_IMU": 20,
            "ACTUATOR_OUTPUT_STATUS": 20, "MARV_FC_STATUS": 10, "MARV_ESTIMATOR": 10}
US_PER_S = 1_000_000
INTERVAL_STOP = -1


def free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--seconds", type=float, default=SECONDS)
    args = ap.parse_args()
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    d = replay_decode.load_dialect(out / "defs")
    from websockets.sync.client import connect

    runs = out / "gs-runs"
    byid = out / "byid"
    byid.mkdir(exist_ok=True)
    proc = subprocess.Popen([sys.executable, str(ROOT / "tools" / "gs" / "helper.py"), "--port", "0", "--runs-dir", str(runs),
                             "--heavy-lock", str(out / "heavy.lock"), "--byid-dir", str(byid), "--sil-udp-port", str(free_udp_port())],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    lines, url, run_dir = [], None, None
    deadline = time.monotonic() + HELPER_START_S
    while time.monotonic() < deadline and (url is None or run_dir is None):
        line = proc.stdout.readline()
        if not line:
            break
        lines.append(line.rstrip("\n"))
        if line.startswith("GCS:"):
            url = line.split(None, 1)[1].strip()
        elif line.startswith("run dir:"):
            run_dir = Path(line.split(":", 1)[1].strip())
    if url is None or run_dir is None:
        raise SystemExit("the helper did not start:\n" + "\n".join(lines))
    port, token = re.match(r"http://127\.0\.0\.1:(\d+)/\?token=(\S+)", url).groups()
    got = bytearray()
    result: dict = {"helper_output": lines}
    ws = None
    try:
        ws = connect(f"ws://127.0.0.1:{port}/ws?token={token}", origin=f"http://127.0.0.1:{port}", compression=None)
        mav = d.MAVLink(None, srcSystem=GCS_SYSTEM, srcComponent=GCS_COMPONENT)
        vehicle = {"sys": 1, "comp": 1}  # the SIL link service's address (fw/link/src/service.cpp)

        def control(**msg):
            ws.send(json.dumps(msg))

        events = []

        def pump(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                try:
                    msg = ws.recv(timeout=max(0.01, end - time.monotonic()))
                except TimeoutError:
                    return
                if isinstance(msg, bytes):
                    got.extend(msg)
                else:
                    event = json.loads(msg).get("event")
                    if event:
                        events.append(event)

        def seen(name):
            return any(m["type"] == name for m in replay_decode.decode(d, bytes(got)))

        def wait_for(name, seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                pump(0.2)
                if seen(name):
                    return
            raise SystemExit(f"no {name} within {seconds} s")

        def command(cmd_id, p1=0.0, p2=0.0):
            ws.send(d.MAVLink_command_long_message(vehicle["sys"], vehicle["comp"], cmd_id, 0, p1, p2, 0, 0, 0, 0, 0).pack(mav))

        control(cmd="sil_start", id=1)
        wait_for("HEARTBEAT", RUN_START_S)
        command(d.MAV_CMD_REQUEST_MESSAGE, float(d.MAVLINK_MSG_ID_MARV_IDENTITY))
        wait_for("MARV_IDENTITY", 10)
        for name, hz in RATES_HZ.items():
            command(d.MAV_CMD_SET_MESSAGE_INTERVAL, float(getattr(d, f"MAVLINK_MSG_ID_{name}")), float(round(US_PER_S / hz)))
        pump(args.seconds)
        for name in RATES_HZ:
            command(d.MAV_CMD_SET_MESSAGE_INTERVAL, float(getattr(d, f"MAVLINK_MSG_ID_{name}")), float(INTERVAL_STOP))
        pump(1.0)
        control(cmd="sil_stop", id=2)
        # Close the device, so that the log ends where the client's bytes do (the queue to the client is in order: its bytes come before the event).
        control(cmd="close", id=3)
        end = time.monotonic() + 30
        while "device_closed" not in events and time.monotonic() < end:
            pump(0.2)
        assert "device_closed" in events, "the helper did not report the device closed"
    finally:
        if ws is not None:
            ws.close()
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        result["helper_output"] += proc.stdout.read().splitlines()
    log = run_dir / "ground.log"
    shutil.copy(log, out / "ground.log")
    rx = b"".join(groundlog.byte_stream(log, groundlog.RX))
    result["ws_bytes"], result["rx_bytes"] = len(got), len(rx)
    assert rx == bytes(got), "the bytes the client received over the WebSocket are not the log's received stream"
    live = replay_decode.decode(d, bytes(got))
    (out / "live.decoded.jsonl").write_text("".join(json.dumps(m, sort_keys=True) + "\n" for m in live), encoding="utf-8")
    counts: dict = {}
    for m in live:
        counts[m["type"]] = counts.get(m["type"], 0) + 1
    result["frames"], result["by_type"] = len(live), counts
    result["helper_output"] = [re.sub(r"token=\S+", "token=<redacted>", line) for line in result["helper_output"]]  # the session token is never written to disk
    (out / "record.json").write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "helper_output"}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
