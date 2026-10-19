#!/usr/bin/env python3
"""Bench only (ground-segment unit GX-5, S-1): a ground log of the FC3v2 flight image with every telemetry stream on. Never run by CI.

  uv run python tests/regression/quad/L10/results/gcs/gx5/board/capture_ground_log.py <out_prefix> [--seconds N]

The helper (tools/gs/helper.py --serial <if02>) owns the board's link port and records the ground log; this script is its
WebSocket client, as the GCS is: it reads the identity and the parameter list once (so the log holds the param records a live view
shows), then asks every stream at an interval of 1 us (each is granted its cap, tlm_hz_*) and only reads for N seconds. The board
is disarmed, never armed by this script, no motor command is sent. Writes <out_prefix>.ground.log (the helper's log, copied),
<out_prefix>.console.log and <out_prefix>.json (the grants and the stream statistics decoded from the received bytes).
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / "telemetry"))
sys.path.insert(0, str(HERE.parents[2] / "link_board"))
import bench_telemetry as bt  # noqa: E402
import link_common as lc  # noqa: E402

SECONDS = 60  # labelled harness value: the packet's ~60 s


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_prefix")
    ap.add_argument("--seconds", type=float, default=SECONDS)
    args = ap.parse_args()
    out = Path(args.out_prefix)
    out.parent.mkdir(parents=True, exist_ok=True)
    d = lc.load_dialect()
    manifest = json.loads(lc.MANIFEST.read_text(encoding="utf-8"))
    console = lc.ConsoleReader()
    console.start()
    helper = lc.HelperProcess(out.parent / "gs-runs")
    res = {}
    pipe = None
    try:
        res["console_before"] = lc.wait_console(console)
        pipe = lc.WsPipe(helper)
        client = lc.Client(d, pipe)
        hb = client.heartbeat()
        res["armed"] = bool(hb.base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED)
        assert not res["armed"], "the board is armed: stop"
        ident = client.identity()
        res["identity"] = {"config_source": ident.config_source, "config_state": ident.config_state,
                           "config_refusal": ident.config_refusal, "config_sha256": bytes(ident.config_sha256).hex(),
                           "image_sha256": bytes(ident.image_sha256).hex()}
        client.list_all(manifest["count"])
        (t0, t1), stats = bt.stream_phase(client, pipe, d, args.seconds)
        res["streams"] = stats
        print("streams:", json.dumps(stats), flush=True)
        client.listen(2)
        hb = client.heartbeat()
        res["armed_after"] = bool(hb.base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED)
    finally:
        if pipe is not None:
            pipe.close()
        helper.stop()
        console.stop()
        Path(str(out) + ".console.log").write_bytes(bytes(console.raw))
        res["helper_output"] = helper.lines
        if helper.run_dir is not None and (helper.run_dir / "ground.log").exists():
            shutil.copy(helper.run_dir / "ground.log", str(out) + ".ground.log")
        Path(str(out) + ".json").write_text(json.dumps(res, indent=1, default=str) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
