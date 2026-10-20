#!/usr/bin/env python3
"""One real host-gz-quadfc build from the GCS (ground-segment unit GX-14, done-when 2). A test tool, never flown.

It starts the real helper (tools/gs/helper.py: this checkout, its build/ directory, the host's heavy lock, the default memory budget, the real
cmake and ninja), waits until the heavy lock is free (the helper refuses a build while it is held, H-3), and then sends the helper the commands the
GCS's Build panel sends, in the panel's order: build_status (the allow-list and the dry run), build {"preset": "host-gz-quadfc"} (the click),
the events until build_done, build_status again, and GET /api/images. It writes into --out-dir:
  status_before.json   the build_status answer before the click (the dry run: "N steps" or "not configured")
  build_reply.json     the helper's answer to the click
  events.jsonl         build_started and build_done as the helper pushed them, and the number of build_progress events (the output text is left out: see below)
  status_after.json    the build_status answer after the build (the dry run: "up to date")
  library_entry.json   the entry of GET /api/images for the tree host-gz-quadfc: its hash and its `build` {preset, describe, finished_utc}
  heavy_lock_during.txt the holder line of the heavy lock while the build ran (ci/heavy.sh --status)
The cmake and ninja output (the helper's own copy, <run dir>/builds/*.log, and the `tail` of the status) is not committed: the GS-3 word scan of the repository
flags the output, which names the link targets. The status JSON keeps `tail_lines` (how many lines the tail held) in its place.
The build tree is not removed afterwards; the lock is the helper's own and is released when the build ends; the helper's run directory (a temporary directory) is removed.

  uv run python tests/regression/quad/L10/results/gcs/gx14/real_build.py --out-dir DIR [--wait-lock SECONDS] [--timeout SECONDS]
"""

from __future__ import annotations

import argparse
import asyncio
import http.client
import json
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from websockets.asyncio.client import connect

ROOT = Path(__file__).resolve().parents[7]
PRESET = "host-gz-quadfc"
POLL_S = 5.0  # harness value: the poll period of the wait for the heavy lock


def without_output(obj):
    """The same JSON with every `tail` list (the build output the helper keeps) replaced by `tail_lines`, its length."""
    if isinstance(obj, dict):
        return {("tail_lines" if k == "tail" else k): (len(v) if k == "tail" and isinstance(v, list) else without_output(v)) for k, v in obj.items()}
    if isinstance(obj, list):
        return [without_output(v) for v in obj]
    return obj


def heavy_status() -> tuple[int, str]:
    done = subprocess.run(["bash", str(ROOT / "ci" / "heavy.sh"), "--status"], capture_output=True, text=True)
    return done.returncode, done.stdout.strip()


async def main(args) -> int:
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    runs = Path(tempfile.mkdtemp(prefix="gx14-helper-runs-"))  # outside the repository: CMake globs the source tree (CONFIGURE_DEPENDS), so a directory made there would make the next dry run report a re-run
    helper = subprocess.Popen([sys.executable, str(ROOT / "tools" / "gs" / "helper.py"), "--port", "0", "--runs-dir", str(runs), "--no-autoconnect"],
                              cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        url = None
        for line in helper.stdout:
            print("helper:", line.rstrip())
            m = re.match(r"GCS:\s+(http://127\.0\.0\.1:(\d+)/\?token=(\S+))", line)
            if m:
                url, port, token = m.group(1), int(m.group(2)), m.group(3)
                break
        if url is None:
            print("the helper did not start", file=sys.stderr)
            return 1
        origin = f"http://127.0.0.1:{port}"
        deadline = time.monotonic() + args.wait_lock
        while heavy_status()[0] != 0:
            if time.monotonic() > deadline:
                print("the heavy lock stayed held:", heavy_status()[1], file=sys.stderr)
                return 1
            print("waiting for the heavy lock:", heavy_status()[1], flush=True)
            await asyncio.sleep(POLL_S)
        async with connect(f"ws://127.0.0.1:{port}/ws?token={token}", origin=origin, compression=None, max_size=None) as ws:
            events, replies = [], asyncio.Queue()

            async def pump():
                async for m in ws:
                    if isinstance(m, bytes):
                        continue
                    d = json.loads(m)
                    (events.append(d) if "event" in d else replies.put_nowait(d))
            pump_task = asyncio.create_task(pump())

            async def cmd(**msg):
                await ws.send(json.dumps(msg))
                return await asyncio.wait_for(replies.get(), 600)

            before = await cmd(cmd="build_status", preset=PRESET)
            (out / "status_before.json").write_text(json.dumps(without_output(before), indent=1) + "\n")
            print("before:", before["build"]["status"], "refusals:", before["build"]["refusals"], flush=True)
            reply = await cmd(cmd="build", preset=PRESET)
            (out / "build_reply.json").write_text(json.dumps(without_output(reply), indent=1) + "\n")
            print("build reply ok:", reply["ok"], reply.get("error"), flush=True)
            if not reply["ok"]:
                return 1
            (out / "heavy_lock_during.txt").write_text(heavy_status()[1] + "\n")
            deadline = time.monotonic() + args.timeout
            while not any(e["event"] == "build_done" for e in events):
                if time.monotonic() > deadline:
                    print("the build did not end in time; cancelling", file=sys.stderr)
                    print(await cmd(cmd="build_cancel"))
                    return 1
                await asyncio.sleep(1)
            done = next(e for e in events if e["event"] == "build_done")
            kept = [e for e in events if e["event"] in ("build_started", "build_done")]
            kept.insert(1, {"event": "build_progress", "count": sum(e["event"] == "build_progress" for e in events)})
            (out / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in kept))
            after = await cmd(cmd="build_status", preset=PRESET)
            (out / "status_after.json").write_text(json.dumps(without_output(after), indent=1) + "\n")
            print("done:", {k: done[k] for k in ("ok", "exit_status", "error", "describe")}, "after:", after["build"]["status"], flush=True)
            c = http.client.HTTPConnection("127.0.0.1", port, timeout=60)
            c.request("GET", "/api/images", headers={"Host": f"127.0.0.1:{port}", "Authorization": f"Bearer {token}"})
            images = json.loads(c.getresponse().read())
            entry = [e for e in images["images"] if e["tree"] == PRESET]
            (out / "library_entry.json").write_text(json.dumps(entry, indent=1) + "\n")
            pump_task.cancel()
            print("library entry:", json.dumps(entry, indent=1))
            return 0 if done["ok"] and entry and entry[0]["build"] and entry[0]["build"]["describe"] == done["describe"] else 1
    finally:
        if helper.poll() is None:
            helper.terminate()
            try:
                helper.wait(30)
            except subprocess.TimeoutExpired:
                helper.kill()
        shutil.rmtree(runs, ignore_errors=True)  # the helper's run directory (its binary ground log) is not evidence; build.log is its copy of the output


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--wait-lock", type=float, default=0, help="wait this long for the heavy lock to be free (default: do not wait)")
    ap.add_argument("--timeout", type=float, default=7200, help="cancel the build after this long")
    sys.exit(asyncio.run(main(ap.parse_args())))
