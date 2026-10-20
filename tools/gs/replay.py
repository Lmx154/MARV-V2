"""The replay of a ground log (ground spec GS-8, ground-segment unit GX-11): a recorded session is fed to the GCS through the path a live
device's bytes take, so that the vehicle panels show the recorded session.

What is replayed. The RX records of a ground log (tools/gs/groundlog.py, the file `ground.log` of a run directory): the bytes the device
sent, in order, each chunk as the operating system handed it over. They go to the browser as binary frames of /ws, unchanged, so the GCS's
codec, session and telemetry client decode them exactly as they decode a live device. Nothing is parsed here (GS-3). The TX records are
what the GCS sent to the vehicle in that session; they are never sent anywhere. The EVENT records are not replayed.

Never to a vehicle. A replay has no device: it is refused while the helper has one open (a live vehicle, or the SIL's UDP link) and while a
run is going (the SIL's autoconnect would open its link), autoconnect is suspended while it plays, and a binary frame the browser sends
meanwhile is counted and dropped: it reaches no device and the ground log. Replayed bytes are not written to the ground log of the
running session either (only EVENT records name the replay), so a replay cannot be mistaken for a recording.

Pace. "paced" sends each chunk at its recorded time relative to the first chunk (the log's monotonic clock); "fast" sends them as fast as the
browser takes them, never dropping one (the helper's usual queue limit drops chunks for a slow browser; a replay waits instead).

Which logs. `<runs dir>/<run>/ground.log` of every run directory of the helper's runs directory (including this session's own, as far as it
is written) and of every --replay-dir; the browser names a log by the path the listing gave and the helper accepts only a listed path.

Commands (helper.py): replay_list, replay_start {"log": path, "mode": "paced"|"fast"}, replay_stop, replay_status. Pushed:
{"event": "replay_state", "state": "playing"|"finished"|"stopped"|"failed", ...status} when the state changes, and while playing at most
every PROGRESS_S.
"""

from __future__ import annotations

import asyncio
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import groundlog
from device import DeviceError

LOG_NAME = "ground.log"
MODES = ("paced", "fast")
NS_PER_S = 1_000_000_000  # a definition
PROGRESS_S = 0.25  # labelled harness value: the longest a playing replay goes without telling the browser where it is
ROOM_POLL_S = 0.002  # labelled harness value: how long a replay waits for room in the browser's queue before looking again


class ReplayError(DeviceError):
    """A replay command the helper refuses: `extra` is merged into the reply (the list of reasons)."""

    def __init__(self, message: str, extra: dict | None = None):
        super().__init__(message)
        self.extra = extra or {}


def summarise(path: Path) -> dict:
    """What the listing says of a ground log: the time it was created, its RX chunks and bytes, the span of the RX chunks, the TX and EVENT
    counts. A file that is not a ground log has "error" instead."""
    entry: dict = {"path": str(path), "name": path.parent.name, "size": path.stat().st_size}
    try:
        header = groundlog.read_header(path)
        count = {groundlog.RX: 0, groundlog.TX: 0, groundlog.EVENT: 0}
        rx_bytes = 0
        first = last = None
        for r in groundlog.read_records(path):
            count[r.kind] += 1
            if r.kind == groundlog.RX:
                rx_bytes += len(r.data)
                first = r.t_ns if first is None else first
                last = r.t_ns
    except (ValueError, OSError) as e:
        entry["error"] = str(e)
        return entry
    entry.update({
        "wall_utc": datetime.fromtimestamp(header.wall_ns / NS_PER_S, timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "rx_records": count[groundlog.RX], "rx_bytes": rx_bytes, "tx_records": count[groundlog.TX], "events": count[groundlog.EVENT],
        "duration_s": 0.0 if first is None else (last - first) / NS_PER_S,
    })
    return entry


def load_rx(path: Path) -> list[tuple[int, bytes]]:
    return [(r.t_ns, r.data) for r in groundlog.read_records(path) if r.kind == groundlog.RX]


class Replay:
    """The replay slot of a Helper (duck-typed: run_dir, replay_dirs, device, runs, _queue, _queued_bytes, log, _push). `queue_limit`: the
    helper's limit of bytes queued for the browser (helper.py CLIENT_QUEUE_MAX_BYTES), at which a live device's chunks are dropped and a
    replay waits."""

    def __init__(self, helper, queue_limit: int):
        self.h = helper
        self._queue_limit = queue_limit
        self._task: asyncio.Task | None = None
        self._cache: dict[tuple[str, int, int], dict] = {}
        self.state = "idle"  # idle | playing | finished | stopped | failed
        self.reason: str | None = None
        self.path: str | None = None
        self.mode: str | None = None
        self.records_total = self.records_sent = self.bytes_total = self.bytes_sent = 0
        self.position_s = self.duration_s = 0.0
        self.tx_dropped_bytes = 0
        self._progress_at = 0.0

    @property
    def active(self) -> bool:
        return self.state == "playing"

    # ---- listing ----------------------------------------------------------------------------------------------------------

    def _candidates(self) -> list[Path]:
        roots = [self.h.run_dir.parent, *self.h.replay_dirs]
        seen: dict[str, Path] = {}
        for root in roots:
            try:
                children = sorted(root.iterdir())
            except OSError:
                continue
            for child in children:
                log = child / LOG_NAME
                if log.is_file() and not log.is_symlink():
                    seen.setdefault(str(log.resolve()), log.resolve())
        return list(seen.values())

    def list_logs(self) -> dict:
        """Every ground log the helper may replay, oldest first, with its summary (cached by size and modification time)."""
        current = str((self.h.run_dir / LOG_NAME).resolve())
        out = []
        for path in self._candidates():
            stat = path.stat()
            key = (str(path), stat.st_size, stat.st_mtime_ns)
            if key not in self._cache:
                self._cache[key] = summarise(path)
            out.append({**self._cache[key], "current": str(path) == current})
        out.sort(key=lambda e: (e.get("wall_utc", ""), e["path"]))
        return {"logs": out, "replay": self.status()}

    def _resolve(self, value) -> Path:
        if not isinstance(value, str):
            raise ReplayError("replay_start needs \"log\": a path from replay_list")
        for path in self._candidates():
            if str(path) == value:
                return path
        raise ReplayError(f"{value!r} is not a ground log the helper lists: choose one from replay_list")

    # ---- refusals ---------------------------------------------------------------------------------------------------------

    def refusals(self) -> list[str]:
        h = self.h
        reasons = []
        if self.active:
            reasons.append("a replay is going: stop it first")
        if h.device is not None:
            reasons.append(f"the helper has a device open ({h.device.description}): close it first; a replay never shares the link with a live vehicle")
        active = h.runs.active()
        if active is not None:
            reasons.append(f"run {active['id']} ({active['kind']}) is going: stop it first; its link would be opened beside the replay")
        return reasons

    # ---- control ----------------------------------------------------------------------------------------------------------

    async def start(self, msg: dict) -> dict:
        mode = msg.get("mode", "paced")
        if mode not in MODES:
            raise ReplayError(f"mode must be one of {', '.join(MODES)}")
        reasons = self.refusals()
        if reasons:
            self.h.log.event(f"replay refused: {'; '.join(reasons)}")
            raise ReplayError(reasons[0], {"refusals": reasons})
        path = self._resolve(msg.get("log"))
        try:
            records = await asyncio.to_thread(load_rx, path)
        except (ValueError, OSError) as e:
            raise ReplayError(f"{path} cannot be read as a ground log: {e}") from None
        if not records:
            raise ReplayError(f"{path} holds no received bytes: there is nothing to replay")
        reasons = self.refusals()  # the helper's state may have changed while the file was read
        if reasons:
            raise ReplayError(reasons[0], {"refusals": reasons})
        self.path, self.mode, self.reason = str(path), mode, None
        self.records_total, self.records_sent = len(records), 0
        self.bytes_total, self.bytes_sent = sum(len(d) for _, d in records), 0
        self.position_s, self.duration_s = 0.0, (records[-1][0] - records[0][0]) / NS_PER_S
        self.tx_dropped_bytes = 0
        self.state = "playing"
        self.h.log.event(f"replay started: {path} {mode}, {self.records_total} chunks, {self.bytes_total} bytes")
        self._announce()
        self._task = asyncio.create_task(self._play(records, mode))
        return {"replay": self.status()}

    async def stop(self, reason: str = "stopped by request") -> dict:
        task = self._task
        if task is not None and not task.done():
            self.reason = reason
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        return {"replay": self.status()}

    def discard_tx(self, n: int) -> None:
        """Bytes the browser sent during a replay: dropped, counted."""
        self.tx_dropped_bytes += n

    def status(self) -> dict:
        return {"state": self.state, "reason": self.reason, "log": self.path, "mode": self.mode,
                "records_total": self.records_total, "records_sent": self.records_sent,
                "bytes_total": self.bytes_total, "bytes_sent": self.bytes_sent,
                "position_s": self.position_s, "duration_s": self.duration_s, "tx_dropped_bytes": self.tx_dropped_bytes}

    # ---- playing ----------------------------------------------------------------------------------------------------------

    def _announce(self) -> None:
        self.h._push(json.dumps({"event": "replay_state", **self.status()}))

    async def _play(self, records: list[tuple[int, bytes]], mode: str) -> None:
        h = self.h
        loop = asyncio.get_running_loop()
        first = records[0][0]
        started = loop.time()
        final, reason = "finished", None
        try:
            for t_ns, data in records:
                if mode == "paced":
                    delay = started + (t_ns - first) / NS_PER_S - loop.time()
                    if delay > 0:
                        await asyncio.sleep(delay)
                if not await self._send(data):
                    final, reason = "stopped", "the browser went away"
                    break
                self.records_sent += 1
                self.bytes_sent += len(data)
                self.position_s = (t_ns - first) / NS_PER_S
                now = time.monotonic()
                if now - self._progress_at >= PROGRESS_S:
                    self._progress_at = now
                    self._announce()
                await asyncio.sleep(0)  # the event loop serves the browser's commands (replay_stop) between chunks
        except asyncio.CancelledError:
            final, reason = "stopped", self.reason or "stopped"
        except Exception as e:  # a failed replay says so and ends; it must not take the helper down
            final, reason = "failed", f"{type(e).__name__}: {e}"
        self.state, self.reason = final, reason
        h.log.event(f"replay {final}: {self.path}, {self.records_sent} of {self.records_total} chunks"
                    + (f" ({reason})" if reason else ""))
        self._announce()

    async def _send(self, data: bytes) -> bool:
        """Queues the chunk for the browser behind everything before it, waiting while the queue is full; False if the browser is gone."""
        h = self.h
        while True:
            queue = h._queue
            if queue is None:
                return False
            if h._queued_bytes == 0 or h._queued_bytes + len(data) <= self._queue_limit:
                h._queued_bytes += len(data)
                queue.put_nowait(data)
                return True
            await asyncio.sleep(ROOM_POLL_S)
