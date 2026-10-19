"""The run supervisor (ground-segment unit GX-1, H-1 to H-3): one simulation run at a time, in its own process group, with a
manifest JSON per run, driven by the helper's run_plan, run_start and run_stop commands.

A run is one of two kinds:
  sil  tools/sim/run_quad_fc.py --interactive on the run directory (tools/gs/sil.py builds the command; the run relaunches on
       EX_TEMPFAIL, the helper tells the browser with a sil_relaunched event)
  hil  tools/sim/run_hil_s4.py --port <board port> --out-dir <dir> [--seconds S] [--expect E] [--gui]
Both are subprocesses started from an argv list, never through a shell, in a process group of their own that stop() signals.
Everything a browser supplies is validated here before it reaches an argv: only the options of the kind are accepted (unknown keys
are refused), a board port must be one of the allow-listed device globs and resolve under /dev, and a run id read back must
resolve under the runs directory.

The manifest, <run dir>/runs/<run id>/manifest.json, is written when the run starts and again when it ends: schema, run id, kind,
state (running, stopped, exited), argv, the options, the directories, UTC start and end, exit status, relaunch count. It never
holds the session token (the supervisor never receives it) nor the environment.
"""

from __future__ import annotations

import fnmatch
import json
import math
import os
import re
import sys
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from sil import ROOT, SilLauncher

KINDS = ("sil", "hil")
MANIFEST_SCHEMA = 1
HIL_EXPECT = ("s4a", "late", "none")  # tools/sim/run_hil_s4.py --expect choices
RUN_ID_RE = re.compile(r"^[0-9]{8}T[0-9]{6}Z-(sil|hil)-[0-9]+$")
OPTIONS = {"sil": (), "hil": ("port", "seconds", "expect", "gui")}


class RunRefused(Exception):
    """A request the supervisor will not carry out; str() is the reason."""


def resolve_device(path, globs: tuple[str, ...]) -> str:
    """A device path from the browser: a normalised absolute path matching one of the allow-listed globs whose target is under
    /dev/ (a symlink out of /dev, a ../ escape or a glob-shaped path elsewhere is refused). Returns the path unchanged."""
    if not isinstance(path, str) or not path or "\0" in path:
        raise ValueError("a device path must be a non-empty string")
    if os.path.normpath(path) != path or not os.path.isabs(path):
        raise ValueError(f"device path {path!r} must be absolute and normalised")
    if not any(fnmatch.fnmatchcase(path, g) for g in globs):
        raise ValueError(f"device path {path!r} is not one of: {', '.join(globs)}")
    if not os.path.realpath(path).startswith("/dev/"):
        raise ValueError(f"device path {path!r} does not resolve to a device under /dev/")
    return path


def resolve_under(path, roots: tuple[Path, ...]) -> Path:
    """The real path of `path`, which must be inside one of the allow-listed roots (symlinks and ../ resolved first)."""
    if not isinstance(path, (str, os.PathLike)) or not str(path) or "\0" in str(path):
        raise ValueError("a path must be a non-empty string")
    real = Path(os.path.realpath(path))
    for root in roots:
        base = Path(os.path.realpath(root))
        if real == base or base in real.parents:
            return real
    raise ValueError(f"path {str(path)!r} is outside the allowed roots")


def parse_options(kind: str, msg: dict, *, device_globs: tuple[str, ...]) -> dict:
    """The validated options of a run_plan or run_start message (ValueError names the offending key). `msg` has the cmd, id and
    kind keys besides the options; every other key must be an option of the kind."""
    if kind not in KINDS:
        raise ValueError(f"unknown run kind {kind!r}: {', '.join(KINDS)}")
    unknown = sorted(set(msg) - {"cmd", "id", "kind"} - set(OPTIONS[kind]))
    if unknown:
        raise ValueError(f"not an option of a {kind} run: {', '.join(unknown)}")
    opts: dict = {}
    if kind == "hil":
        if "port" not in msg:
            raise ValueError("a hil run needs the board's port (/dev/serial/by-id/...)")
        opts["port"] = resolve_device(msg["port"], device_globs)
        if "seconds" in msg:
            seconds = msg["seconds"]
            if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or seconds <= 0:
                raise ValueError("seconds must be a positive finite number")
            opts["seconds"] = float(seconds)
        if "expect" in msg:
            if msg["expect"] not in HIL_EXPECT:
                raise ValueError(f"expect must be one of {', '.join(HIL_EXPECT)}")
            opts["expect"] = msg["expect"]
        if "gui" in msg:
            if not isinstance(msg["gui"], bool):
                raise ValueError("gui must be true or false")
            opts["gui"] = msg["gui"]
    return opts


def hil_command(out_dir: Path, opts: dict) -> list[str]:
    command = [sys.executable, str(ROOT / "tools" / "sim" / "run_hil_s4.py"), "--port", opts["port"], "--out-dir", str(out_dir)]
    if "seconds" in opts:
        command += ["--seconds", repr(opts["seconds"])]
    if "expect" in opts:
        command += ["--expect", opts["expect"]]
    if opts.get("gui"):
        command.append("--gui")
    return command


def utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


class RunSupervisor:
    """`sil` is the helper's SilLauncher (its callbacks are set here). `emit(dict)` is called, from any thread, with each run_*
    and sil_relaunched event; `sil_command` / `hil_command` replace the scripts' command lines (tests)."""

    def __init__(self, run_dir: Path, sil: SilLauncher, emit: Callable[[dict], None], *,
                 hil_command_override: list[str] | None = None):
        self.run_dir = Path(run_dir)
        self.runs_dir = self.run_dir / "runs"
        self.sil = sil
        self._emit = emit
        self._hil_override = hil_command_override
        self._lock = threading.Lock()
        self._counter = 0
        self._current: dict | None = None  # {"id", "kind", "launcher", "manifest", "path"}
        self._last: dict | None = None  # the manifest of the latest run, ended or not

    # ---- state ----------------------------------------------------------------------------------------------------------

    def active(self) -> dict | None:
        """{"id", "kind"} of the run that is going (relaunch gaps included), else None."""
        cur = self._current
        if cur is None or cur["manifest"]["state"] != "running" or not cur["launcher"].active():
            return None
        return {"id": cur["id"], "kind": cur["kind"]}

    def busy(self) -> str | None:
        """The id of the run that is going, "(sil_start)" for a SIL launcher started without the supervisor, else None. A run that
        has ended (its manifest says so) is over even while its watcher thread is still returning."""
        active = self.active()
        if active is not None:
            return active["id"]
        cur = self._current
        ended_sil = cur is not None and cur["kind"] == "sil" and cur["manifest"]["state"] != "running"
        return "(sil_start)" if self.sil.active() and not ended_sil else None

    def status(self) -> dict:
        cur = self.active()
        return {"active": cur, "last": self._last}

    # ---- plan and start -------------------------------------------------------------------------------------------------

    def plan(self, kind: str, opts: dict) -> dict:
        """What a start would run, without starting (the run id is the next one; a second start may take it)."""
        run_id = self._next_id(kind, advance=False)
        out_dir = self.sil.out_dir if kind == "sil" else self.runs_dir / run_id / "out"
        if kind == "sil":
            argv = self.sil.command()
        else:
            argv = list(self._hil_override) if self._hil_override else hil_command(out_dir, opts)
        return {"kind": kind, "run_id": run_id, "argv": argv, "options": opts, "out_dir": str(out_dir),
                "udp_port": self.sil.udp_port if kind == "sil" else None}

    def _next_id(self, kind: str, *, advance: bool) -> str:
        n = self._counter + 1
        if advance:
            self._counter = n
        return f"{utc_stamp()}-{kind}-{n}"

    def start(self, kind: str, opts: dict) -> dict:
        with self._lock:
            if self.active() is not None:
                raise RunRefused(f"run {self._current['id']} is still going: one simulation at a time")
            plan = self.plan(kind, opts)
            run_id = self._next_id(kind, advance=True)
            plan["run_id"] = run_id
            launcher = self.sil if kind == "sil" else SilLauncher(self.run_dir, None, plan["argv"], out_dir=Path(plan["out_dir"]),
                                                                  relaunch=False)
            manifest = {"schema": MANIFEST_SCHEMA, "run_id": run_id, "kind": kind, "state": "running", "argv": plan["argv"],
                        "options": opts, "out_dir": plan["out_dir"], "run_dir": str(self.run_dir),
                        "store_dir": str(self.sil.config_dir) if kind == "sil" else None, "udp_port": plan["udp_port"],
                        "started_utc": utc_iso(), "ended_utc": None, "exit_status": None, "relaunches": 0}
            path = self.runs_dir / run_id / "manifest.json"
            cur = {"id": run_id, "kind": kind, "launcher": launcher, "manifest": manifest, "path": path}
            launcher.on_exit = lambda status: self._exited(cur, status)
            launcher.on_relaunch = lambda count: self._relaunched(cur, count)
            self._current, self._last = cur, manifest
            self._write(path, manifest)
            try:
                launcher.start()
            except (OSError, RuntimeError) as e:
                self._finish(cur, "exited", None)
                raise RunRefused(f"the run did not start: {e}") from None
            manifest["pid"] = launcher.status()["pid"]
            self._write(path, manifest)
        self._emit({"event": "run_started", "run_id": run_id, "kind": kind, "manifest": str(path)})
        return manifest

    def stop(self) -> dict:
        """Stops the current run (its whole process group); the manifest's state becomes stopped. RunRefused when none."""
        cur = self._current
        if cur is None or cur["manifest"]["state"] != "running":
            raise RunRefused("no run is going")
        status = cur["launcher"].stop()
        with self._lock:
            self._finish(cur, "stopped", status)
        self._emit({"event": "run_stopped", "run_id": cur["id"], "kind": cur["kind"], "exit_status": status})
        return cur["manifest"]

    def shutdown(self) -> None:
        """The helper is stopping: the same as stop() when a run is going; never raises."""
        try:
            self.stop()
        except RunRefused:
            pass

    # ---- callbacks (watcher thread) -------------------------------------------------------------------------------------

    def _relaunched(self, cur: dict, count: int) -> None:
        cur["manifest"]["relaunches"] = count
        self._write(cur["path"], cur["manifest"])
        self._emit({"event": "sil_relaunched", "run_id": cur["id"], "relaunches": count,
                    "pid": cur["launcher"].status()["pid"]})

    def _exited(self, cur: dict, status: int) -> None:
        with self._lock:
            if cur["manifest"]["state"] != "running":
                return
            self._finish(cur, "exited", status)
        self._emit({"event": "run_exited", "run_id": cur["id"], "kind": cur["kind"], "exit_status": status})

    def _finish(self, cur: dict, state: str, status: int | None) -> None:
        m = cur["manifest"]
        m.update(state=state, ended_utc=utc_iso(), exit_status=status)
        self._write(cur["path"], m)

    # ---- manifests ------------------------------------------------------------------------------------------------------

    @staticmethod
    def _write(path: Path, manifest: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + ".tmp")
        tmp.write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8")
        os.replace(tmp, path)

    def list_runs(self) -> list[str]:
        if not self.runs_dir.is_dir():
            return []
        return sorted(p.name for p in self.runs_dir.iterdir() if RUN_ID_RE.match(p.name) and (p / "manifest.json").is_file())

    def manifest(self, run_id) -> dict:
        """The manifest of a run of this helper session; run_id must be a run id and resolve under the runs directory."""
        if not isinstance(run_id, str) or not RUN_ID_RE.match(run_id):
            raise ValueError(f"{run_id!r} is not a run id")
        path = resolve_under(self.runs_dir / run_id / "manifest.json", (self.runs_dir,))
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except OSError:
            raise ValueError(f"no run {run_id}") from None
