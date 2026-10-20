"""The run supervisor (ground-segment unit GX-1, H-1 to H-3): one simulation run at a time, in its own process group, with a
manifest JSON per run, driven by the helper's run_plan, run_start and run_stop commands.

A run is one of two kinds:
  sil  tools/sim/run_quad_fc.py --interactive on the run directory (tools/gs/sil.py builds the command; the run relaunches on
       EX_TEMPFAIL, the helper tells the browser with a sil_relaunched event). Its run definition (GX-3) is the options "card"
       (a repo-relative vehicles/<name>.yaml), "overrides" ([{"name", "value"}]: firmware parameter values in the SIL, checked
       against the image's parameter manifest by tools/sim/sil_run.py) and "gui" (the display-only visuals and a gz GUI client);
       a run with an override is labelled "sil-override" in the plan, the manifest and the run_started event (invariant O-1: the
       validator refuses a config that carries the label)
  hil  tools/sim/run_hil_s4.py --port <board port> --out-dir <dir> [--seconds S] [--expect E] [--gui]
A sil run also accepts "pilot" (a boolean, default false; ground-segment unit GX-9): the run is given the helper's pilot port
(--pilot-udp-port), its world has <pilot_udp_port> and no stick script, and the pilot bridge (tools/gs/pilot.py) sends the
RadioMaster's frames to that port while a joystick is open; false leaves the world byte-identical to one without the option.
Both kinds also accept "sim_channel" (a boolean, default true): when the helper has a sim channel listener (tools/gs/simchannel.py,
GX-8) the run gets --sim-telemetry-udp-port <its port>, so that the plugin publishes the sim record the helper relays on /ws/sim;
false leaves the option out and the world byte-identical to one without a sim channel.
Both are subprocesses started from an argv list, never through a shell, in a process group of their own that stop() signals.
Everything a browser supplies is validated here before it reaches an argv: only the options of the kind are accepted (unknown keys
are refused), a board port must be one of the allow-listed device globs and resolve under /dev, a card must be a file of the
vehicles directory, an override must name a parameter of the image with a value in its type's domain, and a run id read back must
resolve under the runs directory.

The manifest, <run dir>/runs/<run id>/manifest.json, is written when the run starts and again when it ends: schema, run id, kind,
state (running, stopped, exited), argv, the options, the label ("sil-override" or null) and the overrides, the directories, UTC
start and end, exit status, relaunch count. It never
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

sys.path.insert(0, str(ROOT / "tools" / "sim"))
import sil_run  # noqa: E402

KINDS = ("sil", "hil")
MANIFEST_SCHEMA = 1
HIL_EXPECT = ("s4a", "late", "none")  # tools/sim/run_hil_s4.py --expect choices
RUN_ID_RE = re.compile(r"^[0-9]{8}T[0-9]{6}Z-(sil|hil)-[0-9]+$")
OPTIONS = {"sil": ("card", "overrides", "gui", "pilot", "sim_channel"), "hil": ("port", "seconds", "expect", "gui", "sim_channel")}
CARD_DIR = ROOT / "vehicles"
OVERRIDE_KEYS = ("name", "value")


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


def resolve_card(path) -> str:
    """The card of a sil run from the browser: a file `vehicles/<name>.yaml` of the repository (a bare `<name>.yaml` too), nothing
    else (a symlink or ../ out of the vehicles directory, a subdirectory or another suffix is refused). Returns the
    repository-relative path."""
    if not isinstance(path, str) or not path or "\0" in path:
        raise ValueError("card must be a non-empty string")
    name = path[len("vehicles/"):] if path.startswith("vehicles/") else path
    unknown = ValueError(f"unknown card {path!r}: a file of the vehicles directory")
    if "/" in name or name.startswith("."):
        raise unknown
    try:
        real = resolve_under(CARD_DIR / name, (CARD_DIR,))
    except ValueError:
        raise unknown from None
    if real.parent != Path(os.path.realpath(CARD_DIR)) or real.suffix != ".yaml" or not real.is_file():
        raise unknown
    return f"vehicles/{real.name}"


def list_cards() -> list[str]:
    """The cards a sil run accepts: the repository-relative `vehicles/<name>.yaml` files, sorted (each one passes resolve_card)."""
    names = []
    for path in sorted(CARD_DIR.glob("*.yaml")):
        try:
            names.append(resolve_card(f"vehicles/{path.name}"))
        except ValueError:
            continue
    return names


def parse_sil_options(msg: dict, param_types: Callable[[], dict[str, str]] | None) -> dict:
    opts: dict = {}
    if "card" in msg:
        opts["card"] = resolve_card(msg["card"])
    if "overrides" in msg:
        items = msg["overrides"]
        if not isinstance(items, list):
            raise ValueError("overrides must be a list of {name, value}")
        pairs = []
        for item in items:
            if not isinstance(item, dict) or set(item) != set(OVERRIDE_KEYS):
                raise ValueError(f"an override is exactly {{{', '.join(OVERRIDE_KEYS)}}}")
            if isinstance(item["value"], bool) or not isinstance(item["value"], (int, float)):
                raise ValueError(f"override {item['name']}: the value must be a JSON number")
            pairs.append((item["name"], item["value"]))
        if pairs:
            if param_types is None:
                raise ValueError("this helper has no parameter manifest to check overrides against")
            opts["overrides"] = sil_run.check_overrides(pairs, param_types())
    if "gui" in msg:
        if not isinstance(msg["gui"], bool):
            raise ValueError("gui must be true or false")
        opts["gui"] = msg["gui"]
    if "pilot" in msg:
        if not isinstance(msg["pilot"], bool):
            raise ValueError("pilot must be true or false")
        opts["pilot"] = msg["pilot"]
    return opts


def parse_sim_channel(msg: dict, opts: dict) -> None:
    if "sim_channel" in msg:
        if not isinstance(msg["sim_channel"], bool):
            raise ValueError("sim_channel must be true or false")
        opts["sim_channel"] = msg["sim_channel"]


def sim_channel_args(opts: dict, sim_port: int | None) -> list[str]:
    """The sim channel option of a run: the helper's listener port, unless the run definition turned the channel off."""
    return ["--sim-telemetry-udp-port", str(sim_port)] if sim_port is not None and opts.get("sim_channel", True) else []


def sil_args(opts: dict, sim_port: int | None = None, pilot_port: int | None = None) -> list[str]:
    """The run_quad_fc.py options of a sil run's run definition."""
    args: list[str] = sim_channel_args(opts, sim_port)
    if opts.get("pilot") and pilot_port is not None:
        args += ["--pilot-udp-port", str(pilot_port)]
    if "card" in opts:
        args += ["--card", str(ROOT / opts["card"])]
    for o in opts.get("overrides", []):
        args += ["--override", sil_run.cli_arg(o)]
    if opts.get("gui"):
        args.append("--gui")
    return args


def label_of(opts: dict) -> str | None:
    return sil_run.LABEL if opts.get("overrides") else None


def parse_options(kind: str, msg: dict, *, device_globs: tuple[str, ...],
                  param_types: Callable[[], dict[str, str]] | None = None) -> dict:
    """The validated options of a run_plan or run_start message (ValueError names the offending key). `msg` has the cmd, id and
    kind keys besides the options; every other key must be an option of the kind. `param_types` reads the image's parameter
    names and types (tools/sim/sil_run.py), called only when a sil run has overrides."""
    if kind not in KINDS:
        raise ValueError(f"unknown run kind {kind!r}: {', '.join(KINDS)}")
    unknown = sorted(set(msg) - {"cmd", "id", "kind"} - set(OPTIONS[kind]))
    if unknown:
        raise ValueError(f"not an option of a {kind} run: {', '.join(unknown)}")
    opts: dict = {}
    if kind == "sil":
        opts = parse_sil_options(msg, param_types)
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
    parse_sim_channel(msg, opts)
    return opts


def hil_command(out_dir: Path, opts: dict, sim_port: int | None = None) -> list[str]:
    command = [sys.executable, str(ROOT / "tools" / "sim" / "run_hil_s4.py"), "--port", opts["port"], "--out-dir", str(out_dir),
               *sim_channel_args(opts, sim_port)]
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
                 hil_command_override: list[str] | None = None, sim_port: Callable[[], int | None] | None = None,
                 pilot_port: Callable[[], int | None] | None = None):
        self._sim_port = sim_port or (lambda: None)  # the port of the helper's sim channel listener, None when it has none
        self._pilot_port = pilot_port or (lambda: None)  # the port the pilot bridge sends to, None when the helper has none
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

    def active_pilot_port(self) -> int | None:
        """The pilot port of the sil run that is going, if it was started with the pilot option, else None."""
        cur = self._current
        if self.active() is None or cur["kind"] != "sil":
            return None
        return cur["manifest"].get("pilot_port")

    # ---- plan and start -------------------------------------------------------------------------------------------------

    def plan(self, kind: str, opts: dict) -> dict:
        """What a start would run, without starting (the run id is the next one; a second start may take it)."""
        run_id = self._next_id(kind, advance=False)
        out_dir = self.sil.out_dir if kind == "sil" else self.runs_dir / run_id / "out"
        if kind == "sil":
            argv = self.sil.command(sil_args(opts, self._sim_port(), self._pilot_port()))
        else:
            argv = list(self._hil_override) if self._hil_override else hil_command(out_dir, opts, self._sim_port())
        return {"kind": kind, "run_id": run_id, "argv": argv, "options": opts, "label": label_of(opts),
                "overrides": list(opts.get("overrides", [])), "out_dir": str(out_dir),
                "udp_port": self.sil.udp_port if kind == "sil" else None,
                "sim_channel_port": self._sim_port() if opts.get("sim_channel", True) else None,
                "pilot_port": self._pilot_port() if kind == "sil" and opts.get("pilot") else None}

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
            if kind == "sil":
                self.sil.extra_args = sil_args(opts, self._sim_port(), self._pilot_port())
            launcher = self.sil if kind == "sil" else SilLauncher(self.run_dir, None, plan["argv"], out_dir=Path(plan["out_dir"]),
                                                                  relaunch=False)
            manifest = {"schema": MANIFEST_SCHEMA, "run_id": run_id, "kind": kind, "state": "running", "argv": plan["argv"],
                        "options": opts, "label": plan["label"], "overrides": plan["overrides"], "out_dir": plan["out_dir"], "run_dir": str(self.run_dir),
                        "store_dir": str(self.sil.config_dir) if kind == "sil" else None, "udp_port": plan["udp_port"],
                        "sim_channel_port": plan["sim_channel_port"], "pilot_port": plan["pilot_port"],
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
        self._emit({"event": "run_started", "run_id": run_id, "kind": kind, "label": plan["label"], "manifest": str(path)})
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
