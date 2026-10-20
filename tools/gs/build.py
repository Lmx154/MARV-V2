"""Builds from the GCS (ground-segment unit GX-14, decision "GCS-triggered builds under the heavy lock", H-2, H-3).

The helper may build, but only
  - the presets of PRESETS (host-gz-quadfc, m33, m33-hil), by exact name: the browser sends a name, never an argument, and the helper
    builds argv lists and calls no shell;
  - on an explicit `build` command: nothing else in the helper builds (a launch never builds, H-3).
A build takes the host-wide heavy lock itself, the way ci/heavy.sh does: one flock(2) lock, LOCK_EX without waiting, on the same file
(hoststate.heavy_lock_path()), the holder line written into it, MARV_HEAVY_HELD exported to the build. The helper holds it from before
the first step until the last process of the build is gone, so an agent's `ci/heavy.sh` job, ci/dev_check.sh, ci/local_ci.sh and a GCS
build exclude one another. A build is refused when the lock is held, when free memory (MemAvailable) is below BUILD_MIN_FREE_BYTES, or when
another build is running.

A build is two steps, each its own process group (start_new_session): `cmake --preset P -B <tree> [-D...]`, then
`cmake --build <tree> [--target ...]`. Ninja puts each compiler in a group of its own, so cancel does not rely on the group alone: it
snapshots the descendants of the step's leader, sends SIGTERM to the group and to each of them, waits BUILD_STOP_S and sends SIGKILL to
what is left; the lock is released after that, and `build_done` is pushed.

Status is `ninja -n -C <tree> [targets]` (a dry run): "up to date" or "N steps" ("not configured" before the first configure).

The describe string, `git describe --dirty --always` of the source tree when the build starts, is recorded in <tree>/gs-build.json with the
hashes of the images the build produced; tools/gs/artifacts.py puts it in the library entry of each (`build`: preset, describe,
finished_utc).

Commands (tools/gs/helper.py): {"cmd": "build_status", "preset"?}, {"cmd": "build", "preset"}, {"cmd": "build_cancel"}. Events:
{"event": "build_started", ...}, {"event": "build_progress", "preset", "line", "step", "steps"} (at most one per BUILD_PUSH_S; the whole output is
in <run dir>/builds/<UTC>-<preset>.log) and {"event": "build_done", "preset", "ok", "cancelled", "exit_status", "error", "describe", "images", ...}.
"""

from __future__ import annotations

import asyncio
import errno
import fcntl
import json
import os
import re
import signal
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

import hoststate
from device import DeviceError
from hil import HIL_CACHE, HIL_TREE

BUILD_RECORD = "gs-build.json"  # in the build tree: what the last GCS build recorded (artifacts.py reads it)
RECORD_SCHEMA = 1
# PLACEHOLDER (design budget, unmeasured): a build starts only when this much memory is available (MemAvailable). The oomd risk is the
# reason for the heavy lock; the number is a guess at the peak of a cold gz-sim plugin build (31 GiB host). It is replaced by the peak
# ci/peakmem.sh measures for a cold host-gz-quadfc build; until then it blocks nothing but a starved host. Decides no result.
BUILD_MIN_FREE_BYTES = 8 << 30
# PLACEHOLDER (harness value): how long a cancelled build gets after SIGTERM before SIGKILL; ninja interrupts its jobs within moments.
BUILD_STOP_S = 10.0
# PLACEHOLDER (harness value): the longest `ninja -n` the status waits for; a dry run of a large tree takes seconds.
NINJA_DRY_RUN_TIMEOUT_S = 120.0
# PLACEHOLDER (harness value): at most one build_progress event per this long (the last line is always sent when a step ends).
BUILD_PUSH_S = 0.1
# PLACEHOLDER (harness value): the last output lines the status keeps for the GCS; the whole output is in the build log.
BUILD_TAIL_LINES = 40
# PLACEHOLDER (harness value): how long the reader may drain a step's pipe after the step's leader has ended (a process that
# escaped the group holds the pipe open).
DRAIN_S = 1.0
READ_SIZE = 65536  # implementation constant: bytes per pipe read
POLL_S = 0.05  # labelled harness value: the poll period of the cancel's wait for the processes to go
STEP_RE = re.compile(r"^\[(\d+)/(\d+)\]")
NOTHING_TO_DO = "ninja: no work to do."


@dataclass(frozen=True)
class BuildPreset:
    configure: str  # the CMake configure preset
    tree: str  # the build tree under the build root
    cache: tuple[tuple[str, str], ...] = ()  # -D entries of the configure step
    targets: tuple[str, ...] = ()  # the build's targets; empty: the preset's default (all)


# The allow-list: the exact names the browser may send. m33-hil is the HIL receiver image of tools/sim/run_hil_s4.py (hil.py HIL_TREE).
PRESETS: dict[str, BuildPreset] = {
    "host-gz-quadfc": BuildPreset("host-gz-quadfc", "host-gz-quadfc"),
    "m33": BuildPreset("m33", "m33"),
    "m33-hil": BuildPreset("m33", HIL_TREE, tuple(sorted(HIL_CACHE.items())), ("marv_fc3v2",)),
}


class BuildRefused(DeviceError):
    """A build command the helper refuses: `extra` is merged into the reply (the list of reasons)."""

    def __init__(self, reasons: list[str]):
        super().__init__(reasons[0])
        self.extra = {"refusals": reasons}


def utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def git_describe(root: Path) -> str:
    """`git describe --dirty --always` of the source tree (--always: the abbreviated commit when no tag is reachable)."""
    done = subprocess.run(["git", "-C", str(root), "describe", "--dirty", "--always"], capture_output=True, text=True, timeout=NINJA_DRY_RUN_TIMEOUT_S)
    if done.returncode != 0 or not done.stdout.strip():
        raise OSError(f"git describe failed: {(done.stderr or done.stdout).strip()}")
    return done.stdout.strip()


def parse_dry_run(returncode: int, output: str) -> dict:
    """The status of a `ninja -n`: {"kind": "up_to_date"|"steps"|"error", "steps": N|None, "text"}."""
    lines = output.splitlines()
    if returncode != 0:
        detail = next((ln for ln in reversed(lines) if ln.strip()), "no output")
        return {"kind": "error", "steps": None, "text": f"ninja -n failed ({returncode}): {detail}"}
    if any(ln.strip() == NOTHING_TO_DO for ln in lines):
        return {"kind": "up_to_date", "steps": None, "text": "up to date"}
    totals = [int(m.group(2)) for m in map(STEP_RE.match, lines) if m]
    if not totals:
        return {"kind": "error", "steps": None, "text": "ninja -n gave neither 'no work to do' nor build steps"}
    n = max(totals)
    return {"kind": "steps", "steps": n, "text": f"{n} step" if n == 1 else f"{n} steps"}


def proc_table() -> dict[int, tuple[int, str, str]]:
    """pid -> (ppid, start time, state) of every process, from /proc/<pid>/stat."""
    table = {}
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            text = Path(entry.path, "stat").read_text()
        except OSError:
            continue
        rest = text[text.rindex(")") + 2:].split()  # after "pid (comm) ": state ppid ... starttime is field 22 overall
        table[int(entry.name)] = (int(rest[1]), rest[19], rest[0])
    return table


def descendants(root_pid: int, table: dict | None = None) -> dict[int, str]:
    """pid -> start time of every living descendant of root_pid (a zombie is not living)."""
    table = proc_table() if table is None else table
    children: dict[int, list[int]] = {}
    for pid, (ppid, _, _) in table.items():
        children.setdefault(ppid, []).append(pid)
    found, stack = {}, [root_pid]
    while stack:
        for child in children.get(stack.pop(), []):
            if child not in found and table[child][2] != "Z":
                found[child] = table[child][1]
                stack.append(child)
    return found


def _alive(pid: int, start: str) -> bool:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
    except OSError:
        return False
    rest = stat[stat.rindex(")") + 2:].split()
    return rest[19] == start and rest[0] != "Z"


class BuildManager:
    def __init__(self, *, root: Path, build_root: Path, run_dir: Path, lock_path: Callable[[], Path], library, log: Callable[[str], None],
                 push: Callable[[dict], None], min_free_bytes: int = BUILD_MIN_FREE_BYTES):
        self.root, self.build_root, self.run_dir = Path(root), Path(build_root), Path(run_dir)
        self.lock_path, self.library, self.log, self.push = lock_path, library, log, push
        self.min_free_bytes = min_free_bytes
        self.stop_s = BUILD_STOP_S
        self.state: dict | None = None  # the running or the latest build
        self._task: asyncio.Task | None = None
        self._proc: asyncio.subprocess.Process | None = None
        self._cancel = False
        self._lock_fd: int | None = None

    # ---- what the browser may ask ---------------------------------------------------------------------------------------

    @staticmethod
    def allowed() -> list[dict]:
        return [{"preset": name, "tree": p.tree, "configure": p.configure, "cache": dict(p.cache), "targets": list(p.targets)} for name, p in PRESETS.items()]

    @staticmethod
    def preset_of(name) -> BuildPreset:
        if not isinstance(name, str) or name not in PRESETS:
            shown = name if isinstance(name, str) and len(name) <= 64 else type(name).__name__
            raise BuildRefused([f"the preset {shown!r} is not on the build allow-list: {', '.join(PRESETS)}"])
        return PRESETS[name]

    def busy(self) -> bool:
        return self._task is not None and not self._task.done()

    def refusals(self) -> list[str]:
        """Every reason a build cannot start now; empty when it can."""
        reasons = []
        if self.busy():
            reasons.append(f"a build is running ({self.state['preset']}): one build at a time")
        lock = hoststate.heavy_lock_state(self.lock_path())
        if lock["held"]:
            reasons.append(f"the heavy lock is held ({lock['holder'] or 'unknown holder'}): heavy work waits for it (H-3)")
        free = hoststate.meminfo()["mem_available_bytes"]
        if free is None:
            reasons.append("free memory cannot be read: a build needs it to be known")
        elif free < self.min_free_bytes:
            reasons.append(f"free memory {free} B is below the build budget {self.min_free_bytes} B (PLACEHOLDER)")
        return reasons

    def tree_dir(self, preset: BuildPreset) -> Path:
        return self.build_root / preset.tree

    async def status(self, name) -> dict:
        """The build block: the allow-list, the limits, the running or the latest build and, for `name`, the dry-run status and the
        refusals a build would meet now."""
        block = {"allowed": self.allowed(), "running": self.busy(), "current": self.view(),
                 "limits": {"min_free_bytes": self.min_free_bytes, "mem_available_bytes": hoststate.meminfo()["mem_available_bytes"]},
                 "preset": None, "status": None, "refusals": None}
        if name is not None:
            preset = self.preset_of(name)
            block.update(preset=name, refusals=self.refusals(), status=await self.dry_run(preset))
        return block

    async def dry_run(self, preset: BuildPreset) -> dict:
        tree = self.tree_dir(preset)
        if not (tree / "build.ninja").is_file():
            return {"kind": "not_configured", "steps": None, "text": "not configured"}
        argv = ["ninja", "-n", "-C", str(tree), *preset.targets]
        try:
            proc = await asyncio.create_subprocess_exec(*argv, stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
                                                        stderr=asyncio.subprocess.STDOUT, start_new_session=True)
        except OSError as e:
            return {"kind": "error", "steps": None, "text": f"ninja did not start: {e}"}
        try:
            out, _ = await asyncio.wait_for(proc.communicate(), NINJA_DRY_RUN_TIMEOUT_S)
        except asyncio.TimeoutError:
            await self._kill_group(proc)
            return {"kind": "error", "steps": None, "text": f"ninja -n did not end within {NINJA_DRY_RUN_TIMEOUT_S} s"}
        return parse_dry_run(proc.returncode, out.decode("utf-8", "replace"))

    def view(self) -> dict | None:
        return None if self.state is None else dict(self.state)

    # ---- the heavy lock, the way ci/heavy.sh takes it -------------------------------------------------------------------

    def _take_lock(self, name: str) -> int:
        path = self.lock_path()
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o666)  # ci/heavy.sh: exec 9>>"${lock}"
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as e:
            os.close(fd)
            if e.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
                raise BuildRefused([f"the heavy lock {path} cannot be taken: {e}"]) from None
            holder = hoststate.heavy_lock_state(path)["holder"]
            raise BuildRefused([f"the heavy lock is held ({holder or 'unknown holder'}): heavy work waits for it (H-3)"]) from None
        user = os.environ.get("USER") or str(os.getuid())
        os.ftruncate(fd, 0)
        os.write(fd, f"pid={os.getpid()} user={user} since={utc_iso()} cmd=GCS build {name}\n".encode())
        return fd

    def _release_lock(self) -> None:
        if self._lock_fd is not None:
            os.close(self._lock_fd)  # closing the descriptor drops the flock
            self._lock_fd = None

    # ---- start, run, cancel ---------------------------------------------------------------------------------------------

    async def start(self, name) -> dict:
        preset = self.preset_of(name)
        reasons = self.refusals()
        if reasons:
            self.log(f"build refused ({name}): {'; '.join(reasons)}")
            raise BuildRefused(reasons)
        try:
            describe = await asyncio.to_thread(git_describe, self.root)
        except (OSError, subprocess.SubprocessError) as e:
            raise BuildRefused([f"the build is not started: {e}"]) from None
        self._lock_fd = self._take_lock(name)  # the lock itself is the arbiter: the refusals above are a courtesy
        try:
            tree = self.tree_dir(preset)
            logs = self.run_dir / "builds"
            logs.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            log_path = logs / f"{stamp}-{name}.log"
            self.state = {"running": True, "preset": name, "tree": preset.tree, "describe": describe, "started_utc": utc_iso(),
                          "finished_utc": None, "ok": None, "cancelled": False, "exit_status": None, "error": None, "step": None,
                          "steps": None, "tail": [], "log": str(log_path), "images": []}
            self._cancel = False
            self.log(f"build started: {name} describe {describe} tree {tree} lock {self.lock_path()}")
            self.push({"event": "build_started", "preset": name, "tree": preset.tree, "describe": describe, "started_utc": self.state["started_utc"]})
            self._task = asyncio.create_task(self._run(preset, tree, log_path))
        except BaseException:
            self._release_lock()
            raise
        return {"started": True, "build": self.view()}

    @staticmethod
    def argvs(preset: BuildPreset, tree: Path) -> list[list[str]]:
        configure = ["cmake", "--preset", preset.configure, "-B", str(tree), *[f"-D{k}={v}" for k, v in preset.cache]]
        build = ["cmake", "--build", str(tree)]
        if preset.targets:
            build += ["--target", *preset.targets]
        return [configure, build]

    async def _run(self, preset: BuildPreset, tree: Path, log_path: Path) -> None:
        st = self.state
        status, error = None, None
        try:
            env = {**os.environ, "MARV_HEAVY_HELD": str(self.lock_path())}
            with open(log_path, "wb") as out:
                for argv in self.argvs(preset, tree):
                    if self._cancel:
                        break
                    out.write(f"+ {' '.join(argv)}\n".encode())
                    status = await self._step(argv, env, out)
                    if status != 0:
                        break
        except OSError as e:
            error = f"the build did not start: {e}"
        except asyncio.CancelledError:
            error = "the helper stopped the build"
            raise
        finally:
            await self._reap()
            self._release_lock()
            self._proc = None
            ok = status == 0 and error is None and not self._cancel
            if self._cancel:
                error = "cancelled"
            elif not ok and error is None:
                error = f"the build step exited with status {status}"
            images = []
            if ok:
                images = await asyncio.to_thread(self._record, preset, tree, st)
            st.update(running=False, ok=ok, cancelled=self._cancel, exit_status=status, error=error, finished_utc=utc_iso(), images=images)
            self.log(f"build done: {st['preset']} ok {ok} status {status} {error or ''}")
            self.push({"event": "build_done", **{k: st[k] for k in ("preset", "tree", "describe", "ok", "cancelled", "exit_status", "error", "finished_utc", "images")}})

    async def _step(self, argv: list[str], env: dict, out) -> int:
        proc = await asyncio.create_subprocess_exec(*argv, cwd=self.root, env=env, stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
                                                    stderr=asyncio.subprocess.STDOUT, start_new_session=True)
        self._proc = proc
        if self._cancel:  # a cancel arrived while the step was being spawned
            self._signal(proc.pid, {}, signal.SIGKILL)
        reader = asyncio.create_task(self._read(proc, out))
        try:
            await proc.wait()
            try:
                await asyncio.wait_for(asyncio.shield(reader), DRAIN_S)
            except asyncio.TimeoutError:
                reader.cancel()
                await asyncio.gather(reader, return_exceptions=True)
        finally:
            if not reader.done():
                reader.cancel()
                await asyncio.gather(reader, return_exceptions=True)
        return proc.returncode

    async def _read(self, proc, out) -> None:
        st, pending, last_push, unsent = self.state, b"", 0.0, None

        def line(raw: bytes) -> None:
            nonlocal last_push, unsent
            text = raw.decode("utf-8", "replace").rstrip()
            if not text:
                return
            st["tail"] = [*st["tail"][-(BUILD_TAIL_LINES - 1):], text]
            m = STEP_RE.match(text)
            if m:
                st["step"], st["steps"] = int(m.group(1)), int(m.group(2))
            now = time.monotonic()
            if now - last_push >= BUILD_PUSH_S:
                last_push, unsent = now, None
                self.push({"event": "build_progress", "preset": st["preset"], "line": text, "step": st["step"], "steps": st["steps"]})
            else:
                unsent = text

        try:
            while chunk := await proc.stdout.read(READ_SIZE):
                out.write(chunk)
                pending += chunk
                *lines, pending = re.split(rb"[\r\n]", pending)
                for raw in lines:
                    line(raw)
        finally:
            if pending:
                line(pending)
            if unsent is not None:
                self.push({"event": "build_progress", "preset": st["preset"], "line": unsent, "step": st["step"], "steps": st["steps"]})

    def _record(self, preset: BuildPreset, tree: Path, st: dict) -> list[dict]:
        """Writes <tree>/gs-build.json for the images the tree now holds; returns them as [{hash, name, kind}]."""
        images = [e for e in self.library.scan()["images"] if e["tree"] == preset.tree]
        record = {"schema": RECORD_SCHEMA, "preset": st["preset"], "describe": st["describe"], "started_utc": st["started_utc"],
                  "finished_utc": utc_iso(), "images": {e["hash"]: e["name"] for e in images}}
        (tree / BUILD_RECORD).write_text(json.dumps(record, indent=1, sort_keys=True) + "\n")
        return [{"hash": e["hash"], "name": e["name"], "kind": e["kind"]} for e in images]

    async def cancel(self) -> dict:
        if not self.busy():
            raise DeviceError("no build is running")
        self._cancel = True
        self.log(f"build cancel: {self.state['preset']}")
        await self._terminate()
        await asyncio.wait_for(asyncio.shield(self._task), self.stop_s * 3)
        return {"cancelled": True, "build": self.view()}

    async def _terminate(self) -> None:
        """SIGTERM, then SIGKILL after stop_s, to the step's process group and to every descendant of its leader (ninja's jobs are in
        groups of their own); returns when none of them lives."""
        proc = self._proc
        if proc is None:
            return
        known = {proc.pid: _start_of(proc.pid)}
        known.update(descendants(proc.pid))
        sent_kill = False
        deadline = time.monotonic() + self.stop_s
        self._signal(proc.pid, known, signal.SIGTERM)
        while True:
            if proc.returncode is None:
                known.update(descendants(proc.pid))
            living = {p: s for p, s in known.items() if s is not None and _alive(p, s)}
            if not living and proc.returncode is not None:
                return
            if time.monotonic() >= deadline and not sent_kill:
                sent_kill = True
                self._signal(proc.pid, living, signal.SIGKILL)
            elif sent_kill and not living:
                return
            await asyncio.sleep(POLL_S)

    @staticmethod
    def _signal(group: int, pids: dict, sig: int) -> None:
        try:
            os.killpg(group, sig)
        except ProcessLookupError:
            pass
        for pid, start in pids.items():
            if start is not None and _alive(pid, start):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass

    async def _kill_group(self, proc) -> None:
        known = {proc.pid: _start_of(proc.pid)}
        known.update(descendants(proc.pid))
        self._signal(proc.pid, known, signal.SIGKILL)
        await proc.wait()

    async def _reap(self) -> None:
        """A step has ended (or the task was cancelled): nothing of its process group may remain."""
        proc = self._proc
        if proc is None:
            return
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    async def stop(self) -> None:
        """The helper is stopping: a build that is still going is cancelled with its processes (H-2), the lock released."""
        if self.busy():
            self._cancel = True
            await self._terminate()
            try:
                await asyncio.wait_for(asyncio.shield(self._task), self.stop_s * 3)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                self._task.cancel()
                await asyncio.gather(self._task, return_exceptions=True)
        self._release_lock()


def _start_of(pid: int) -> str | None:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
    except OSError:
        return None
    return stat[stat.rindex(")") + 2:].split()[19]
