"""The SIL launcher: starts and stops one Gazebo SIL run (tools/sim/run_quad_fc.py --interactive, which owns the world
generation), and relaunches it when the run asks for a reboot.

The default command is the interactive run (real time, arm switch low, until stopped) with --config-dir <run_dir>/store, the
config storage the SIL's config store boots from (it survives the relaunches), and, when the helper has a UDP port (a scenario
value set by whoever starts the helper), --link-udp-port <port>. The port is also in the environment variable UDP_PORT_ENV,
which the plugin reads when the world names no port (the contract of a command given instead of the default).

A run that exits with os.EX_TEMPFAIL asked for a reboot (a config command was accepted; sim/gz/plugin/src/lockstep.cpp): a
watcher thread starts the same command again, on the same run directory, unless stop() was called (a lock-guarded flag).
`status()["link"]` is one of
  none         the helper has no UDP port: the run has no link
  stopped      no run
  starting     the run is up and nothing is bound to 127.0.0.1:<port> yet
  ready        a socket is bound to 127.0.0.1:<port> (the SIL's link; Linux /proc/net/udp)
  relaunching  a relaunched run is up and has not bound the port yet
"""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
UDP_PORT_ENV = "MARV_SIL_LINK_UDP_PORT"
LOOPBACK = "127.0.0.1"
PROC_NET_UDP = Path("/proc/net/udp")
STOP_GRACE_S = 10.0  # labelled harness value: how long SIGTERM gets before SIGKILL


def default_command(out_dir: Path, config_dir: Path, udp_port: int | None) -> list[str]:
    command = [sys.executable, str(ROOT / "tools" / "sim" / "run_quad_fc.py"), "--out-dir", str(out_dir), "--interactive",
               "--config-dir", str(config_dir)]
    return command + (["--link-udp-port", str(udp_port)] if udp_port is not None else [])


def udp_bound(port: int) -> bool:
    """True iff a socket is bound to 127.0.0.1:<port> (/proc/net/udp prints the address as the hex of its 32 bits read in
    host byte order, and the port in hex); False where the table cannot be read."""
    want = f"{int.from_bytes(socket.inet_aton(LOOPBACK), sys.byteorder):08X}:{port:04X}"
    try:
        rows = PROC_NET_UDP.read_text(encoding="ascii").splitlines()[1:]
    except OSError:
        return False
    return any(len(fields) > 1 and fields[1] == want for fields in (row.split() for row in rows))


class SilLauncher:
    def __init__(self, run_dir: Path, udp_port: int | None, command: list[str] | None = None):
        self.run_dir = Path(run_dir)
        self.udp_port = udp_port
        self.config_dir = self.run_dir / "store"
        self._command = command
        self._proc: subprocess.Popen | None = None
        self._log = None
        self._lock = threading.Lock()
        self._stopping = False
        self._relaunches = 0
        self._watcher: threading.Thread | None = None

    def start(self) -> None:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                raise RuntimeError("a SIL run is already running")
            out_dir = self.run_dir / "sil"
            out_dir.mkdir(parents=True, exist_ok=True)
            if self._log is None:
                self._log = open(out_dir / "launcher.log", "ab")
            self._stopping = False
            self._relaunches = 0
            self._spawn()
            proc = self._proc
            self._watcher = threading.Thread(target=self._watch, args=(proc,), name="sil-watcher", daemon=True)
            self._watcher.start()

    def _spawn(self) -> None:
        """Starts the command in its own process group; the caller holds the lock."""
        env = dict(os.environ)
        if self.udp_port is not None:
            env[UDP_PORT_ENV] = str(self.udp_port)
        command = self._command or default_command(self.run_dir / "sil", self.config_dir, self.udp_port)
        self._proc = subprocess.Popen(command, cwd=ROOT, env=env, stdout=self._log, stderr=subprocess.STDOUT,
                                      start_new_session=True)

    def _watch(self, proc: subprocess.Popen) -> None:
        """Relaunches the run while it exits with EX_TEMPFAIL and stop() has not been called."""
        while True:
            proc.wait()
            with self._lock:
                if self._stopping or self._proc is not proc or proc.returncode != os.EX_TEMPFAIL:
                    return
                self._relaunches += 1
                self._log.write(f"sil.py: exit EX_TEMPFAIL (reboot requested): relaunch {self._relaunches}\n".encode())
                self._log.flush()
                self._spawn()
                proc = self._proc

    def running(self) -> bool:
        proc = self._proc
        return proc is not None and proc.poll() is None

    def stop(self) -> int | None:
        """Ends the relaunching, SIGTERMs the run's whole process group, SIGKILLs what is left after STOP_GRACE_S; returns the
        leader's exit status. The group is signalled whether or not the leader is still alive: children of an exited leader
        are stopped too."""
        with self._lock:
            self._stopping = True
            proc = self._proc
        if proc is None:
            return None
        self._signal(proc, signal.SIGTERM)
        deadline = time.monotonic() + STOP_GRACE_S
        while time.monotonic() < deadline and (proc.poll() is None or self._group_alive(proc)):
            time.sleep(0.05)
        self._signal(proc, signal.SIGKILL)
        proc.wait()
        if self._watcher is not None:
            self._watcher.join()
            self._watcher = None
        with self._lock:
            if self._log is not None:
                self._log.close()
                self._log = None
        return proc.returncode

    @staticmethod
    def _group_alive(proc: subprocess.Popen) -> bool:
        try:
            os.killpg(proc.pid, 0)
        except ProcessLookupError:
            return False
        return True

    @staticmethod
    def _signal(proc: subprocess.Popen, sig: int) -> None:
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            pass

    def link_state(self) -> str:
        if self.udp_port is None:
            return "none"
        if not self.running():
            return "stopped"
        if udp_bound(self.udp_port):
            return "ready"
        return "relaunching" if self._relaunches > 0 else "starting"

    def status(self) -> dict:
        proc = self._proc
        return {
            "running": self.running(),
            "pid": proc.pid if proc else None,
            "exit_status": proc.returncode if proc and proc.poll() is not None else None,
            "udp_port": self.udp_port,
            "config_dir": str(self.config_dir),
            "relaunches": self._relaunches,
            "link": self.link_state(),
        }
