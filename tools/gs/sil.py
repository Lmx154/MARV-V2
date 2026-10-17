"""The SIL launcher: starts and stops one Gazebo SIL run (tools/sim/run_quad_fc.py, which owns the world generation).

The SIL's link service and UDP socket arrive with ground segment unit 6. Until then a launched run has no link:
`link_status` says so, and the UDP port the run is to serve is passed in the environment variable UDP_PORT_ENV so that
unit 6 reads it from the launch contract (the scenario value of the port is set by whoever starts the helper).
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
UDP_PORT_ENV = "MARV_SIL_LINK_UDP_PORT"
LINK_NOT_AVAILABLE = "link: not yet available (the SIL link service and UDP socket arrive with unit 6)"
STOP_GRACE_S = 10.0  # labelled harness value: how long SIGTERM gets before SIGKILL


def default_command(out_dir: Path) -> list[str]:
    return [sys.executable, str(ROOT / "tools" / "sim" / "run_quad_fc.py"), "--out-dir", str(out_dir), "--runs", "flight"]


class SilLauncher:
    def __init__(self, run_dir: Path, udp_port: int | None, command: list[str] | None = None):
        self.run_dir = Path(run_dir)
        self.udp_port = udp_port
        self._command = command
        self._proc: subprocess.Popen | None = None
        self._log = None

    def start(self) -> None:
        if self.running():
            raise RuntimeError("a SIL run is already running")
        out_dir = self.run_dir / "sil"
        out_dir.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ)
        if self.udp_port is not None:
            env[UDP_PORT_ENV] = str(self.udp_port)
        self._log = open(out_dir / "launcher.log", "ab")
        self._proc = subprocess.Popen(self._command or default_command(out_dir), cwd=ROOT, env=env, stdout=self._log,
                                      stderr=subprocess.STDOUT, start_new_session=True)

    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def stop(self) -> int | None:
        """SIGTERM the run's process group, SIGKILL after STOP_GRACE_S; returns the exit status."""
        proc = self._proc
        if proc is None:
            return None
        if proc.poll() is None:
            self._signal(proc, signal.SIGTERM)
            deadline = time.monotonic() + STOP_GRACE_S
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if proc.poll() is None:
                self._signal(proc, signal.SIGKILL)
                proc.wait()
        if self._log is not None:
            self._log.close()
            self._log = None
        return proc.returncode

    @staticmethod
    def _signal(proc: subprocess.Popen, sig: int) -> None:
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            pass

    def status(self) -> dict:
        proc = self._proc
        return {
            "running": self.running(),
            "pid": proc.pid if proc else None,
            "exit_status": proc.returncode if proc and proc.poll() is not None else None,
            "udp_port": self.udp_port,
            "link": LINK_NOT_AVAILABLE,
        }
