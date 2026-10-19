"""The helper's end of the sim channel (ground-segment unit GX-8, decision 0056): a UDP socket on 127.0.0.1 that the lockstep
plugin's records arrive on (sim/gz/plugin/src/sim_telemetry.hpp has the record), relayed on /ws/sim and logged.

Like the link relay it never parses what it carries: one datagram is one record, handed on as one binary WebSocket message and
written as one record of the sim log. The only things it counts are datagrams and bytes. The sim channel is one way (simulator to
browser): nothing the browser sends reaches the simulator, and the helper closes a /ws/sim client that sends anything (helper.py).

The sim log, `sim.log` in the run directory, is a ground log (tools/gs/groundlog.py, same header and record framing): every
datagram is one RX record with the helper's monotonic time, so it aligns with ground.log, whose EVENT records carry the run's
start and stop. It is created at the first datagram, so a session with no simulator has none. It is a sibling of ground.log and
not part of it: the ground log is the vehicle's byte stream (GS-8, replayed as a vehicle) and sim truth must never be in it (S-1).
"""

from __future__ import annotations

import asyncio
import socket
from pathlib import Path
from typing import Callable

import groundlog

LOOPBACK = "127.0.0.1"
SIM_LOG_NAME = "sim.log"


class SimChannel:
    """`on_datagram(bytes)` is called for every datagram, on the event loop, after it is logged."""

    def __init__(self, run_dir: Path, on_datagram: Callable[[bytes], None], port: int = 0):
        self.run_dir = Path(run_dir)
        self._on_datagram = on_datagram
        self._want_port = port
        self.port = 0
        self.datagrams = 0
        self.bytes = 0
        self._log: groundlog.GroundLog | None = None
        self._transport: asyncio.DatagramTransport | None = None

    async def start(self) -> None:
        channel = self

        class Protocol(asyncio.DatagramProtocol):
            def datagram_received(self, data, addr):
                channel._received(data)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setblocking(False)
            sock.bind((LOOPBACK, self._want_port))
            self._transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(Protocol, sock=sock)
        except OSError:
            sock.close()
            raise
        self.port = self._transport.get_extra_info("sockname")[1]

    def _received(self, data: bytes) -> None:
        if self._log is None:
            self._log = groundlog.GroundLog(self.run_dir / SIM_LOG_NAME)
        self._log.write(groundlog.RX, data)
        self.datagrams += 1
        self.bytes += len(data)
        self._on_datagram(data)

    def close(self) -> None:
        if self._transport is not None:
            self._transport.close()
            self._transport = None
        if self._log is not None:
            self._log.close()
            self._log = None

    def status(self) -> dict:
        return {"port": self.port, "datagrams": self.datagrams, "bytes": self.bytes,
                "log": str(self.run_dir / SIM_LOG_NAME) if self._log is not None else None}
