"""The console relay of the helper (ground-segment unit GX-16, decision 0038): the board's console text to the GCS and the operator's
typed lines back, nothing else.

The board's console is the by-id `-if00` CDC of its composite USB device (fw/hal/target/src/usb.cpp); the link is `-if02` and stays the
autoconnect device (tools/gs/helper.py). The console is a device slot of its own beside the link's: the operator opens and closes it
(console_open / console_close on /ws), it never autoconnects, and it is a SerialDevice (tools/gs/device.py), so it takes the same flock
as the link and as a HIL run's plugin: one opener per port (H-1).

Text, not bytes. The device's bytes go down /ws/console as text frames (UTF-8, an incomplete sequence at the end of a chunk is kept for the
next, invalid bytes become U+FFFD), and a text frame from the client goes to the device as its UTF-8 bytes, unchanged: this module adds,
removes and interprets nothing, so whatever the operator types is what the board receives (the motor test of decision 0038 is a line the
board's console parses; the helper and the GCS know no motor command). A binary frame from the client closes it (1003). /ws/console has the
same Host, Origin and token gate as /ws and accepts one client; with no console open it carries nothing, and text sent to it is dropped.

Path. The device named by the browser must be a board's `usb-Raspberry_Pi_Pico_*-if00` entry (the link's vendor part; with --board-serial only that
board's) of the by-id directory itself (an absolute normalised path whose directory
is that directory: no ../, no subdirectory), and what it resolves to must be a character device under /dev. A symlink is how by-id entries
are made, so a symlink is followed, but one that leads anywhere else is refused.

Refused while the board belongs to something else: a HIL run is going, the HIL hold is taken, an SWD or picotool flash is running
(Console.refusals). An open console is closed when one of them begins (Console.watch, called by Helper.poll), and a HIL launch or an SWD
flash is refused while a console is open (tools/gs/hil.py).

Log. `console.log` in the helper's run directory, beside ground.log, in the ground log's format (tools/gs/groundlog.py: RX the board's
bytes, TX the operator's, EVENT open and close), created when a console opens and closed when it closes (a second open in the same run
writes console-1.log, and so on). Nothing is written while no console is open, and none of it goes into ground.log.
"""

from __future__ import annotations

import asyncio
import codecs
import fnmatch
import json
import os
import stat
from pathlib import Path

import groundlog
from device import DEFAULT_BAUD, DeviceError, SerialDevice

WS_PATH = "/ws/console"
WS_UNSUPPORTED = 1003  # RFC 6455 section 7.4.1 "unsupported data": a binary frame on the text channel
BOARD_PREFIX = "usb-Raspberry_Pi_Pico_"  # the Pico SDK's product string in the by-id name (helper.py BOARD_LINK_GLOB has the same vendor part)
CONSOLE_GLOB = BOARD_PREFIX + "*-if00"  # the by-id name of the board's console interface (interface 0 of the composite device)
CLIENT_QUEUE_MAX_BYTES = 1 << 20  # PLACEHOLDER (harness): device bytes queued for a slow browser before chunks are dropped (counted, logged); the board prints ~0.4 kB a second


class ConsoleError(DeviceError):
    """A console command the helper refuses: `extra` is merged into the reply (the list of reasons)."""

    def __init__(self, message: str, extra: dict | None = None):
        super().__init__(message)
        self.extra = extra or {}


class Console:
    """The console slot of a Helper (duck-typed: byid_dir, run_dir, runs, hil, _flash_task, log, _push)."""

    def __init__(self, helper, ws_busy: int):
        self.h = helper
        self._ws_busy = ws_busy
        self.device: SerialDevice | None = None
        self.path: str | None = None
        self.state = "closed"  # closed | open | lost
        self.reason: str | None = None
        self.rx_bytes = self.tx_bytes = self.dropped_bytes = 0
        self.client = None
        self._queue: asyncio.Queue | None = None
        self._queued_bytes = 0
        self._decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        self._log: groundlog.GroundLog | None = None
        self._log_path: Path | None = None
        self._logs = 0

    @property
    def is_open(self) -> bool:
        return self.device is not None

    # ---- the device -------------------------------------------------------------------------------------------------------

    def devices(self) -> list[str]:
        """The console interfaces under the by-id directory, sorted."""
        try:
            names = sorted(os.listdir(self.h.byid_dir))
        except OSError:
            return []
        return [os.path.join(self.h.byid_dir, n) for n in names if self._board_name(n)]

    def _board_name(self, name: str) -> bool:
        """Whether a by-id name is a board's console interface: CONSOLE_GLOB, and with --board-serial only that board's."""
        if not fnmatch.fnmatchcase(name, CONSOLE_GLOB):
            return False
        serial = self.h.board_serial
        return serial is None or name == f"{BOARD_PREFIX}{serial}-if00"

    def resolve(self, path) -> str:
        """A device path from the browser (module docstring); ConsoleError naming why not."""
        if not isinstance(path, str) or not path or "\0" in path:
            raise ConsoleError("device must be a non-empty string")
        byid = str(self.h.byid_dir)
        if (os.path.normpath(path) != path or not os.path.isabs(path) or os.path.dirname(path) != byid
                or not self._board_name(os.path.basename(path))):
            raise ConsoleError(f"device {path!r} is not the board's console interface: {byid}/{CONSOLE_GLOB}"
                               + (f" of the board {self.h.board_serial}" if self.h.board_serial else ""))
        real = os.path.realpath(path)
        try:
            is_tty_node = stat.S_ISCHR(os.stat(real).st_mode)
        except OSError as e:
            raise ConsoleError(f"cannot open {path}: {e.strerror}") from None
        if not (real.startswith("/dev/") and is_tty_node):
            raise ConsoleError(f"device {path!r} does not lead to a character device under /dev (it leads to {real})")
        return path

    def refusals(self) -> list[str]:
        """Every reason a console cannot be open now because the board belongs to a HIL run or a flash; empty when none."""
        h = self.h
        reasons = []
        active = h.runs.active()
        if active is not None and active["kind"] == "hil":
            reasons.append(f"a HIL run ({active['id']}) owns the board port: stop the run first (H-1)")
        if h.hil.hold:
            reasons.append("the board is held for a HIL run (hil_hold): release the hold first")
        if (h._flash_task is not None and not h._flash_task.done()) or h.hil.swd_busy():
            reasons.append("a flash is running: the board reboots, wait for it to finish")
        return reasons

    def open(self, msg: dict) -> dict:
        unknown = sorted(set(msg) - {"cmd", "id", "device"})
        if unknown:
            raise ConsoleError(f"not an option of console_open: {', '.join(unknown)}")
        if self.device is not None:
            raise ConsoleError(f"a console is open ({self.path}): close it first")
        path = self.resolve(msg.get("device"))
        reasons = self.refusals()
        if reasons:
            self.h.log.event(f"console refused: {'; '.join(reasons)}")
            raise ConsoleError(reasons[0], {"refusals": reasons})
        device = SerialDevice(path, DEFAULT_BAUD, self._rx, self._lost)  # DeviceError: the port is absent or another opener holds its flock
        try:
            self._open_log(path)
        except OSError as e:
            device.close()
            raise ConsoleError(f"cannot write the console log: {e.strerror}") from None
        self.device, self.path = device, path
        self._decoder.reset()
        self.rx_bytes = self.tx_bytes = self.dropped_bytes = 0
        self.state, self.reason = "open", None
        self._event()
        return self.status()

    def close(self, reason: str = "closed by request") -> dict:
        """Closes the device and the log. Idempotent: no console open, nothing happens."""
        self._release("closed", reason)
        return self.status()

    def watch(self) -> None:
        """Called by Helper.poll: a console is closed when the board begins to belong to a HIL run or a flash."""
        if self.device is not None:
            reasons = self.refusals()
            if reasons:
                self._release("closed", reasons[0])

    def _lost(self, reason: str) -> None:
        self._release("lost", reason)

    def _release(self, state: str, reason: str) -> None:
        device, self.device = self.device, None
        if device is None:
            return
        device.close()
        if self._log is not None:
            self._log.event(f"console {state}: {reason}")
            self._log.close()
            self._log = None
        self.state, self.reason = state, reason
        self._event()

    def _open_log(self, path: str) -> None:
        name = "console.log" if self._logs == 0 else f"console-{self._logs}.log"
        self._log_path = Path(self.h.run_dir) / name
        self._log = groundlog.GroundLog(self._log_path)
        self._logs += 1
        self._log.event(f"console opened: {path}")

    def _event(self) -> None:
        event = {"event": "console_state", "state": self.state, "reason": self.reason, "device": self.path if self.device else None}
        self.h.log.event("console " + json.dumps({k: v for k, v in event.items() if k != "event"}))
        self.h._push(json.dumps(event))

    # ---- the relay --------------------------------------------------------------------------------------------------------

    def _rx(self, data: bytes) -> None:
        """Bytes from the board: logged as they came, decoded and queued for the client."""
        self.rx_bytes += len(data)
        if self._log is not None:
            self._log.write(groundlog.RX, data)
        text = self._decoder.decode(data)
        queue = self._queue
        if queue is None or not text:
            return
        if self._queued_bytes + len(data) > CLIENT_QUEUE_MAX_BYTES:
            self.dropped_bytes += len(data)
            if self._log is not None:
                self._log.event(f"relay to browser dropped {len(data)} bytes (queue full)")
            return
        self._queued_bytes += len(data)
        queue.put_nowait((len(data), text))

    def _send(self, text: str) -> None:
        """Text from the operator: the device gets its UTF-8 bytes, unchanged. With no console open it is dropped."""
        if self.device is None:
            return
        data = text.encode("utf-8")
        self.tx_bytes += len(data)
        if self._log is not None:
            self._log.write(groundlog.TX, data)
        self.device.write(data)

    async def ws_handler(self, ws) -> None:
        """/ws/console: text frames both ways; one client."""
        if self.client is not None:
            await ws.close(self._ws_busy, "another console client is attached")
            return
        self.client = ws
        self._queue = queue = asyncio.Queue()
        self._queued_bytes = 0
        sender = asyncio.create_task(self._sender(ws, queue))
        try:
            async for message in ws:
                if isinstance(message, bytes):
                    await ws.close(WS_UNSUPPORTED, "the console carries text")
                    break
                self._send(message)
        finally:
            self.client, self._queue = None, None
            sender.cancel()
            await asyncio.gather(sender, return_exceptions=True)

    async def _sender(self, ws, queue: asyncio.Queue) -> None:
        while True:
            size, text = await queue.get()
            self._queued_bytes -= size
            await ws.send(text)

    # ---- status -----------------------------------------------------------------------------------------------------------

    def status(self) -> dict:
        return {"state": self.state, "reason": self.reason, "device": self.path if self.device else None, "client": self.client is not None,
                "rx_bytes": self.rx_bytes, "tx_bytes": self.tx_bytes, "dropped_bytes": self.dropped_bytes,
                "log": str(self._log_path) if self.device is not None and self._log_path is not None else None,
                "devices": self.devices(), "blocked": self.refusals()}
