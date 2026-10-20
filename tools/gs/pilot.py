"""The SIL pilot bridge of the helper (ground-segment unit GX-9, decision 0056; invariants PB-1 and PB-2).

The pilot flies the SIL from the RadioMaster in USB-joystick mode. The helper reads the radio through the Linux joystick API
(/dev/input/js*, struct js_event on the device file; no library), turns the axes into the 16 raw 11-bit channels of a CRSF RC
frame and sends one frame per period as a UDP datagram to the lockstep plugin's <pilot_udp_port> (sim/gz/plugin/src/pilot_input.hpp
has the datagram and the staging rule). The plugin hands each frame to marv_sil_stage_rc_frame, which decodes it with the board's
own rc_input::crsf_to_controls and the parameter table's rc_channel_*: this module decides no stick, no sign and no channel role,
it forwards channels (invariant R-2).

Manual control never goes on the MAVLink link (PB-1): the frames go to the plugin's own port, a different socket from the link's.

Closing. close() cancels the sender and closes the device and the socket before it returns, so no frame is sent after it (PB-2). A
joystick that goes away (a read error, end of file) is a close with state "lost". With no frames the plugin stops staging, the SIL's
last frame ages, and the link-loss failsafe acts as it does on the board when the receiver stops.

Axis to channel. The EdgeTX channel to axis layout of the USB-joystick mode is UNKNOWN until the bench check (BENCH.md in
tests/regression/quad/L10/results/gcs/gx9/): by default channel n is axis n - 1; `axis_map` (16 axis numbers, null = centred) replaces
it. An axis value v in [-32767, 32767] (Linux joystick API: Documentation/input/joydev/joystick-api.rst, "axis values from -32767 to
32767") is the radio's channel from -100 % to +100 %: ticks = centre + round(v / 32767 * half_span_ticks), where centre is the CRSF
centre tick, half_span_ticks = (kRcUsHalfSpan us) * kCrsfUsPerTickDen / kCrsfUsPerTickNum, both read from
fw/prim/include/marv/prim/constants.hpp (TBS CRSF spec; the header carries the citation), clamped to the 11-bit field. That mapping
is a harness choice (PLACEHOLDER until the bench shows the radio's scaling); a channel with no axis, or an axis not yet seen, is centred.
"""

from __future__ import annotations

import asyncio
import os
import re
import socket
import struct
from pathlib import Path
from typing import Callable

LOOPBACK = "127.0.0.1"
JS_DIR = Path("/dev/input")
JS_NAME = re.compile(r"^js[0-9]+$")
JS_EVENT = struct.Struct("<IhBB")  # struct js_event: time (ms), value, type, number (linux/joystick.h)
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80  # linux/joystick.h
JS_AXIS_FULL = 32767  # Linux joystick API: axis values -32767 .. 32767
JS_READ_EVENTS = 64  # implementation constant: events read per wake-up
FRAME_PERIOD_S = 0.02  # PLACEHOLDER (harness): the helper sends a frame every 20 ms (50 Hz), far inside failsafe_loss_us (100 ms); the radio's own rate is ~125 Hz (docs/m2-bench-card.md)
STATE_HZ_MAX = 30.0  # PLACEHOLDER (harness): the cap on pilot_state events (the architect design: at most 30 Hz); a display rate
MAGIC = b"MPLT"
VERSION = 1
CHANNELS = 16  # CRSF: 16 channels (prim kCrsfRcChannels)
DATAGRAM = struct.Struct("<4sHH16H")
DATAGRAM_BYTES = DATAGRAM.size
PRIM_HEADER = Path(__file__).resolve().parents[2] / "fw" / "prim" / "include" / "marv" / "prim" / "constants.hpp"
_CONST = re.compile(r"^inline constexpr [\w:]+ (k\w+) = ([-+0-9.eE]+)F?;", re.MULTILINE)


class PilotError(Exception):
    pass


def crsf_constants(header: Path = PRIM_HEADER) -> dict:
    """centre tick, the half span of a stick in ticks and the largest value of the channel field, from the firmware's constants."""
    raw = dict(_CONST.findall(Path(header).read_text(encoding="utf-8")))
    try:
        centre = int(raw["kCrsfTickCentre"])
        bits = int(raw["kCrsfRcChannelBits"])
        channels = int(raw["kCrsfRcChannels"])
        half_span_us, per_num, per_den = float(raw["kRcUsHalfSpan"]), float(raw["kCrsfUsPerTickNum"]), float(raw["kCrsfUsPerTickDen"])
    except KeyError as e:
        raise PilotError(f"{header} lacks the constant {e}") from None
    if channels != CHANNELS:
        raise PilotError(f"the header says {channels} CRSF channels, the datagram format has {CHANNELS}")
    return {"centre": centre, "half_span_ticks": half_span_us * per_den / per_num, "raw_max": (1 << bits) - 1}


CONSTANTS = crsf_constants()


def axis_to_ticks(value: int) -> int:
    """The 11-bit channel value of a joystick axis value (module docstring)."""
    v = max(-JS_AXIS_FULL, min(JS_AXIS_FULL, int(value)))
    ticks = CONSTANTS["centre"] + round(v / JS_AXIS_FULL * CONSTANTS["half_span_ticks"])
    return max(0, min(CONSTANTS["raw_max"], ticks))


def encode_frame(channels) -> bytes:
    """The datagram of 16 channel values (each 0 .. 2^11 - 1)."""
    if len(channels) != CHANNELS or any(not 0 <= c <= CONSTANTS["raw_max"] for c in channels):
        raise PilotError(f"a frame is {CHANNELS} channels in 0 .. {CONSTANTS['raw_max']}")
    return DATAGRAM.pack(MAGIC, VERSION, DATAGRAM_BYTES, *channels)


def decode_frame(data: bytes) -> list[int]:
    """The reference decoder of a datagram (tests); PilotError for anything else."""
    if len(data) != DATAGRAM_BYTES:
        raise PilotError(f"{len(data)} bytes, not {DATAGRAM_BYTES}")
    magic, version, length, *channels = DATAGRAM.unpack(data)
    if magic != MAGIC or version != VERSION or length != DATAGRAM_BYTES or any(c > CONSTANTS["raw_max"] for c in channels):
        raise PilotError("not a version-1 pilot datagram")
    return channels


def parse_events(data: bytes) -> list[tuple[int, int, int]]:
    """(type without the INIT bit, number, value) of each whole js_event in `data`."""
    out = []
    for off in range(0, len(data) - JS_EVENT.size + 1, JS_EVENT.size):
        _, value, kind, number = JS_EVENT.unpack_from(data, off)
        out.append((kind & ~JS_EVENT_INIT, number, value))
    return out


def free_udp_port() -> int:
    """A UDP port that is free on the loopback interface now (the plugin binds it when the run starts)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind((LOOPBACK, 0))
        return s.getsockname()[1]


def check_axis_map(axis_map) -> list[int | None]:
    if axis_map is None:
        return list(range(CHANNELS))
    if (not isinstance(axis_map, list) or len(axis_map) != CHANNELS
            or any(a is not None and (isinstance(a, bool) or not isinstance(a, int) or not 0 <= a < 256) for a in axis_map)):
        raise PilotError(f"axis_map is a list of {CHANNELS} axis numbers (0 .. 255) or null")
    return list(axis_map)


class Pilot:
    """`emit(event dict)` is called on the event loop for each pilot_state event. `device_dir` is where the joysticks are
    (JS_DIR; tests use a directory of their own). One joystick at a time."""

    def __init__(self, emit: Callable[[dict], None], device_dir: Path | None = None, *, frame_period_s: float = FRAME_PERIOD_S,
                 state_hz: float = STATE_HZ_MAX):
        self._emit = emit
        self.device_dir = Path(device_dir) if device_dir is not None else JS_DIR
        self.frame_period_s = frame_period_s
        self._min_gap_s = 1.0 / state_hz
        self.state = "closed"  # closed | open | lost
        self.reason: str | None = None
        self.device: str | None = None
        self.port: int | None = None
        self.frames_sent = 0
        self.send_errors = 0
        self._axes: dict[int, int] = {}
        self._axis_map: list[int | None] = list(range(CHANNELS))
        self._fd: int | None = None
        self._sock: socket.socket | None = None
        self._task: asyncio.Task | None = None
        self._last_emit = float("-inf")

    # ---- devices ----------------------------------------------------------------------------------------------------------

    def devices(self) -> list[str]:
        """The joystick device files, sorted."""
        try:
            names = sorted(os.listdir(self.device_dir))
        except OSError:
            return []
        return [os.path.join(self.device_dir, n) for n in names if JS_NAME.match(n)]

    def resolve(self, path) -> str:
        """A device path from the browser: a normalised absolute path of the form <device dir>/js<N> that is a file of that
        directory itself (a symlink to elsewhere, ../ or another name is refused)."""
        if not isinstance(path, str) or not path or "\0" in path:
            raise PilotError("device must be a non-empty string")
        if (os.path.normpath(path) != path or not os.path.isabs(path) or os.path.dirname(path) != str(self.device_dir)
                or not JS_NAME.match(os.path.basename(path))):
            raise PilotError(f"device {path!r} is not a joystick device ({self.device_dir}/js<N>)")
        if Path(os.path.realpath(path)).parent != Path(os.path.realpath(self.device_dir)):
            raise PilotError(f"device {path!r} does not resolve to a file of {self.device_dir}")
        return path

    # ---- open and close ---------------------------------------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self.state == "open"

    async def open(self, path, port: int, axis_map=None) -> None:
        """Opens the joystick and starts sending frames to 127.0.0.1:<port>. PilotError for a refusal."""
        if self.is_open:
            raise PilotError(f"the joystick {self.device} is open: close it first")
        path = self.resolve(path)
        axes = check_axis_map(axis_map)
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
        except PermissionError:
            raise PilotError(f"cannot open {path}: permission denied (the user needs read access to the joystick device; "
                             "tests/regression/quad/L10/results/gcs/gx9/BENCH.md)") from None
        except OSError as e:
            raise PilotError(f"cannot open {path}: {e.strerror}") from None
        self._fd, self._axis_map, self._axes = fd, axes, {}
        self.device, self.port, self.reason = path, int(port), None
        self.frames_sent = self.send_errors = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        loop = asyncio.get_running_loop()
        loop.add_reader(fd, self._readable)
        self.state = "open"
        self._task = asyncio.create_task(self._send_loop())
        self._state_event(force=True)

    def close(self, reason: str = "closed by request") -> None:
        """Stops the frames and releases the device and the socket, then tells the GCS. Idempotent (a closed pilot says nothing)."""
        self._stop("closed", reason)

    def _lose(self, reason: str) -> None:
        self._stop("lost", reason)

    def _stop(self, state: str, reason: str) -> None:
        if self.state != "open":
            return
        if self._task is not None:
            self._task.cancel()
            self._task = None
        if self._fd is not None:
            try:
                asyncio.get_running_loop().remove_reader(self._fd)
            except (RuntimeError, ValueError):
                pass
            os.close(self._fd)
            self._fd = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        self.state, self.reason = state, reason
        self._state_event(force=True)

    # ---- reading ----------------------------------------------------------------------------------------------------------

    def _readable(self) -> None:
        if self._fd is None:
            return
        try:
            data = os.read(self._fd, JS_EVENT.size * JS_READ_EVENTS)
        except BlockingIOError:
            return
        except OSError as e:
            self._lose(f"read error: {e.strerror}")
            return
        if not data:
            self._lose("end of file")
            return
        for kind, number, value in parse_events(data):
            if kind == JS_EVENT_AXIS:
                self._axes[number] = value

    def channels(self) -> list[int]:
        """The 16 raw channel values now (module docstring)."""
        centre = CONSTANTS["centre"]
        return [centre if a is None or a not in self._axes else axis_to_ticks(self._axes[a]) for a in self._axis_map]

    # ---- sending ----------------------------------------------------------------------------------------------------------

    async def _send_loop(self) -> None:
        loop = asyncio.get_running_loop()
        due = loop.time()
        while True:
            sock = self._sock
            if sock is None:
                return
            try:
                sock.sendto(encode_frame(self.channels()), (LOOPBACK, self.port))
                self.frames_sent += 1
            except OSError:
                self.send_errors += 1
            self._state_event(force=False)
            # A stalled loop resumes without a burst of catch-up frames: the next one is due a period from now.
            due = max(due + self.frame_period_s, loop.time())
            await asyncio.sleep(max(0.0, due - loop.time()))

    # ---- state ------------------------------------------------------------------------------------------------------------

    def status(self) -> dict:
        return {"state": self.state, "reason": self.reason, "device": self.device, "port": self.port,
                "frames_sent": self.frames_sent, "send_errors": self.send_errors, "frame_period_s": self.frame_period_s,
                "devices": self.devices(), "channels": self.channels() if self.is_open else None}

    def _state_event(self, *, force: bool) -> None:
        """A pilot_state event; the periodic ones are held to STATE_HZ_MAX, a change of state is always told."""
        now = asyncio.get_running_loop().time()
        if not force and now - self._last_emit < self._min_gap_s:
            return
        self._last_emit = now
        self._emit({"event": "pilot_state", "state": self.state, "reason": self.reason, "device": self.device, "port": self.port,
                    "frames_sent": self.frames_sent, "send_errors": self.send_errors,
                    "channels": self.channels() if self.is_open else None})
