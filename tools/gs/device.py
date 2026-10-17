"""The helper's device connections: a serial port or the SIL's UDP socket. Bytes in, bytes out; nothing is parsed here."""

from __future__ import annotations

import asyncio
import errno
import fcntl
import os
import termios
import tty
from typing import Callable

LOOPBACK = "127.0.0.1"
UDP_PAYLOAD_MAX = 65507  # largest IPv4 UDP payload: 65535 - 20 (IPv4 header) - 8 (UDP header), RFC 768 and RFC 791
READ_SIZE = 65536  # serial read size per wake-up: an implementation constant, the chunk the log and the relay carry
DEFAULT_BAUD = 115200  # a convenience default; a USB CDC port ignores the baud rate


class DeviceError(Exception):
    pass


class Device:
    """on_rx(bytes) is called for every chunk read; on_close(reason) once when the connection ends by itself."""

    description = ""

    def write(self, data: bytes) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


class SerialDevice(Device):
    def __init__(self, path: str, baud: int, on_rx: Callable[[bytes], None], on_close: Callable[[str], None]):
        self.description = f"serial {path} {baud}"
        self._on_rx, self._on_close = on_rx, on_close
        self._out = bytearray()
        self._closed = False
        self._loop = asyncio.get_running_loop()
        try:
            self._fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            raise DeviceError(f"cannot open {path}: {e.strerror}") from e
        try:
            tty.setraw(self._fd)
            attrs = termios.tcgetattr(self._fd)
            speed = getattr(termios, f"B{baud}", None)
            if speed is None:
                raise DeviceError(f"unsupported baud rate {baud}")
            attrs[4] = attrs[5] = speed
            attrs[2] |= termios.CLOCAL | termios.CREAD
            termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
            fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (termios.error, OSError) as e:
            os.close(self._fd)
            raise DeviceError(f"cannot configure {path}: {e}") from e
        except DeviceError:
            os.close(self._fd)
            raise
        self._loop.add_reader(self._fd, self._readable)

    def _readable(self) -> None:
        try:
            data = os.read(self._fd, READ_SIZE)
        except BlockingIOError:
            return
        except OSError as e:
            self._fail(f"read error: {e.strerror}")
            return
        if not data:
            self._fail("end of file")
            return
        self._on_rx(data)

    def _fail(self, reason: str) -> None:
        if not self._closed:
            self.close()
            self._on_close(reason)

    def write(self, data: bytes) -> None:
        if self._closed:
            return
        if self._out:
            self._out += data
            return
        try:
            n = os.write(self._fd, data)
        except BlockingIOError:
            n = 0
        except OSError as e:
            if e.errno == errno.EAGAIN:
                n = 0
            else:
                self._fail(f"write error: {e.strerror}")
                return
        if n < len(data):
            self._out += data[n:]
            self._loop.add_writer(self._fd, self._writable)

    def _writable(self) -> None:
        try:
            n = os.write(self._fd, bytes(self._out))
        except BlockingIOError:
            return
        except OSError as e:
            self._fail(f"write error: {e.strerror}")
            return
        del self._out[:n]
        if not self._out:
            self._loop.remove_writer(self._fd)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._loop.remove_reader(self._fd)
        self._loop.remove_writer(self._fd)
        os.close(self._fd)


class UdpDevice(Device):
    """A connected UDP socket to 127.0.0.1:<port>: datagrams from any other source are not delivered."""

    def __init__(self, transport: asyncio.DatagramTransport, port: int):
        self.description = f"udp {LOOPBACK}:{port}"
        self.port = port
        self._transport = transport
        self._closed = False

    @classmethod
    async def open(cls, port: int, on_rx: Callable[[bytes], None], on_close: Callable[[str], None]) -> "UdpDevice":
        if not 0 < port < 65536:
            raise DeviceError(f"bad UDP port {port}")
        holder: dict = {}

        class Protocol(asyncio.DatagramProtocol):
            def datagram_received(self, data, addr):
                on_rx(data)

            def error_received(self, exc):  # e.g. connection refused while nothing listens yet: the link is simply silent
                holder["last_error"] = str(exc)

            def connection_lost(self, exc):
                if exc is not None and not holder["dev"]._closed:
                    on_close(f"socket error: {exc}")

        loop = asyncio.get_running_loop()
        try:
            transport, _ = await loop.create_datagram_endpoint(Protocol, remote_addr=(LOOPBACK, port))
        except OSError as e:
            raise DeviceError(f"cannot open UDP {LOOPBACK}:{port}: {e.strerror}") from e
        dev = cls(transport, port)
        holder["dev"] = dev
        return dev

    def write(self, data: bytes) -> None:
        for i in range(0, len(data), UDP_PAYLOAD_MAX):
            self._transport.sendto(data[i:i + UDP_PAYLOAD_MAX])

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._transport.close()
