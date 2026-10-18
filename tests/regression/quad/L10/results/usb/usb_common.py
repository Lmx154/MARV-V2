"""Bench only (ground-segment unit 12a): shared helpers of the USB bench scripts. Raw termios, DTR asserted, every read bounded."""
import fcntl
import os
import select
import struct
import termios
import threading
import time

CONSOLE = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if00"
LINK = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if02"


def open_raw(dev, baud=termios.B115200):
    """Open a CDC port raw (no echo, no translation) with DTR asserted. The board ignores the baud rate."""
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = attrs[5] = baud
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR))
    return fd


def set_baud(fd, baud):
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = baud
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


class ConsoleReader(threading.Thread):
    """Reads the console until stop() and records every complete line with its host arrival time."""

    def __init__(self, dev=CONSOLE):
        super().__init__(daemon=True)
        self.fd = open_raw(dev)
        self.raw = bytearray()
        self.lines = []  # (host_time, bytes)
        self._partial = b""
        self._halt = threading.Event()

    def run(self):
        while not self._halt.is_set():
            ready, _, _ = select.select([self.fd], [], [], 0.1)
            if not ready:
                continue
            try:
                chunk = os.read(self.fd, 65536)
            except (BlockingIOError, OSError):
                continue
            now = time.time()
            self.raw += chunk
            self._partial += chunk
            while b"\n" in self._partial:
                line, self._partial = self._partial.split(b"\n", 1)
                self.lines.append((now, line + b"\n"))

    def stop(self):
        self._halt.set()
        self.join(timeout=2)
        os.close(self.fd)

    def last(self, pattern):
        """The last captured line (text) containing the regular expression, or None."""
        import re
        for _, line in reversed(self.lines):
            m = re.search(pattern, line.decode(errors="replace"))
            if m:
                return m
        return None
