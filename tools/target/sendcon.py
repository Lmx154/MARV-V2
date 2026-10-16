#!/usr/bin/env python3
# Send lines to the FC3v2 USB console (CDC ACM) with DTR asserted and print everything it answers, with the time of each
# chunk. Bench only. Steps run in order:
#   send:<text>   the text and a line feed          raw:<text>   the text, no line feed
#   wait:<s>      read for s seconds                close        drop DTR and close the port (a USB disconnect as the
#                                                              firmware sees it), then reopen it with DTR asserted
#   python3 tools/target/sendcon.py /dev/ttyACM2 "send:motor_test 1 100 500 PROPS-OFF" wait:1
import os, sys, termios, time, select, fcntl, struct

TIOCMBIC, TIOCMBIS, TIOCM_DTR = 0x5417, 0x5416, 0x002
dev = sys.argv[1]
t0 = time.time()


def open_port():
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[4] = a[5] = termios.B115200
    a[2] |= termios.CLOCAL | termios.CREAD
    a[3] &= ~(termios.ICANON | termios.ECHO)
    a[1] &= ~termios.OPOST
    a[0] &= ~termios.ICRNL
    termios.tcsetattr(fd, termios.TCSANOW, a)
    fcntl.ioctl(fd, TIOCMBIS, struct.pack('I', TIOCM_DTR))
    return fd


def read_for(fd, secs):
    end = time.time() + secs
    buf = b""
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try:
                buf += os.read(fd, 4096)
            except BlockingIOError:
                pass
    for line in buf.decode(errors="replace").splitlines():
        print(f"[{time.time() - t0:7.3f}] {line}", flush=True)


fd = open_port()
for step in sys.argv[2:]:
    kind, _, arg = step.partition(":")
    if kind == "send":
        print(f"[{time.time() - t0:7.3f}] > {arg}", flush=True)
        os.write(fd, arg.encode() + b"\n")
    elif kind == "raw":
        print(f"[{time.time() - t0:7.3f}] > (raw) {arg!r}", flush=True)
        os.write(fd, arg.encode())
    elif kind == "wait":
        read_for(fd, float(arg))
    elif kind == "close":
        print(f"[{time.time() - t0:7.3f}] > (close port)", flush=True)
        fcntl.ioctl(fd, TIOCMBIC, struct.pack('I', TIOCM_DTR))
        os.close(fd)
        time.sleep(0.3)
        fd = open_port()
    else:
        sys.exit(f"unknown step {step!r}")
os.close(fd)
