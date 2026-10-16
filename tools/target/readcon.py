#!/usr/bin/env python3
# Read the FC3v2 USB console (CDC ACM) for a number of seconds with DTR asserted, raw, and print it. Bench only.
#   python3 tools/target/readcon.py [/dev/ttyACM1] [seconds]
import os, sys, termios, time, select, fcntl, struct
dev=sys.argv[1] if len(sys.argv)>1 else "/dev/ttyACM1"; secs=float(sys.argv[2]) if len(sys.argv)>2 else 3
fd=os.open(dev, os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
a=termios.tcgetattr(fd); a[4]=a[5]=termios.B115200; a[2]|=termios.CLOCAL|termios.CREAD; a[3]&=~(termios.ICANON|termios.ECHO); a[1]&=~termios.OPOST; a[0]&=~termios.ICRNL; termios.tcsetattr(fd,termios.TCSANOW,a)
fcntl.ioctl(fd,0x5416,struct.pack('I',0x002))
buf=b""; t=time.time()
while time.time()-t<secs:
    r,_,_=select.select([fd],[],[],0.2)
    if r:
        try: buf+=os.read(fd,4096)
        except BlockingIOError: pass
os.close(fd); sys.stdout.write(buf.decode(errors='replace'))
