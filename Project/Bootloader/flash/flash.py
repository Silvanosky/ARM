#!/bin/python

import sys
import struct

from serial import Serial
from xmodem import XMODEM

import os.path

messagefile = sys.argv[1]

with open(messagefile) as m:
    message = m.read()

f = open("output", "w")
f.write(message)
f.close()

print("Sending file to board...")

s = Serial(port='/dev/ttyUSB0', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=0, rtscts=0)
#s.open() #Already opened

def getc(size, timeout=1):
    return s.read(size)

def putc(data, timeout=1):
    s.write(data)

modem = XMODEM(getc, putc)
status = modem.send(open("output", "r"), retry=8)

while True:
    data = s.readline()
    if data:
        print data,
s.close()

print(status)

