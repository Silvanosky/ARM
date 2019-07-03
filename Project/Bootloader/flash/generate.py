#!/bin/python

import sys
import struct

signsize = 42
filesize = 42

packed_data = struct.pack("<II", signsize, filesize)

buf = packed_data

sys.stdout.buffer.write(buf)
