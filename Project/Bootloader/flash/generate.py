#!/bin/python

import sys
import struct
import hashlib

from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.backends import default_backend

from cryptography.hazmat.primitives import serialization as crypto_serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.backends import default_backend as crypto_default_backend

from serial import Serial
from xmodem import XMODEM

import os.path

privkeyfile = "./keys/private"
publkeyfile = "./keys/public"

if False:
#if not os.path.isfile(privkeyfile):
    key = rsa.generate_private_key(
        backend=crypto_default_backend(),
        public_exponent=65537,
        key_size=1024
    )
    privkey = key.private_bytes(
        crypto_serialization.Encoding.PEM,
        crypto_serialization.PrivateFormat.PKCS8,
        crypto_serialization.NoEncryption())

    public_key = key.public_key().public_bytes(
        crypto_serialization.Encoding.PEM,
        crypto_serialization.PublicFormat.PKCS1
    )

    priv = open(privkeyfile, "w")
    priv.write(privkey)
    priv.close()

    publ = open(publkeyfile, "w")
    publ.write(public_key)
    publ.close()
    print("New public key:")
    print(public_key)
else:
    with open(privkeyfile) as f:
        privkey = serialization.load_pem_private_key(
            f.read(), password=None, backend=default_backend())

messagefile = sys.argv[1]

with open(messagefile) as m:
    message = m.read()
    prehashed = hashlib.sha256(message).hexdigest()

sig = privkey.sign(
    prehashed,
    padding.PSS(
        mgf=padding.MGF1(hashes.SHA256()),
        salt_length=padding.PSS.MAX_LENGTH),
    hashes.SHA256())

f = open("sign", "w")
f.write(sig)
f.close()

print("Size of signature and binary:")
print(len(sig))
print(len(message))

padding = "\0" * (1024 - len(sig))

packed_data = struct.pack("<II", len(sig), len(message))

sys.stdout.write(packed_data)

buf = packed_data + sig + padding + message

f = open("output", "w")
f.write(buf)
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

