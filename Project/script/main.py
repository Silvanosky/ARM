#!/usr/bin/env python
import serial

port = "/dev/ttyUSB0"
baud = 115200

try:
    com = serial.Serial(port, baud, timeout=10)
    print("Serial communication: " + port + " " + str(baud))
    if com.isOpen():
        print("Serial opened")
    else:
        print("Serial open failed")
        exit()

    #readong public key
    publicKey = ""
    while len(publicKey) < 4:
        print("reading public key")
        publicKey = com.readline()
    print("finished reading, acquired rsa key = " + publicKey)

    with  open("sha", 'r') as sha:
        sha_txt = sha.read()
    print("SHA256 is: " + sha_txt)
    com.write(sha_txt.encode('ascii'))
    print("SHA written")

    sign = ""
    while len(sign) < 4 :
        print("Receiving signature")
        sign = com.readline()
    print("Signature: " + sign)
except exc:
    print("Exception thrown" + exc)
    exit()
