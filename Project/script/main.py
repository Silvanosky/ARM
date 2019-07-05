#!/usr/bin/env python
import serial

port = "/dev/ttyUSB0"
baud = 115200

try:
    # Open serial
    
    com = serial.Serial(port, baud, timeout = None)
    print("Serial communication: " + port + " " + str(baud))
    if com.isOpen():
        print("Serial opened")
    else:
        print("Serial open failed")
        exit()

    # Reading public key

    print("Reading public key")
    publicKey = com.readline()[0:-2]
    print("Finished reading, RSA public key: " + publicKey.decode("ascii"))

    with open("sha", 'r') as sha:
        sha_txt = sha.read()
    print("SHA256 is: " + sha_txt[0:-2])
    com.write(sha_txt.encode('ascii'))
    print("SHA written")

    print("Receiving signature")
    sign = com.readline()
    print("Signature: " + str(sign))

except Exception as e:
    print("Exception thrown: " + str(e))
    exit()
