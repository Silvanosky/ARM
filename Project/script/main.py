#!/usr/bin/env python
import serial
import base64

port = "/dev/ttyUSB0"
baud = 115200

def password_configuration():
    print("No password configured, please type one:")

def debug(f):
    while True:
        s = f.readline()
        print(str(s))

try:
    # Open serial
    
    with serial.Serial(port, baud, timeout = None) as com:
        print("Serial communication: " + port + " " + str(baud))
        if com.isOpen():
            print("Serial opened")
        else:
            print("Serial open failed")
            exit()

        # Reading public key

        print("Reading public key")
        publicKey = ""
        while len(publicKey) < 38:
            publicKey = com.readline()
            print("Length key: " + str(len(publicKey)))
        print("Finished reading, RSA public key: " + publicKey.decode("ascii"))

        with open("sha", 'r') as sha:
            sha_txt = sha.readline()
        sha_txt = sha_txt + '\0'
        print("SHA: " + sha_txt)
        transmitted = False
        res = ""
        while not transmitted:
            com.write(sha_txt.encode('ascii'))
            res = com.readline()
            if res == b"errorBUSY\n":
                print("BUSY")
            elif res ==  b'errorTIMEOUT\n':
                print("TIMEOUT")
            elif res ==  b'errorERROR\n':
                print("ERROR")
            else :
                transmitted = True
        print("Received: " + str(res))

        print("SHA written")
        print("Receiving signature")
        sign =  com.readline()
        print("Signature: " + str(sign))

except Exception as e:
    print("Exception thrown: " + str(e))
    exit()
