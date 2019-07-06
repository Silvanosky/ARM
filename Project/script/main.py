#!/usr/bin/env python
import serial
import sys

port = "/dev/ttyUSB0"
baud = 115200

def password_configuration(b, com):
    if b:
        print("No password configured, please type one:")
        pas = sys.stdin.readline()
        rec = write(com, pas)
        print("Password sent and configured")
    else:
        while True:
            print("Password required to continue:")
            pas = sys.stdin.readline()
            rec = write(com, pas)
            if (rec == b"OK"):
                break
            print("Wrong password, try again")
        
def write(com, txt):
    print("Begin writing: " + txt)
    while True:
        com.write(sha_txt.encode('ascii'))
        res = com.readline()
        if res == b"errorBUSY\n":
            print("BUSY")
        elif res ==  b'errorTIMEOUT\n':
            print("TIMEOUT")
        elif res ==  b'errorERROR\n':
            print("ERROR")
        else:
            print("End of transmission")
            return res

def publicKey(com):
    print("Reading public key")
    public = com.readline()
    print("Key length: " + str(len(publicKey)))
    print("Finished reading, RSA public key: " + publicKey.decode("ascii"))

def signature(com):
    with open("sha", 'r') as sha:
        sha_txt = sha.readline()
    sha_txt = sha_txt + '\0'
    print("SHA: " + sha_txt)
    res = write(com, sha_txt)
    print("SHA written")
    print("Receiving signature")
    sign =  com.readline()
    print("Signature: " + str(sign))


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

        # Configure password
        
        password_configuration(com.readline() == b"PASS", com)
        
        # Interactive mode

        while True:
            print("Type 1 to ask public key")
            print("Type 2 to ask signature")
            print("Type 3 to exit program")
            val = sys.stdin.readline()
            if val == 1:
                publicKey(com)
            elif val == 2:
                signature(com)
            elif val == 3:
                break
            else:
                print("Wrong command")

except Exception as e:
    print("Exception thrown: " + str(e))
    exit()
