#!/usr/bin/env python
import serial
import sys

port = "/dev/ttyUSB0"
baud = 115200
sha_txt = ''

reservedStrings = [ '', '\n', b'PASS\n', b'OK\n', b"errorBUSY\n", b'errorTIMEOUT\n',
        b'errorERROR\n' , b'1\n', b'2\n', b'3\n', b'FINISHED\n', b'ACCESSDENIED\n',
        b'ACCESSGRANTED\n' ]


def password_configuration(com, firsttime):
    waiting = True
    while waiting:
        sha_txt = com.readline()
        print(sha_txt)
        if sha_txt == b'PASS\n':
            waiting = False
    if firsttime:
        print("Enter new password :")
    else:
        print("Enter password:")
    pas = sys.stdin.readline()
    pwdPadding = pas + str(((50 - len(pas)) * b'\0'))
    print(pwdPadding)
    rec = write(com, pwdPadding, b'PASS\n')
    if firsttime:
        print("Password sent and configured")
    else:
        s = ''
        while s != b'ACCESSGRANTED\n' and s != b'ACCESSDENIED\n':
            s = com.readline()
        return s == b'ACCESSGRANTED\n'
    return False

        
def write(com, txt, sup):
    print("Begin writing: " + txt)
    while True:
        com.write(txt.encode('ascii'))
        res = com.readline()
        if res == b"errorBUSY\n":
            print("BUSY")
        elif res ==  b'errorTIMEOUT\n':
            print("TIMEOUT")
        elif res ==  b'errorERROR\n':
            print("ERROR")
        elif res == sup:
            print('RESERVEDWORD')
        else:
            print("End of transmission")
            return res

def publicKey(com):
    print("Reading public key")
    public = ''
    while public in reservedStrings:
        public = com.readline()
        print("receiived: " + public.decode("ascii"))
    print("Key length: " + str(len(public)))
    print("Finished reading, RSA public key: " + public.decode("ascii"))

def signature(com):
    with open("sha", 'r') as sha:
        sha_txt = sha.readline()
    sha_txt = sha_txt + '\0'
    print("SHA: " + sha_txt)
    res = write(com, sha_txt, '')
    print("SHA written")
    print("Receiving signature")
    sign = ''
    while sign in reservedStrings :
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
        
        password_configuration(com, True)
        
        # Interactive mode

        while True:
            print("Type 1 to ask public key")
            print("Type 2 to ask signature")
            print("Type 3 to exit program")
            val = sys.stdin.readline()
            if val == '1\n':
                write(com, '1', '')
                if password_configuration(com, False) :
                    print('ACCES GRANTED')
                    publicKey(com)
                else:
                    print('ACCESS DENIED')
                    
            elif val == '2\n':
                write(com, '2', '')
                if password_configuration(com, False):
                    print('ACCES GRANTED')
                    signature(com)
                else:
                    print('ACCESS DENIED')

            elif val == '3\n':
                write(com, '3', '')
                break

            else:
                print("Wrong command")

except Exception as e:
    print("Exception thrown: " + str(e))
    exit()
