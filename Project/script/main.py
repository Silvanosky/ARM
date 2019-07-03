import serial

port = "/dev/ttyUSB0"
baud = 115200

try:
    com = serial.Serial(port, baud)
    print("Serial communication: " + port + " " + str(baud))
    if com.isOpen():
        print("Serial opened")
    else:
        print("Serial open failed")
        exit()

    sha = open("sha")
    sha_txt = sha.read()
    print("SHA256 is: " + sha_txt)
    com.write(sha_txt.encode('ascii'))
    print("SHA written")

    print("Receiving signature")
    sign = com.read()
    print("Signature: " + sign)
except:
    print("Exception thrown")
    exit()
