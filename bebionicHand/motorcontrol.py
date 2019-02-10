import serial
import struct
import numpy as np



PORT = '/dev/ttyACM0'
BAUDRATE = 115200
NUM_P_BOARDS = 2

SER = serial.Serial(PORT, BAUDRATE, timeout=0.1)
print(SER.isOpen())

def get_addresses():
    while 1:
        try:
            # send 0x2C hex to serial to read board addresses
            SER.write(b"\x2C")
            dataIn = SER.read_until(size=NUM_P_BOARDS)
            if len(dataIn)==NUM_P_BOARDS:
                return dataIn
        except serial.SerialException as e:
            print ('There is no new data from serial port')

# s = get_addresses()
# print s

while 1:
    SER.write("\x7E \x04 \x01 \x01 \x7D")