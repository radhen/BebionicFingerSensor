import serial
import struct
import numpy as np
import time



PORT = '/dev/ttyACM0'
BAUDRATE = 115200
NUM_P_BOARDS = 1

'''
HEX commands are send to the controller board. 
The CLOSE command for e.g. starts and ends with a 
terminator HEX character i.e x7E. x00 and x01
are for write and read resp. (a bit long). This is followed by 
a 7-bit address of the PBoard. x06 for has a 7-bit add. i.e 3 (000 0011)
and x01 (one bit) for write command, resulting in x06 (0000 0110). 
x0C is for setting the PWM value also called the command code and the rest two are payload
bytes. Ref. PBoard manual for more details on this.  
'''

CLOSE = "\x7E\x08\x0C\x80\x40\x7E"
OPEN = "\x7E\x06\x0C\xC0\x40\x7E"
BREAK = "\x7E\x08\x0C\x03\x00\x7E"

SER = serial.Serial(PORT, BAUDRATE, timeout=0.1, write_timeout=3)
print(SER.isOpen())

def get_addresses():
    while 1:
        try:
            # send 0x2C hex to serial to read board addresses
            SER.write(b"\x2C")
            dataIn = SER.read_until(size=NUM_P_BOARDS+1)
            if len(dataIn)==NUM_P_BOARDS+1:
                return dataIn
        except serial.SerialException as e:
            print ('There is no new data from serial port')

s = get_addresses()
print "Board address(es): "+str(s[1:])


#########################################
#### Emergency break if serial clogs ####
#########################################
# while 1:
#     SER.write(BREAK)


# SER.flushOutput()
# SER.flushInput()

SER.write(CLOSE)
time.sleep(3)
# SER.write(OPEN)
# time.sleep(2)
# SER.write(BREAK)
# time.sleep(2)



SER.write("\x7E\x08\x08\x01\x7E") # Set Position Count = 1
time.sleep(2)
print "Set Position Count = 1"
# SER.write("\x7E\x06\x0C\x80\x7E") # Set PWM
# time.sleep(4)
# print "Set PWM"
SER.write("\x7E\x08\x80\x80\x7E") # Set Enable PID
time.sleep(2)
print "Set Enable PID"
SER.write("\x7E\x08\x81\x03\x00\x00\x00\x7E") # Set Kp=3
time.sleep(2)
print "Set Kp=3"
SER.write("\x7E\x08\x82\x3f\x00\x00\x00\x7E") # Set Ki=0.5 or 0x3f000000
time.sleep(2)
print "Set Ki=0.5 or 0x3f000000"
SER.write("\x7E\x08\x83\x00\x00\x00\x00\x7E") # Set Kd=0
time.sleep(2)
print "Set Kd=0"
SER.write("\x7E\x08\x84\x27\x10\x7E") # Set Target position = 100
time.sleep(3)
print "Set Target position = 100"
SER.write(BREAK)
time.sleep(2)

print ("ANYTHING HAPPENED?")

