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

CLOSE = "\x7E\x06\x0C\x80\x30\x7E"
OPEN = "\x7E\x06\x0C\xC0\x30\x7E"
BREAK = "\x7E\x06\x0C\x03\x00\x7E"



def make_serial_connection(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)  # This must match the port selected in the Arduino IDE
        print "Connected to port:", ser.name
        return ser
    except Exception as ex:
        print "ERROR: COULD NOT ESTABLISH SERIAL CONNECTION WITH", port, ", Check that the port is correct ..."
        print ex


def get_all_bytes_from_connection( connection ):
    if connection.in_waiting:
        readResult = connection.read( connection.in_waiting )
        # rtnNums    = [ ord( elem ) for elem in list( readResult ) ]
        print readResult
        return readResult
    else:
        print "No bytes to read!"
        return []


def get_addresses(ser):
    while 1:
        try:
            # send 0x2C hex to serial to read board addresses
            ser.write(b"\x2C")
            time.sleep(1)
            # get_all_bytes_from_connection(ser)
            dataIn = ser.read_until(size=NUM_P_BOARDS+1)
            if len(dataIn)==NUM_P_BOARDS+1:
                print dataIn
                return dataIn
        except serial.serialException as e:
            print ('There is no new data from serial port')


def read_pboards(ser):
    ser.write("\x7E\x08\x01\x06\x7E") # SetReadTarget
    time.sleep(1)
    ser.write("\x7E\x09\x00\x00\x7E") # Sent read command to specific Pboard add.
    time.sleep(1)
    get_all_bytes_from_connection(ser)


ser = make_serial_connection(PORT, BAUDRATE)
get_addresses(ser)
# print "Board address(es): "+str(s[1:])


#########################################
#### Emergency break: if serial clogs ####
#########################################
# while 1:
#     ser.write(BREAK)


#########################################
#### Debug: open and close finger ####
#########################################
# ser.write(CLOSE)
# time.sleep(1)
# ser.write(OPEN)
# time.sleep(1)
# ser.write(BREAK)
# time.sleep(2)




ser.write(b"\x7E\x06\x08\x0B\xB8\x7E") # Set Position Count = 3500
print "Set Position Count = 3000"
time.sleep(2)

ser.write(b"\x7E\x06\x81\x40A00000\x7E") # Set Kp
print "Set Kp = 5.0"
time.sleep(2)

ser.write(b"\x7E\x06\x82\x3c23d70a\x7E") # Set Ki
print "Set Ki = 0.01"
time.sleep(2)

ser.write(b"\x7E\x06\x83\x3e4ccccd\x7E") # Set Kd=0
print "Set Kd = 0.5"
time.sleep(2)

ser.write(b"\x7E\x06\x84\x00\x64\x7E") # Set Target position = 4000
print "Set Target position = 100"
time.sleep(2)

ser.write(b"\x7E\x06\x80\x80\x7E") # Set Enable PID
print "Set Enable PID"
time.sleep(2)

ser.write(BREAK)
print "Applying breaks"
time.sleep(2)


print ("ANYTHING HAPPENED?")
