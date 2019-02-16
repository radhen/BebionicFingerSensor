import serial
import struct
import numpy as np
import time



PORT = '/dev/ttyACM0'
BAUDRATE = 115200
NUM_P_BOARDS = 2

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


def make_serial_connection(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)  # This must match the port selected in the Arduino IDE
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


def fully_open(ser, addr):
    # he finger. Make sure to apply breaks after calling this function
    OPEN = b'\x7e'
    OPEN += bytes(chr(2 * int(addr)))
    OPEN += b'\x0c\xc0\x30\x7e'
    ser.write(OPEN)
    time.sleep(2)


def fully_close(ser, addr):
    # the finger. Make sure to apply breaks after calling this function
    CLOSE = b'\x7e'
    CLOSE += bytes(chr(2 * int(addr)))
    CLOSE += b'\x0c\x80\x30\x7e'
    ser.write(CLOSE)
    time.sleep(2)


def apply_breaks(ser, addr):
    # sends pwm = 0 to cut down motor current to zero
    BREAK = b'\x7e'
    BREAK += bytes(chr(2 * int(addr[0])))
    BREAK += b'\x0c\x03\x00\x7e'
    BREAK += b'\x7e'
    BREAK += bytes(chr(2 * int(addr[1])))
    BREAK += b'\x0c\x03\x00\x7e'
    ser.write(BREAK)
    time.sleep(2)


ser = make_serial_connection(PORT, BAUDRATE)
addrs = get_addresses(ser)
# print "Board address(es): "+str(addrs[1:])
addList = [addrs[i+1] for i in range(len(addrs[1:]))]


##################################################
#### Emergency breaks to all: if serial clogs ####
##################################################
# while 1:
#     print "Applying breaks to all"
#     for i in range(len(addList)): apply_breaks(ser,addList[i])


#########################################
##### Debug: open and close finger ######
#########################################
# fully_open(ser, addList[0])
# fully_close(ser, addList[0])
# apply_breaks(ser, addList[0])



def set_position_count(ser, addr, value):

    binary_value = bin(value)[2:].zfill(16) # remove the '0b' char from the beginning and zero pads
    lsb_bin = binary_value[-8:] # take the last or first 8 digits
    lsb_hex = hex(int(lsb_bin, 2))
    msb_bin = binary_value[:-8] # take the first 8 digits
    msb_hex = hex(int(msb_bin,2))

    pc = b'\x7e'
    pc += bytes(chr(2 * int(addr)))
    pc += b'\x08'
    pc += bytes(chr(int(msb_hex,16)))
    pc += bytes(chr(int(lsb_hex,16)))
    pc += b'\x7e'

    # ser.write(b"\x7E\x06\x08\x0B\xB8\x7E") # originally tested with this single cmmnd
    ser.write(pc)
    print "Position Count = {}".format(value)
    time.sleep(2)


def set_target_position(ser, addr, value):

    binary_value = bin(value)[2:].zfill(16) # remove the '0b' char from the beginning and zero pads
    lsb_bin = binary_value[-8:]  # take the last or first 8 digits
    lsb_hex = hex(int(lsb_bin, 2))
    msb_bin = binary_value[:-8] # take the first 8 digits
    msb_hex = hex(int(msb_bin, 2))

    tp = b'\x7e'
    tp += bytes(chr(2 * int(addr)))
    tp += b'\x84'
    tp += bytes(chr(int(msb_hex, 16)))
    tp += bytes(chr(int(lsb_hex, 16)))
    tp += b'\x7e'

    # ser.write(b"\x7E\x06\x84\x00\x64\x7E") # originally tested with this single cmmnd
    ser.write(tp)
    print "Target position = {}".format(value)
    time.sleep(2)


def float_to_hex(f):
    return hex(struct.unpack('<I', struct.pack('<f', f))[0])


def set_pid_gains(ser, addr):

    kp = b'\x7e'
    kp += bytes(chr(2 * int(addr)))
    kp += b'\x81'
    kp += b'\x40A00000'
    kp += b'\x7e'
    ser.write(kp)
    # ser.write(b"\x7E\x06\x81\x40A00000\x7E") # Set Kp
    print "Kp = 5.0"
    time.sleep(2)

    ki = b'\x7e'
    ki += bytes(chr(2 * int(addr)))
    ki += b'\x81'
    ki += b'\x3c23d70a'
    ki += b'\x7e'
    ser.write(ki)
    # ser.write(b"\x7E\x06\x82\x3c23d70a\x7E") # Set Ki
    print "Ki = 0.01"
    time.sleep(2)

    kd = b'\x7e'
    kd += bytes(chr(2 * int(addr)))
    kd += b'\x81'
    kd += b'\x3c23d70a'
    kd += b'\x7e'
    ser.write(kd)
    # ser.write(b"\x7E\x06\x83\x3e4ccccd\x7E") # Set Kd=0
    print "Kd = 0.5"
    time.sleep(2)


def enable_pid(ser, addr):
    pid_on = b'\x7e'
    pid_on += bytes(chr(2 * int(addr[0])))
    pid_on += b'\x80\x80\x7e'
    pid_on += b'\x7e'
    pid_on += bytes(chr(2 * int(addr[1])))
    pid_on += b'\x80\x80\x7e'
    ser.write(pid_on)
    # ser.write(b"\x7E\x06\x80\x80\x7E") # Set Enable PID
    print "Enable PID"
    time.sleep(2)


set_position_count(ser, addList[0], 2000)
set_target_position(ser, addList[0], 6000)
set_pid_gains(ser, addList[0])

set_position_count(ser, addList[1], 2000)
set_target_position(ser, addList[1], 6000)
set_pid_gains(ser, addList[1])


enable_pid(ser, addList)
apply_breaks(ser, addList)



print ("ANYTHING HAPPENED?")
