#!/usr/bin/env python
import serial
import struct
import numpy as np
import time
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from simple_pid import PID
from binascii import unhexlify
from get_data import GetData
from serial import SerialException


'''
HEX commands are send to the controller board. The CLOSE command for e.g. starts and ends with a 
terminator HEX character i.e x7E. x00 and x01 are for write and read resp. (a bit long). This is 
followed by  a 7-bit address of the PBoard. x06 for has a 7-bit add. i.e 3 (000 0011) and x01
(one bit) for write command, resulting in x06 (0000 0110). x0C is for setting the PWM value also 
called the command code and the rest two are payload bytes. Ref. PBoard manual for more details. 
'''


class MotorFunctions(object):

    def __init__(self):
        PORT = '/dev/ttyACM0'
        BAUDRATE = 115200
        self.NUM_P_BOARDS = 1
        self.delay = 0.037  # decided by the sensor samp freq. i.e. 50Hz with motor control loop (5Khz)
        TARG_FORCE = 0.55

        # def make_serial_con   nection(self, port, baudrate):
        try:
            self.ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)  # This must match the port selected in the Arduino IDE
            print "Connected to port:", self.ser.name
            # return ser
        except Exception as ex:
            print "ERROR: COULD NOT ESTABLISH SERIAL CONNECTION WITH", port, ", Check that the port is correct ..."
            print ex


    def get_all_bytes_from_connection(self, connection):
        if connection.in_waiting:
            readResult = connection.read( connection.in_waiting )
            # rtnNums    = [ ord( elem ) for elem in list( readResult ) ]
            print readResult
            return readResult
        else:
            print "No bytes to read!"
            return []


    def get_addresses(self):
        while 1:
            try:
                # send 0x2C hex to serial to read board addresses
                self.ser.write(b"\x2C")
                time.sleep(0.1)
                # get_all_bytes_from_connection(ser)
                dataIn = self.ser.read_until(size=self.NUM_P_BOARDS+1)
                # print dataIn
                if len(dataIn)==self.NUM_P_BOARDS+1:
                    print dataIn
                    return dataIn
            except serial.SerialException as e:
                print ('There is no new data from serial port')


    def set_address(self):
        # he finger. Make sure to apply breaks after calling this function
        sa = b'\x7e\x02\xAD\x01\x06\x7e'
        self.ser.write(sa)
        time.sleep(1)


    def fully_open(self, addr, pwm_value):
        # he finger. Make sure to apply breaks after calling this function
        open = b'\x7e'
        open += bytes(chr(2 * int(addr)))
        open += b'\x0c\xc0'
        # open += b"\x10"
        open += chr(pwm_value)
        open += b'\x7e'
        self.ser.write(open)
        time.sleep(self.delay)


    def fully_close(self, addr, pwm_value):
        # the finger. Make sure to apply breaks after calling this function
        close = b'\x7e'
        close += bytes(chr(2 * int(addr)))
        close += b'\x0c\x80'
        # close += b"\x10"
        close += chr(pwm_value)
        close += b'\x7e'
        self.ser.write(close)
        time.sleep(self.delay)


    def apply_breaks(self, addr):
        # sends pwm = 0 to cut down motor current to zero
        BREAK = b'\x7e'
        BREAK += bytes(chr(2 * int(addr)))
        BREAK += b'\x0c\x03\x00\x7e'
        self.ser.write(BREAK)
        # print "Applying breaks!"
        time.sleep(self.delay)


    def emergency_break(self):
        # Emergency breaks to all: if serial clogs
        while 1:
            print "Applying breaks to all"
            for i in range(len(addList)): apply_breaks(ser,addList[i])
            apply_breaks(addList)


    def debug_open_close_break(self):
        fully_open(ser, addList[0])
        fully_close(ser, addList[0])
        apply_breaks(ser, addList[0])


    def set_position_count(self, addr, value):
        binary_value = bin(value)[2:].zfill(16) # remove the '0b' char from the beginning and zero pad
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
        self.ser.write(pc)
        print "Position Count = {}".format(value)
        time.sleep(self.delay)


    def set_target_position(self, addr, value):
        binary_value = bin(value)[2:].zfill(16) # remove the '0b' char from the beginning and zero pad
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
        self.ser.write(tp)
        print "Target position = {}".format(value)
        time.sleep(self.delay)


    def float_to_hex(self, f):
        return hex(struct.unpack('<I', struct.pack('<f', f))[0])


    def set_pid_gains(self, addr):
        kp = b'\x7e'
        kp += bytes(chr(2 * int(addr)))
        kp += b'\x81'
        kp += b'\x3f000000'
        kp += b'\x7e'
        self.ser.write(kp)
        # ser.write(b"\x7E\x06\x81\x40A00000\x7E") # Set Kp
        print "Kp = 0.5"
        time.sleep(self.delay)

        ki = b'\x7e'
        ki += bytes(chr(2 * int(addr)))
        ki += b'\x81'
        ki += b'\x3da3d70a'
        ki += b'\x7e'
        self.ser.write(ki)
        # ser.write(b"\x7E\x06\x82\x3c23d70a\x7E") # Set Ki
        print "Ki = 0.08"
        time.sleep(self.delay)

        kd = b'\x7e'
        kd += bytes(chr(2 * int(addr)))
        kd += b'\x81'
        kd += b'\x3dcccccd'
        kd += b'\x7e'
        self.ser.write(kd)
        # ser.write(b"\x7E\x06\x83\x3e4ccccd\x7E") # Set Kd=0
        print "Kd = 0.1"
        time.sleep(self.delay)


    def enable_pid(self, addr):
        pid_on = b'\x7e'
        pid_on += bytes(chr(2 * int(addr)))
        pid_on += b'\x80\x80\x7e'
        self.ser.write(pid_on)
        print "Enable PID"
        time.sleep(self.delay)


    def read_pboards(ser):
        self.ser.write("\x7E\x08\x01\x02\x7E") # SetReadTarget
        time.sleep(1)
        self.ser.write("\x7E\x09\x00\x00\x7E") # Sent read command to specific Pboard add.
        time.sleep(1)
        get_all_bytes_from_connection(ser)


if __name__ == "__main__":


    # ser = make_serial_connection(PORT, BAUDRATE)
    mf = MotorFunctions()

    # self.ser.flushInput()
    # self.ser.flushOutput()

    # addrs = mf.get_addresses()
    # print "Board address(es): "+str(addrs[1:])
    # addList = [addrs[i+1] for i in range(len(addrs[1:]))]
    # addList = ['1','2','3','4','5']
    addList = ['5']
    # print addList

    # mf.set_address()

    # for i in addList: mf.fully_close(str(i), 64)
    # time.sleep(0.5)
    # for i in addList: mf.apply_breaks(str(i))

    # time.sleep(1)

    # for i in addList: mf.fully_open(str(i), 64)
    # time.sleep(1)
    # for i in addList: mf.apply_breaks(str(i))

    ############ Testing poistion control thru PID control ###############

    for i in addList: mf.set_position_count(str(i), 5000)
    for i in addList: mf.set_target_position(str(i), 1000)
    for i in addList: mf.set_pid_gains(str(i))
    for i in addList: mf.enable_pid(str(i))
    rospy.sleep(2)
    for i in addList: mf.apply_breaks(str(i))

    #########################################################################

    print "ANYTHING HAPPENED?"

