#!/usr/bin/env python
import serial
import struct
import numpy as np
import time
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from simple_pid import PID
from binascii import unhexlify


'''
HEX commands are send to the controller board. The CLOSE command for e.g. starts and ends with a 
terminator HEX character i.e x7E. x00 and x01 are for write and read resp. (a bit long). This is 
followed by  a 7-bit address of the PBoard. x06 for has a 7-bit add. i.e 3 (000 0011) and x01
(one bit) for write command, resulting in x06 (0000 0110). x0C is for setting the PWM value also 
called the command code and the rest two are payload bytes. Ref. PBoard manual for more details. 
'''


PORT = '/dev/ttyACM1'
BAUDRATE = 115200
NUM_P_BOARDS = 1
DELAY = 0.02 # decided by the sensor samp freq. i.e. 50Hz with motor control loop (5Khz)

TARG_FORCE = 7120000 #Analog baro value. TODO: replace with NN predicted force (N)

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
            time.sleep(0.1)
            # get_all_bytes_from_connection(ser)
            dataIn = ser.read_until(size=NUM_P_BOARDS+1)
            # print dataIn
            if len(dataIn)==NUM_P_BOARDS+1:
                print dataIn
                return dataIn
        except serial.serialException as e:
            print ('There is no new data from serial port')


def fully_open(ser, addr, pwm_value):
    # he finger. Make sure to apply breaks after calling this function
    open = b'\x7e'
    open += bytes(chr(2 * int(addr)))
    open += b'\x0c\xc0'
    # open += b"\x10"
    open += chr(pwm_value)
    open += b'\x7e'
    ser.write(open)
    time.sleep(DELAY)


def fully_close(ser, addr, pwm_value):
    # the finger. Make sure to apply breaks after calling this function
    close = b'\x7e'
    close += bytes(chr(2 * int(addr)))
    close += b'\x0c\x80'
    # close += b"\x10"
    close += chr(pwm_value)
    close += b'\x7e'
    ser.write(close)
    time.sleep(DELAY)


def apply_breaks(ser, addr):
    # sends pwm = 0 to cut down motor current to zero
    BREAK = b'\x7e'
    BREAK += bytes(chr(2 * int(addr)))
    BREAK += b'\x0c\x03\x00\x7e'
    ser.write(BREAK)
    print "Applying breaks!"
    time.sleep(DELAY)


def emergency_break():
    # Emergency breaks to all: if serial clogs
    while 1:
        print "Applying breaks to all"
        for i in range(len(addList)): apply_breaks(ser,addList[i])
        apply_breaks(ser, addList)


def debug_open_close_break():
    fully_open(ser, addList[0])
    fully_close(ser, addList[0])
    apply_breaks(ser, addList[0])


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
    time.sleep(DELAY)


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
    time.sleep(DELAY)


def float_to_hex(f):
    return hex(struct.unpack('<I', struct.pack('<f', f))[0])


def set_pid_gains(ser, addr):
    kp = b'\x7e'
    kp += bytes(chr(2 * int(addr)))
    kp += b'\x81'
    kp += b'\x40a00000'
    kp += b'\x7e'
    ser.write(kp)
    # ser.write(b"\x7E\x06\x81\x40A00000\x7E") # Set Kp
    print "Kp = 5.0"
    time.sleep(DELAY)

    ki = b'\x7e'
    ki += bytes(chr(2 * int(addr)))
    ki += b'\x81'
    ki += b'\x3c23d70a'
    ki += b'\x7e'
    ser.write(ki)
    # ser.write(b"\x7E\x06\x82\x3c23d70a\x7E") # Set Ki
    print "Ki = 0.01"
    time.sleep(DELAY)

    kd = b'\x7e'
    kd += bytes(chr(2 * int(addr)))
    kd += b'\x81'
    kd += b'\x3c23d70a'
    kd += b'\x7e'
    ser.write(kd)
    # ser.write(b"\x7E\x06\x83\x3e4ccccd\x7E") # Set Kd=0
    print "Kd = 0.01"
    time.sleep(DELAY)


def enable_pid(ser, addr):
    pid_on = b'\x7e'
    pid_on += bytes(chr(2 * int(addr)))
    pid_on += b'\x80\x80\x7e'
    # pid_on += b'\x7e'
    # pid_on += bytes(chr(2 * int(addr[1])))
    # pid_on += b'\x80\x80\x7e'
    ser.write(pid_on)
    # ser.write(b"\x7E\x06\x80\x80\x7E") # Set Enable PID
    print "Enable PID"
    time.sleep(DELAY)


def read_pboards(ser):
    ser.write("\x7E\x08\x01\x02\x7E") # SetReadTarget
    time.sleep(1)
    ser.write("\x7E\x09\x00\x00\x7E") # Sent read command to specific Pboard add.
    time.sleep(1)
    get_all_bytes_from_connection(ser)


def push(x, y):
    push_len = len(y)
    assert len(x) >= push_len
    x[:-push_len] = x[push_len:]
    x[-push_len:] = y
    return x


def sub_callback(msg, args):
    # print (msg.data[0])
    # pass

    e_curr = TARG_FORCE - msg.data[0]

    push(sum_e_arr, [e_curr])
    args[0] = sum(sum_e_arr)

    diff_e = e_curr - args[1]
    args[1] = e_curr

    u_t = args[2]*e_curr + args[3]*args[0] + args[4]*diff_e

    # rescale from 1000-500000 to 0-255 (PWM)
    pwm = int(((u_t - 750)/(200000-750)) * 150)
    pwm = np.clip(pwm,0,250)
    print e_curr, args[0], int(u_t), pwm

    y_predict = [e_curr, u_t, pwm]
    msg = Float32MultiArray(MultiArrayLayout([MultiArrayDimension('nn_predictions', 3, 1)], 1), y_predict)
    args[5].publish(msg)


    if -750 < e_curr < 750:
        apply_breaks(ser, addList[0])
    else:
        if pwm > 0:
            fully_close(ser, addList[0], pwm)
            print "closing"
        else:
            fully_open(ser, addList[0], abs(pwm))
            print "opening"


    # print e_curr, args[0], args[1]




if __name__ == "__main__":

    ser = make_serial_connection(PORT, BAUDRATE)
    # addrs = get_addresses(ser)
    # print "Board address(es): "+str(addrs[1:])
    # addList = [addrs[i+1] for i in range(len(addrs[1:]))]
    addList = ['3']
    # print addList

    ############ Testing poistion control thru PID control ###############

    # set_position_count(ser, addList[0], 15000)
    # set_target_position(ser, addList[0], 10000)
    # set_pid_gains(ser, addList[0])

    # set_position_count(ser, addList[1], 15000)
    # set_target_position(ser, addList[1], 10000)
    # set_pid_gains(ser, addList[1])


    # enable_pid(ser, addList[0])
    # enable_pid(ser, addList[1])

    # apply_breaks(ser, addList[0])
    # apply_breaks(ser, addList[1])

    #########################################################################

    rospy.init_node('real_time_testing')

    pid = rospy.Publisher("/pid_output", Float32MultiArray, queue_size=1)

    sum_e = 0
    e_last = 0
    kp = 3.0
    ki = 0.01
    kd = 0.1
    count = 0
    sum_e_arr = np.zeros(50)
    pcf_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, sub_callback, [sum_e, e_last, kp, ki, kd, pid, count, sum_e_arr])
    rospy.spin()

    # fully_close(ser, addList[0], 5)
    # time.sleep(1)
    # fully_open(ser, addList[0], 5)
    # time.sleep(1)
    # apply_breaks(ser, addList[0])


    print ("ANYTHING HAPPENED?")
