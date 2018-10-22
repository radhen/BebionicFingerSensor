## readSkin.py
##
## Simple script to read in from the serial port and write to a file. Waits for the char 's' to
## begin saving the data in a text file. Intergrated with the arduino script which also waits for
## char 's' to beign publishing the data on the serial port.

import time
import serial
from sys import platform
import sys

# File to write to, should be command line arguments
if len(sys.argv) < 2:
  print ("Error: No filename to save to!")
  exit()
FILENAME = sys.argv[1]

# Should the data be written to the console also?
ECHO = True

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 57600
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS


# Open the serial port
ser = serial.Serial(port=SERIAL_PORT,baudrate=BAUDRATE, parity=PARITY,stopbits=STOPBITS,bytesize=BYTESIZE)
print ("sleeping for 3s..")
time.sleep(3)
print ("slept for 3s..")
# Did the serial port open?
if not ser.isOpen():
   print ("Error:  Serial port did not open!")
   exit()

# Open the file to write to
out_file = open(FILENAME, 'w')

# Continue to read the serial port until we should stop
readSerial = True

# dummy = ser.readline()

dur = 0.0
# cmmnd = input('press s to start.\n')
# ser.write(b"cmmnd")
# print ('char s written on serial..')
# Flush anything that is in the buffer, and read in a dummy line
ser.flushInput()

with open(FILENAME,'w',buffering=1) as out_file:
    while ser.read():
        data = ser.readline()
        # data_raw = data.split()
        data = str(data).encode("utf-8") # convert bytes to string
        print(data)
        out_file.write(data)
        # out_file.flush()

# try:
# except KeyboardInterrupt:
#     time.sleep(5)
#     ser.close()
#     out_file.close()
