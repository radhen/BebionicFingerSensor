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
  print "Error: No filename to save to!"
  exit()
FILENAME = sys.argv[1]

# Should the data be written to the console also?
ECHO = True

# Serial port information.  Should be command line arguments, but later
# if platform == "linux" or platform == "linux2":
#     SERIAL_PORT = '/dev/ttyACM0'
# elif platform == "win32":
#     SERIAL_PORT = 'COM3'
SERIAL_PORT = "/dev/cu.usbmodem11"
BAUDRATE = 115200
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
   print "Error:  Serial port did not open!"
   exit()

# Open the file to write to
out_file = open(FILENAME, 'w')

# Continue to read the serial port until we should stop
readSerial = True

# dummy = ser.readline()

dur = 0.0
cmmnd = raw_input('press s to start.\n')
ser.write(str(cmmnd))
print ('char s written on serial..')
# Flush anything that is in the buffer, and read in a dummy line
# ser.flushInput()

try:
   # start = time.time()

   while readSerial:

      # Is there stuff to read?
        # num_bytes = ser.inWaiting()
        # if num_bytes > 0:
        data = ser.readline()
        # data_raw = data.split()
        out_file.write(data)
        if ECHO:
            print(data)

        # end = time.time()
        # dur = end-start

        # if dur >= 30.0:
        #     readSerial=False
        #     print
        #     print "30 Seconds!"
        time.sleep(0.1)

except KeyboardInterrupt:
   # Read in the last bit of data
   data = ser.readline()
   out_file.write(data)

out_file.close()
print dur
