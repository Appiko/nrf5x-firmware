#!usr/bin/python3

import serial
import os

port = serial.Serial("/dev/ttyBmpTarg", baudrate=1000000, bytesize = serial.EIGHTBITS, timeout = 40)
flag = 1
end_line = "END\n"
#os.system("make eraseall clean all upload")
line_data = port.readline()
while (flag == 1) :
#    fp = open("file_test.txt", "a")
    line_data = port.readline()
    print(line_data)
#    fp.close()
    if (line_data == end_line):
        flag = 0
port.close()

