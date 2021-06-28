import serial
import time 

ser = serial.Serial(
    port='/dev/ttyACM0',\
    baudrate=9600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)

#this will store the line
line = []

while True:
    for c in ser.read():
        line.append(chr(c))
        if chr(c) == '\n':
            print("Line: " + ''.join(line))
            line = []
            break

ser.close()
