#!/usr/bin/env python

import serial
import json
import syslog,time,sys

def bytes_to_int(bytes):
    result = 0
    for b in bytes:
        if b!=13 and b!=10:
            result = result * 10 + (b-48)
    return result

port = '/dev/ttyUSB0'
arduino = serial.Serial(port, 9600, timeout=1)
while True:
    msg = arduino.readline()
    result = bytes_to_int(msg)
    print("Encoder Value: ")
    print(result)

