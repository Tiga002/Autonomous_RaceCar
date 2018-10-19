import serial
import json
import syslog,time,sys
def bytes_to_int(data):
    result = 0
    for b in data:
       b = ord(b)
       if b!=13 and b!=10:
           result = result * 10 + (b-48)
    return result

port = '/dev/ttyUSB0'
arduino = serial.Serial(port, 9600, timeout=1)
while True:
    then = time.time()
    msg = arduino.readline()
    result = bytes_to_int(msg)
    print("Encoder Value: ")
    print(result)
    print(time.time()-then)
    
print("Process Terminated")
