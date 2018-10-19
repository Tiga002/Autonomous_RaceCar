#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import UInt16, Float32
import time
import serial
import serial
import json
import syslog,time,sys

global encoder_value
encoder_value = 0
#################################################################

def bytes_to_int(data):
    result = 0
    for b in data:
        b = ord(b)
        if b!=13 and b!=10:
            result = result * 10 + (b-48)
    return result

port = '/dev/ttyUSB0'
global arduino
arduino = serial.Serial(port,9600,timeout=1)

def ReadFromSerial():
    global arduino
    msg = arduino.readline()
    result = bytes_to_int(msg)
    return result


#############################################################
def main_auto():
    # initialize ROS node
    global encoder_value;
    rospy.init_node('encoder_publisher', anonymous=True)
    encoder_publisher = rospy.Publisher('encoder', UInt16, queue_size=10)
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        encoder_value = ReadFromSerial()
        rospy.loginfo(encoder_value)
        encoder_publisher.publish(encoder_value)
        rate.sleep()
#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except rospy.ROSInterruptException:
		pass
