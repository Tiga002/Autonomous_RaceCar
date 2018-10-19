#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import UInt16, Float32
import time
import serial
import serial
import json
import syslog,time,sys

global steerling_value
steerling_value = 0

global throttle_value
throttle_value = 0
#################################################################

port = '/dev/ttyACM0'

# occupy the port all the time to save time
ard = serial.Serial(port,9600,timeout=1)
#global object_detected_index
def SendToSerial(esc, servo):
    global ard

    send =""
    send += str(int(esc))
    send += " "
    send += str(int(servo))

    ard.flush()
    send = str(send)
    #print ("Python value sent: ")
    #print (send)
    ard.write(send)

    #for testing
    #time.sleep(0.01)
    #msg = ard.readline().strip('\n\r') 
    #print ("Message from arduino: ")
    #print (msg)
    #print ('\n')
    #print msg

    #exit()
#################################################################

# Steering Offset Global Variables
def steerling_callback(data):
    global steerling_value
    
    if data is not None:
        steerling_value = data.data
        #print("subscribing servo~~")
        #print(steerling_value)
    #main_function()
    return 0
def throttle_callback(data):
    global throttle_value
    rate = rospy.Rate(20)
    if data is not None:
        throttle_value = data.data
        #print("subscribing throttle~~")
        #print(throttle_value)
##
    # Command to be sent
 #   servo_CMD = steerling_value
  #  esc_CMD = throttle_value  
    
    # Send the Command
   # SendToSerial(esc_CMD, servo_CMD)
   # print('Servo Command output is :')
   # print(servo_CMD)
   # print('ESC command output is:')
   # print(esc_CMD)
   # print("==============================================")

    return 0

def main_callback(data):
    global steerling_value
    global throttle_value
    SendToSerial(throttle_value, steerling_value)
    print('Servo Command output is :')
    print(steerling_value)
    print('ESC command output is:')
    print(throttle_value)
    print("==============================================")
	
#############################################################
def main_auto():
    # initialize ROS node
    global steerling_value
    global throttle_value
    rospy.init_node('auto_mode', anonymous=True)
    throttle_subscriber = rospy.Subscriber("esc", UInt16, throttle_callback)
    steerling_subscriber = rospy.Subscriber("servo", UInt16, steerling_callback)
    main_sub = rospy.Subscriber("esc", UInt16, main_callback)
    print(throttle_subscriber)
    print(steerling_subscriber)
    rospy.spin()
#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except rospy.ROSInterruptException:
		pass
