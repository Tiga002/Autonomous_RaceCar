#!/usr/bin/env python

# Convert twist_msg into ackermann_msg
# Then Covert ackermann_msg to PWM
# PID is used for throttle Control
# Author: Tiga


import rospy
import math
from std_msgs.msg import UInt16, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0
    
    radius = v / omega
    return math.atan(wheelbase / radius)

def mapFloat(x, in_min, in_max, out_min, out_max):
    
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def feedback_callback(data):
    global feedback_rpm
    feedback_rpm = data.data
    feedback_rpm = int(feedback_rpm)
   # print(type(feedback_rpm))

def command_callback(data):
    global wheelbase
    global ackermann_cmd_topic
    global frame_id
    #global throttle_publisher
    #global steering_publisher
    global throttle_PWM
    global steering_angle
    global desired_velocity
    global desired_rpm
    global wheel_radius
    # Throttle
    #v = data.linear.x
    #throttle_PWM = mapFloat(v, 0, 0.4, 1550, 1580)
    desired_velocity = data.linear.x
    wheel_circumference = 2*math.pi*wheel_radius
    desired_rpm = int((desired_velocity * 60)/wheel_circumference)
    #print(type(desired_RPM))    

    # Ackermann Steering
    #steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
    omega = data.angular.z
    steering_angle = omega * (180/math.pi)
    # If Turning Positive ( Turn Left ) 
    if steering_angle > 0:
        steering_angle = mapFloat(steering_angle, 0, 90, 90, 0)
    # If Turning Negative ( Turn Right)
    elif steering_angle < 0:
        steering_angle = mapFloat(steering_angle, 0, -90, 90, 180)
    # Else stay at Middle
    else:
        steering_angle = 90

  
   
    # For Debug
    #print("throttle command is :" )
    #print(throttle_PWM)
    #print("steering desired is :")
    #print(steering_PWM)
    #print("desired linear speed is :")
    #print(v)
    #print("desired steering angle is")
    #print(steering)
    #print("desired angular about z is")
    #print(data.angular.z

def PID_calculate(desired_rpm, feedback_rpm, P_Gain):
    error = desired_rpm - feedback_rpm 
    PID_Output = 1500 + error * P_Gain
    if PID_Output > 1580:
        PID_Output = 1580
    if PID_Output < 1300:
        PID_Output = 1300
    #debug messages:
    print("Desired RPM: ")
    print(desired_rpm)
    print("Feedback RPM: ")
    print(feedback_rpm)
    print("Error")
    print(error)
    return PID_Output

if __name__ == '__main__':
    global throttle_PWM
    global steering_angle
    global feedback_rpm
    global desired_velocity
    global desired_rpm
    global wheel_radius
    wheel_radius = 0.05
    feedback_rpm = 0
    desired_rpm = 0
    desired_velocity = 0
    throttle_PWM = 1500
    steering_angle = 90
    try:
        
        rospy.init_node('pid_base_controller_node')
        rate = rospy.Rate(10)        
        cmd_vel_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
        feedback_rpm_topic =rospy.get_param('~rpm_topic', '/encoder')
        wheelbase = rospy.get_param('~wheelbase', 0.55)
        P_Gain = rospy.get_param('~P_Gain', 0.65)

        rospy.Subscriber(cmd_vel_topic, Twist, command_callback, queue_size=1)
        rospy.Subscriber(feedback_rpm_topic, UInt16, feedback_callback, queue_size=1) 
        throttle_publisher = rospy.Publisher("esc", UInt16, queue_size=1)
        steering_publisher = rospy.Publisher("servo", UInt16, queue_size=1)
   
        while not rospy.is_shutdown():
            throttle_PWM = PID_calculate(desired_rpm, feedback_rpm, P_Gain)
            esc_msg = int(throttle_PWM)
            servo_msg = int(steering_angle)
            # Debug Messages
            print("Throttle Command is")
            print(esc_msg)
            print("Servo Command is")
            print(servo_msg)

            # Publish the message
            throttle_publisher.publish(esc_msg)
            steering_publisher.publish(servo_msg)
            # Pause the Loop Rate while publishing
            rate.sleep()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Ackermann Base Controller is shutting down ~~ ! ")


    except rospy.ROSInterruptException:
        pass




