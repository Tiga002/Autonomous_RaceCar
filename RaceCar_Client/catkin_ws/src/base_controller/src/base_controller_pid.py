#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import time
import syslog, time, sys
import math

# Declare the Global Variables

def mapFloat(x, in_min, in_max, out_min, out_max):

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class Base_Controller():
    # Holds all the attributes and methods of a Base Controller 
    
    # Constructor
    def __init__(self):
        # Clear some previous instance's data 
        self.clear()
        # Declare the Subscriber that subscribe "/cmd_vel" from the local_planner
        self.command_subscriber = rospy.Subscriber("ackermann_cmd", AckermannDriveStamped, self.command_CallBack)
        # Declare the Subscriber that subscribe "/odom" { Actual Velocity } from the sensors (ekf_node
        self.feedback_subscriber = rospy.Subscriber("odom", Odometry, self.feedback_CallBack)
        print("I am created") 
        # Initialize the PID parameters
        self.Kp_throttle = float(rospy.get_param('Kp_throttle', '40.0'))
        self.Kd_throttle = float(rospy.get_param('Kd_thorttle', '0.0'))
        self.Kp_steerling = float(rospy.get_param('Kp_steerling', '20.0'))
        self.Kd_steerling = float(rospy.get_param('Kd_steerling', '0.0'))
        

    def clear(self):
        # Desired Input
        self.desired_linear_velocity = 0.0 
        self.desired_steerling_velocity = 0.0
        # Actual Output
        self.actual_linear_velocity = 0.0
        self.actual_steerling_velocity = 0.0
       
        self.throttle_P_Term = 0.0
        self.throttle_D_Term = 0.0
        
        self.steerling_P_Term = 0.0
        self.steerling_D_Term = 0.0

        self.prev_throttle_error = 0.0
        self.prev_steerling_error = 0.0
      
        self.throttle_error = 0.0
        self.steerling_error = 0.0
        
        self.throttle_error_derivative = 0.0  
      
        self.throttle_PWM = 0.0
        self.steerling_PWM = 0.0


    def command_CallBack(self, data):
     
        if data is not None:
            self.desired_linear_velocity = -data.drive.speed 
            self.desired_steerling_velocity = data.drive.steering_angle

        return 0

    def feedback_CallBack(self, data):
   
        if data is not None:
            self.actual_linear_velocity = data.twist.twist.linear.x
            self.actual_steerling_velocity = data.twist.twist.angular.z
    
        return 0
   
    def Throttle_PID_Controller(self):
        self.throttle_error = self.desired_linear_velocity - self.actual_linear_velocity

        self.throttle_error_derivative = self.throttle_error - self.prev_throttle_error 
        self.prev_throttle_error = self.throttle_error

        self.throttle_P_Term = self.Kp_throttle * self.throttle_error
        self.throttle_D_Term = self.Kd_throttle * self.throttle_error_derivative
    
        self.throttle_PWM = 1500 + self.throttle_P_Term + self.throttle_D_Term 
         
        # Set the PWM output Constrains
        if self.throttle_PWM > 2200:
            self.throttle_PWM = 2200
        if self.throttle_PWM < 1000:
            self.throttle_PWM = 1000
 
        return self.throttle_PWM 

    def Steerling_Command_Mapping(self):
        self.steerling_PWM = mapFloat(self.desired_steerling_velocity, -0.8, 0.8, math.pi, 0) * (180/math.pi) # here change to output an angle --> then use servo.write(angle)
        
        return self.steerling_PWM
    
    def Throttle_Command_Mapping(self):
        self.throttle_PWM = mapFloat(self.desired_linear_velocity, -0.2, 0.4, 1300, 1900)
        #debug messages
        print("Desired speed")
        print(self.desired_linear_velocity)
        return self.throttle_PWM




global base_controller_instance
base_controller_instance = None

def main():
    # Initialize and Declare the base_controller_node
    rospy.init_node('base_controller', anonymous=True)

    # Setup the Node running rate
    rate = rospy.Rate(10)
    
    # Create the Base_controller_instance
    global base_controller_instance
    base_controller_instance = Base_Controller()
    
    # Calculate Throttle PWM Command Output
    #Throttle_PWM = base_controller_instance.Throttle_PID_Controller()
    
    #Testing
    Throttle_PWM = base_controller_instance.Throttle_Command_Mapping()
    # Calculate Steerling PWM Command Output
    Steerling_PWM = base_controller_instance.Steerling_Command_Mapping()

    # Declare the Publisher that publish the esc and servo PWM value : "/esc" and "servo" 
    throttle_publisher = rospy.Publisher("esc", UInt16, queue_size=1)
    steerling_publisher = rospy.Publisher("servo", UInt16, queue_size=1)
    
    while not rospy.is_shutdown():
        # Debug messages: 
        #rospy.loginfo("Throttle PWM Output")
        #print(base_controller_instance.throttle_PWM)
        #rospy.loginfo("Steerling PWM Output")
        #print(base_controller_instance.steerling_PWM)
     
        # Create the messages for Data Transmission
        esc_msg = UInt16(Throttle_PWM)
        servo_msg = UInt16(Steerling_PWM)

        # Publish the messages
        throttle_publisher.publish(esc_msg)
        steerling_publisher.publish(servo_msg)

        # Pause the Loop rate while publishing
        rate.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Base_Controller is shutting DOWN !!! ~~")       
    

if __name__ == '__main__':
    try:
        # base_controller_instance = Base_Controller() 
         main()
    except rospy.ROSInterruptException:
        pass
        
