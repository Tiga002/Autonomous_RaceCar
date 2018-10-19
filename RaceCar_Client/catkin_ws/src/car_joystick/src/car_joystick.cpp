#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt16.h>

#include <memory>
#include <mutex>
#include <algorithm>
#include <map>

#include "car_joystick.hpp"

JoystickController::JoystickController(ros::NodeHandle &node, ros::NodeHandle &private_node):
    steering_(0),
    throttle_(0),
    rate_(10),
    node_(node), 
    private_node_(private_node) {
    sub_ = node_.subscribe("joy", 1, &JoystickController::recvMsg, this);
    servo_pub_ = node_.advertise<std_msgs::UInt16>("servo", 32);
    esc_pub_ = node_.advertise<std_msgs::UInt16>("esc", 32);
}

void JoystickController::recvMsg(const sensor_msgs::Joy::ConstPtr &msg) {
    std::lock_guard<std::mutex> mutex_lock(mutex_);

    throttle_ = msg->axes[AXIS_THROTTLE];
    steering_ = msg->axes[AXIS_STEERING] * -1;
}

void JoystickController::pubCmd() {
    std::lock_guard<std::mutex> mutex_lock(mutex_);
    std_msgs::UInt16 throttle_cmd, steering_cmd;
    std::cout << "throttle: " <<  THROTTLE_BASE + THROTTLE_RANGE * throttle_ << std::endl;
    std::cout << "steering: " <<  STEERING_BASE + STEERING_RANGE * steering_ << std::endl;
    throttle_cmd.data = THROTTLE_BASE + THROTTLE_RANGE * throttle_; 
    esc_pub_.publish(throttle_cmd);
    steering_cmd.data = STEERING_BASE + STEERING_RANGE * steering_;
    servo_pub_.publish(steering_cmd);

}

bool JoystickController::spin() {
    while (node_.ok() ) {
        pubCmd();
        ros::spinOnce();
        if (!rate_.sleep()) {
            return false;
        }
    }
    return true;
}
