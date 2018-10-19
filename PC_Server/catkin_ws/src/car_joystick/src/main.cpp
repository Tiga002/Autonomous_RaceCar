#include <iostream>
#include <ros/ros.h>
#include "car_joystick.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "js_controller");

    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    JoystickController js_controller(node, private_node);
    js_controller.spin();
}
