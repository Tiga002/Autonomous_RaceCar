#ifndef __CAR_JOYSTICK_HPP__
#define __CAR_JOYSTICK_HPP__ 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <mutex>
#include <memory>

#define AXIS_THROTTLE 1
#define AXIS_STEERING 3

#define STEERING_BASE 1500
#define STEERING_RANGE 200

#define THROTTLE_BASE 1500
#define THROTTLE_RANGE 80

class JoystickController {
    private:
        double steering_ = 0;
        double throttle_ = 0;
        std::mutex mutex_;
        ros::Rate rate_;
        ros::NodeHandle node_, private_node_;
        ros::Publisher servo_pub_, esc_pub_;
        ros::Subscriber sub_;

        void recvMsg(const sensor_msgs::Joy::ConstPtr &);
        void pubCmd();
    public:
        JoystickController(
            ros::NodeHandle &,
            ros::NodeHandle &
        );
        bool spin();

};

#endif
