#ifndef MY_ROS_UTILS_H
#define MY_ROS_UTILS_H

#include <ros/ros.h>

class my_ros_utils
{
public:
    static bool is_ros_ok()
    {
        if (!ros::ok()) {
            ROS_ERROR("ros node is shutted down");
            return false;
        }
        return true;
    }
};

#endif // MY_ROS_UTILS_H
