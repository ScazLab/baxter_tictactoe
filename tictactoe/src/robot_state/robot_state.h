#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <ros/ros.h>
#include <baxter_core_msgs/AssemblyState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "src/utils/T_ThreadSafe.h"

namespace ttt
{

class Robot_State
{
private:
    ros::NodeHandle _nh; //! ROS node handle

    ros::Subscriber _sub_robot_state; //! subscriber to receive the messages about the robot state

    ThreadSafeVariable<baxter_core_msgs::AssemblyState> _robot_state; //! Internal variable where the state of the robot is stored. This value is updated every time a /sdk/robot/state topic is received.

    /**
     * It handles the message published about the state of the robot. It updates the internal variable where the state of the robot is stored.
     * \param msg the message with the robot state: enabled, stopped, error, estop_button, and estop_source.
     **/
    void robot_state_handler(const baxter_core_msgs::AssemblyStateConstPtr & msg);

    ros::Publisher _pub_enable;   //! publisher to enable/disable the robot

    ros::Publisher _pub_reset;   //! publisher to reset the robot

public:
    Robot_State();

    bool enable(); //! It enables the robot. This function takes 1s to check if the robot is enabled.

    bool disable(); //! It disables the robot. This function takes 1s to check if the robot is disabled.

    bool reset(); //! It resets all the joints and disable the robot.

    bool is_enabled();

    bool is_disabled();

    bool is_stopped();

    bool has_error();
};

}
#endif // ROBOT_STATE_H
