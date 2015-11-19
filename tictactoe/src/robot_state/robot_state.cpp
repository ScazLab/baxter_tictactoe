#include "robot_state.h"

namespace ttt
{

void Robot_State::robot_state_handler(const baxter_core_msgs::AssemblyStateConstPtr &msg)
{
    _robot_state.set(*msg);
}

Robot_State::Robot_State()
{
    _sub_robot_state = _nh.subscribe("/sdk/robot/state",1,  &Robot_State::robot_state_handler,this);

    _pub_enable = _nh.advertise<std_msgs::Bool>("/robot/set_super_enable", 1);
    ROS_ASSERT_MSG(_pub_enable,"Empty publisher for enabling/disabling");

    _pub_reset = _nh.advertise<std_msgs::Empty>("/robot/set_super_reset", 1);
    ROS_ASSERT_MSG(_pub_reset,"Empty publisher to reset");
}

bool Robot_State::enable()
{
    if (this->is_stopped()) {
        ROS_ERROR_COND(!this->reset(), "Reset not successful");
    }
    std_msgs::Bool msg;
    msg.data=true;
    _pub_enable.publish(msg);
    ros::Duration(1).sleep(); //Let's wait one second for the change to be effective
    return _robot_state.get().enabled;
}

bool Robot_State::disable()
{
    std_msgs::Bool msg;
    msg.data=false;
    _pub_enable.publish(msg);
    ros::Duration(1).sleep(); //Let's wait one second for the change to be effective
    return !_robot_state.get().enabled;
}

bool Robot_State::reset()
{
    std_msgs::Empty msg;
    _pub_reset.publish(msg);
    ros::Duration(1).sleep(); //Let's wait one second for the change to be effective
    return this->disable() && \
          !this->has_error() && \
           !this->is_stopped() && \
           _robot_state.get().estop_button==baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED && \
           _robot_state.get().estop_source==baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
}

bool Robot_State::is_enabled()
{
    return _robot_state.get().enabled;
}

bool Robot_State::is_disabled()
{
    return !_robot_state.get().enabled;
}

bool Robot_State::is_stopped()
{
    return _robot_state.get().stopped;
}

bool Robot_State::has_error()
{
    return _robot_state.get().error;
}

}
