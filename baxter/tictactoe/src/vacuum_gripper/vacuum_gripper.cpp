#include "vacuum_gripper.h"

namespace ttt
{

Vacuum_Gripper::Vacuum_Gripper(vacuum_gripper_type gripper)
{
    _gripper=gripper;

    _pub_command_grip = _nh.advertise<std_msgs::Float32>("/robot/limb/" + Vacuum_Gripper::type_to_str(_gripper) + "/accessory/gripper/command_grip", 1);
    ROS_ASSERT_MSG(_pub_command_grip,"Empty publisher for gripping");
    _pub_command_release = _nh.advertise<std_msgs::Empty>("/robot/limb/" + Vacuum_Gripper::type_to_str(_gripper) + "/accessory/gripper/command_release", 1);
    ROS_ASSERT_MSG(_pub_command_release,"Empty publisher for releasing");

    _sub_state = _nh.subscribe("/sdk/robot/limb/" + Vacuum_Gripper::type_to_str(_gripper) + "/accessory/gripper/state", 1, &Vacuum_Gripper::new_state_msg_handler, this);

    baxter_msgs::GripperState initial_gripper_state; //Initially all the interesting properties of the state are unknown
    initial_gripper_state.calibrated=   \
    initial_gripper_state.enabled=      \
    initial_gripper_state.error=        \
    initial_gripper_state.gripping=     \
    initial_gripper_state.missed=       \
    initial_gripper_state.ready=        \
    initial_gripper_state.moving=baxter_msgs::GripperState::STATE_UNKNOWN;

    _state.set(initial_gripper_state);
}

void Vacuum_Gripper::new_state_msg_handler(const baxter_msgs::GripperStateConstPtr &msg)
{
    _state.set(*msg);
}


void Vacuum_Gripper::suck()
{
    std_msgs::Float32 sucking_position;
    sucking_position.data=baxter_msgs::GripperState::POSITION_CLOSED;
    _pub_command_grip.publish(sucking_position);
}

void Vacuum_Gripper::blow()
{
    std_msgs::Empty em;
    _pub_command_release.publish(em);
}

bool Vacuum_Gripper::is_enabled()
{
    baxter_msgs::GripperState aux=_state.get();
    return _state.get().enabled==baxter_msgs::GripperState::STATE_TRUE;
}

bool Vacuum_Gripper::is_calibrated()
{
    return _state.get().calibrated==baxter_msgs::GripperState::STATE_TRUE;
}

bool Vacuum_Gripper::is_ready_to_grip()
{    
    return _state.get().ready==baxter_msgs::GripperState::STATE_TRUE;
}

bool Vacuum_Gripper::has_error()
{
    return _state.get().error==baxter_msgs::GripperState::STATE_TRUE;
}

bool Vacuum_Gripper::is_sucking()
{
    return _state.get().position==baxter_msgs::GripperState::POSITION_CLOSED;
}

bool Vacuum_Gripper::is_blowing()
{
    return _state.get().position==baxter_msgs::GripperState::POSITION_OPEN;
}

bool Vacuum_Gripper::is_gripping()
{
    return _state.get().gripping==baxter_msgs::GripperState::STATE_TRUE;
}

}
