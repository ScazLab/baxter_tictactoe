#include "vacuum_gripper/vacuum_gripper.h"

namespace ttt
{

Vacuum_Gripper::Vacuum_Gripper(vacuum_gripper_type gripper)
{
    _gripper=gripper;

    _pub_command = _nh.advertise<baxter_core_msgs::EndEffectorCommand>(
                   "/robot/end_effector/" + Vacuum_Gripper::type_to_str(_gripper) + "_gripper/command", 1);

    _sub_state = _nh.subscribe("/robot/end_effector/" + Vacuum_Gripper::type_to_str(_gripper)
                               + "_gripper/state", 1, &Vacuum_Gripper::new_state_msg_handler, this);

    //Initially all the interesting properties of the state are unknown
    baxter_core_msgs::EndEffectorState initial_gripper_state; 
    initial_gripper_state.calibrated=   \
    initial_gripper_state.enabled=      \
    initial_gripper_state.error=        \
    initial_gripper_state.gripping=     \
    initial_gripper_state.missed=       \
    initial_gripper_state.ready=        \
    initial_gripper_state.moving=baxter_core_msgs::EndEffectorState::STATE_UNKNOWN;

    _state.set(initial_gripper_state);
}

void Vacuum_Gripper::new_state_msg_handler(const baxter_core_msgs::EndEffectorStateConstPtr &msg)
{
    _state.set(*msg);
}

void Vacuum_Gripper::suck()
{
    baxter_core_msgs::EndEffectorCommand sucking_command;
    sucking_command.id=get_id();
    sucking_command.command=baxter_core_msgs::EndEffectorCommand::CMD_GO;
    sucking_command.args="{\"grip_attempt_seconds\": 5.0}";
    _pub_command.publish(sucking_command);
}

void Vacuum_Gripper::blow()
{
    baxter_core_msgs::EndEffectorCommand release_command;
    release_command.id=get_id();
    release_command.command=baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
    _pub_command.publish(release_command);
}

int Vacuum_Gripper::get_id()
{
    return _state.get().id;
}

bool Vacuum_Gripper::is_enabled()
{
    return _state.get().enabled==baxter_core_msgs::EndEffectorState::STATE_TRUE;
}

bool Vacuum_Gripper::is_calibrated()
{
    return _state.get().calibrated==baxter_core_msgs::EndEffectorState::STATE_TRUE;
}

bool Vacuum_Gripper::is_ready_to_grip()
{    
    return _state.get().ready==baxter_core_msgs::EndEffectorState::STATE_TRUE;
}

bool Vacuum_Gripper::has_error()
{
    return _state.get().error==baxter_core_msgs::EndEffectorState::STATE_TRUE;
}

bool Vacuum_Gripper::is_sucking()
{
    return _state.get().force==baxter_core_msgs::EndEffectorState::FORCE_MAX;
}

bool Vacuum_Gripper::is_gripping()
{
    return true;
    return _state.get().gripping==baxter_core_msgs::EndEffectorState::STATE_TRUE;
}

}
