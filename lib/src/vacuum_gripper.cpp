#include "vacuum_gripper/vacuum_gripper.h"

using namespace baxter_core_msgs;

namespace ttt
{

Vacuum_Gripper::Vacuum_Gripper(std::string type) : _type(type)
{
    _pub_command = _nh.advertise<EndEffectorCommand>(
                   "/robot/end_effector/" + _type + "_gripper/command", 1);

    _sub_state = _nh.subscribe("/robot/end_effector/" + _type + "_gripper/state", 4,
                                &Vacuum_Gripper::gripperStateCb, this);

    //Initially all the interesting properties of the state are unknown
    EndEffectorState initial_gripper_state; 
    initial_gripper_state.calibrated=   \
    initial_gripper_state.enabled=      \
    initial_gripper_state.error=        \
    initial_gripper_state.gripping=     \
    initial_gripper_state.missed=       \
    initial_gripper_state.ready=        \
    initial_gripper_state.moving=EndEffectorState::STATE_UNKNOWN;

    _state.set(initial_gripper_state);
}

void Vacuum_Gripper::gripperStateCb(const EndEffectorStateConstPtr &msg)
{
    // ROS_INFO("Vacuum_Gripper Callback!");
    _state.set(*msg);
}

void Vacuum_Gripper::suck()
{
    EndEffectorCommand sucking_command;
    sucking_command.id=get_id();
    sucking_command.command=EndEffectorCommand::CMD_GRIP;
    sucking_command.args="{\"grip_attempt_seconds\": 5.0}";
    _pub_command.publish(sucking_command);
}

void Vacuum_Gripper::blow()
{
    EndEffectorCommand release_command;
    release_command.id=get_id();
    release_command.command=EndEffectorCommand::CMD_RELEASE;
    _pub_command.publish(release_command);
}

int Vacuum_Gripper::get_id()
{
    return _state.get().id;
}

bool Vacuum_Gripper::is_enabled()
{
    return _state.get().enabled==EndEffectorState::STATE_TRUE;
}

bool Vacuum_Gripper::is_calibrated()
{
    return _state.get().calibrated==EndEffectorState::STATE_TRUE;
}

bool Vacuum_Gripper::is_ready_to_grip()
{    
    return _state.get().ready==EndEffectorState::STATE_TRUE;
}

bool Vacuum_Gripper::has_error()
{
    return _state.get().error==EndEffectorState::STATE_TRUE;
}

bool Vacuum_Gripper::is_sucking()
{
    // ROS_INFO("force is: %g\n",_state.get().force);
    return _state.get().force==EndEffectorState::FORCE_MAX;
}

bool Vacuum_Gripper::is_gripping()
{
    return true;
    return _state.get().gripping==EndEffectorState::STATE_TRUE;
}

}
