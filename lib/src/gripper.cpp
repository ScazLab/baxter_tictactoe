#include "arm_controller/gripper.h"

using namespace baxter_core_msgs;

namespace ttt
{

Gripper::Gripper(std::string type) : _type(type)
{
    _pub_command = _nh.advertise<EndEffectorCommand>(
                   "/robot/end_effector/" + _type + "_gripper/command", 1);

    _sub_state = _nh.subscribe("/robot/end_effector/" + _type + "_gripper/state", 4,
                                &Gripper::gripperStateCb, this);

    //Initially all the interesting properties of the state are unknown
    EndEffectorState initial_gripper_state; 
    initial_gripper_state.calibrated=   \
    initial_gripper_state.enabled=      \
    initial_gripper_state.error=        \
    initial_gripper_state.gripping=     \
    initial_gripper_state.missed=       \
    initial_gripper_state.ready=        \
    initial_gripper_state.moving=EndEffectorState::STATE_UNKNOWN;

    _state = initial_gripper_state;

    pthread_mutex_init(&_mutex, NULL);
}

void Gripper::gripperStateCb(const EndEffectorStateConstPtr &msg)
{
    // ROS_INFO("Gripper Callback");
    pthread_mutex_lock(&_mutex);
    _state = *msg;
    pthread_mutex_unlock(&_mutex);
}

void Gripper::suck()
{
    EndEffectorCommand sucking_command;
    sucking_command.id=get_id();
    sucking_command.command=EndEffectorCommand::CMD_GRIP;
    sucking_command.args="{\"grip_attempt_seconds\": 5.0}";
    _pub_command.publish(sucking_command);
}

void Gripper::blow()
{
    EndEffectorCommand release_command;
    release_command.id=get_id();
    release_command.command=EndEffectorCommand::CMD_RELEASE;
    _pub_command.publish(release_command);
}

int Gripper::get_id()
{
    return _state.id;
}

bool Gripper::is_enabled()
{
    return _state.enabled==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_calibrated()
{
    return _state.calibrated==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_ready_to_grip()
{    
    return _state.ready==EndEffectorState::STATE_TRUE;
}

bool Gripper::has_error()
{
    return _state.error==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_sucking()
{
    // ROS_INFO("force is: %g\n",_state.force);
    return _state.force==EndEffectorState::FORCE_MAX;
}

bool Gripper::is_gripping()
{
    return true;
    return _state.gripping==EndEffectorState::STATE_TRUE;
}

}