#include "trajectory_player.h"

namespace ttt
{

Trajectory_Player::Trajectory_Player(const char * service_name)
{
    _client = new Client(service_name, true);
    ROS_INFO_STREAM("Waiting to connect to service " << service_name);
    ROS_ASSERT_MSG(_client->waitForServer(ros::Duration(10.0)),"Timeout. Service not available. Is the trajectory controller running?");
    ROS_INFO("Service CONNECTED");
    _tip_collision.set(false);

    if (std::string(service_name).find("left")!=std::string::npos) _gripper = new Vacuum_Gripper(left);
    else _gripper = new Vacuum_Gripper(right);

    _left_joint_states=_n.subscribe("/robot/joint_states", 1, &Trajectory_Player::check_joint_states, this);
}

void Trajectory_Player::check_left_ir_range(const sensor_msgs::RangeConstPtr& msg)
{
    // ROS_DEBUG_STREAM("Left IR range=" << msg->range << " (" << msg->min_range << ".." << msg->max_range << ")");
    // if(msg->range<=msg->max_range) ROS_DEBUG("msg->range<=msg->max_range");
    // else ROS_DEBUG("NO msg->range<=msg->max_range");
    // if(msg->range>=msg->min_range) ROS_DEBUG("msg->range>=msg->min_range");
    // else ROS_DEBUG("NO msg->range>=msg->min_range");
    // if(msg->range<=Trajectory_Player::IR_RANGE_THRESHOLD) ROS_DEBUG("msg->range<=Trajectory_Player::IR_RANGE_THRESHOLD");
    // else ROS_DEBUG("NO msg->range<=Trajectory_Player::IR_RANGE_THRESHOLD");

    // if the distance is between the min and max values and it is below the threshold
    if(msg->range<=msg->max_range && msg->range>=msg->min_range && msg->range<=Trajectory_Player::IR_RANGE_THRESHOLD)
    {        
        _tip_collision.set(true);
        ROS_INFO("Obstacle on the field of the left hand gripper");
    }
}

void Trajectory_Player::check_joint_states(const sensor_msgs::JointState& msg)
{
    left_arm_state.clear();

    for (int i = 0; i < 7; ++i)
    {
        left_arm_state[msg.name[i+2]]=msg.position[i+2];
    }
    
    // printf("I heard: ");
    // for (int i = 0; i < 7; ++i)
    // {
    //     printf("[%s %g]\t",msg.name[i+2].c_str(),left_arm_state[msg.name[i+2]]);
    // }
    // printf("\n");
    // ROS_INFO("I heard: [%g %g %g %g %g %g %g]", left_arm_state[0], left_arm_state[1], left_arm_state[2],
    //                           left_arm_state[3], left_arm_state[4], left_arm_state[5], left_arm_state[6]);
}

bool Trajectory_Player::grasp()
{
    // if (_gripper->is_enabled() && _gripper->is_calibrated() && _gripper->is_ready_to_grip()) _gripper->suck();
    // else {
    //     ROS_ERROR("Unexpected gripper state does not allow to grasp %i %i %i",_gripper->is_enabled(),
    //                                         _gripper->is_calibrated(),_gripper->is_ready_to_grip());
    //     return false;
    // }
    _gripper->suck();
    ros::Duration(1).sleep(); //waiting 1s to achieve the grasping
    if (!_gripper->is_gripping())
    {
        ROS_ERROR("Attempt to grasp failed");
        return false;
    }
    return true;
}

bool Trajectory_Player::release()
{
    // if (_gripper->is_enabled() && _gripper->is_calibrated() && _gripper->is_gripping()) _gripper->blow();
    // else {
    //     ROS_ERROR("Unexpected gripper state does not allow to release");
    //     return false;
    // }
    _gripper->blow();
    ros::Duration(1).sleep(); //waiting 1s to perform the releasing
    // if (_gripper->is_gripping())
    // {
    //     ROS_ERROR("Attempt to release failed");
    //     return false;
    // }
    return true;
}

control_msgs::FollowJointTrajectoryGoal Trajectory_Player::trajectory_to_goal(trajectory_msgs::JointTrajectory t)
{
    trajectory_msgs::JointTrajectory full_t; // With current position prepended
    int n_joints=t.joint_names.size();
    
    trajectory_msgs::JointTrajectoryPoint initial_point;
    initial_point.positions.resize(n_joints,0.0);
    initial_point.velocities.resize(n_joints,0.0);
    initial_point.accelerations.resize(n_joints,0.0);
    initial_point.effort.resize(n_joints,0.0);
    initial_point.time_from_start=ros::Duration(0.0);

    printf("trajectory_to_goal: \n");
    for (int i = 0; i < n_joints; ++i)
    {
        initial_point.positions[i]=left_arm_state[t.joint_names[i]];
        printf("[%s %g %g]\t", t.joint_names[i].c_str(), initial_point.positions[i], left_arm_state[t.joint_names[i]]);
    }
    printf("\n");

    full_t.header=t.header;
    full_t.joint_names=t.joint_names;
    full_t.points.resize(t.points.size()+1);
    full_t.points[0]=initial_point;

    for (int i = 0; i < t.points.size(); ++i)
    {
        full_t.points[i+1]=t.points[i];
    }

    // full_t.points = 
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = full_t;
    goal.trajectory.header.stamp = ros::Time::now(); // start now
    return goal;
}

bool Trajectory_Player::run_trajectory(trajectory_msgs::JointTrajectory t)
{
    ROS_INFO("test");

    // Action goal
    control_msgs::FollowJointTrajectoryGoal goal=trajectory_to_goal(t);
    _client->sendGoal(goal);

    if(!_client->waitForResult(ros::Duration(40.0))) //timeout for complete the trajectory
    {
        ROS_ERROR("Timeout! Goal not reached.");
        return false;
    }
    if (_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal reached. Ready for next trajectory");
        return true;
    }
    else
    {
        ROS_ERROR("Goal not reached.");
        ROS_ERROR("Current State: %s", _client->getState().toString().c_str());
    }

    return true; // THIS IS BAD

    return false;
}

bool Trajectory_Player::run_trajectory_and_grasp(trajectory_msgs::JointTrajectory t)
{
    _tip_collision.set(false);
    _left_ir_range_sub = _n.subscribe("/robot/range/left_hand_range", 1, &Trajectory_Player::check_left_ir_range, this); //start checking when the tip of the left hand range touches an obstacle

    control_msgs::FollowJointTrajectoryGoal goal=trajectory_to_goal(t);
    _client->sendGoal(goal);

    actionlib::SimpleClientGoalState goal_state = _client->getState();
    while(!_tip_collision.get() && (goal_state==Goal_State::PENDING || goal_state==Goal_State::ACTIVE))
    {
        ros::Duration(0.2).sleep();
        goal_state = _client->getState();
        ROS_DEBUG_STREAM("action client state is " << goal_state.toString());
    }
    _left_ir_range_sub.shutdown(); //stop checking the left ir range

    if (goal_state==Goal_State::PENDING || goal_state==Goal_State::ACTIVE) //there is a collision: _tip_collision.get()==true
    {
        _client->cancelGoal();
        ROS_WARN("The left hand tip has collided with an obstacle");        
        return this->grasp();
    }
    else if(goal_state==Goal_State::SUCCEEDED) //the trajectory has succesfully ended
    {
        ROS_DEBUG_STREAM("Successful trajectory");
        // if(_tip_collision.get()){
        if(true){
            ROS_WARN("The left hand tip has collided with an obstacle");
            return this->grasp();
        }
        ROS_WARN("Item to grasp not found");
        return false; //the trajectory has successfully ended but there is not item to grasp
    }
    else // if(goal_state==Goal_State::ABORTED || goal_state==Goal_State::LOST || goal_state==Goal_State::PREEMPTED || goal_state==Goal_State::RECALLED || goal_state==Goal_State::REJECTED)
    {
        ROS_WARN_STREAM("Goal not reached. State is " << goal_state.toString());
        return false;
    }    
    return true; // This line never has to be executed
}

bool Trajectory_Player::run_trajectory_and_release(trajectory_msgs::JointTrajectory t)
{
    // if(this->run_trajectory(t) && _client->getState()==Goal_State::SUCCEEDED)
    if(this->run_trajectory(t)) // THIS IS BAD
    {
        return this->release();
    }
    return false;
}

Trajectory_Player::~Trajectory_Player()
{
    _left_joint_states.shutdown();

    if (_client)
    {
        delete _client;
        _client = 0;
    }
    if (_gripper)
    {
        delete _gripper;
        _gripper = 0;
    }
}

}
