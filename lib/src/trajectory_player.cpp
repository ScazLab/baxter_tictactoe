#include "move_maker/trajectory_player.h"

using namespace ttt;

Trajectory_Player::Trajectory_Player(const char * service_name)
{
    _client = new Client(service_name, true);
    ROS_INFO_STREAM("[Trajectory_Player] Waiting to connect to service " << service_name);

    ROS_ASSERT_MSG(_client->waitForServer(ros::Duration(10.0)),
                   "[Trajectory_Player] Service not available. Is the trajectory controller running?");
    
    ROS_INFO("[Trajectory_Player] Service CONNECTED");
    _tip_collision.set(false);

    if (std::string(service_name).find("left")!=std::string::npos) _gripper = new Gripper("left");
    else _gripper = new Gripper("right");

    pthread_mutex_init(&this->mutex, NULL);
    _left_joint_sub=_n.subscribe("/robot/joint_states", 1, &Trajectory_Player::check_joint_states, this);
}

void Trajectory_Player::check_left_ir_range(const sensor_msgs::RangeConstPtr& msg)
{
    // If the distance is between the min and max values and it is below the threshold
    if (msg->range<=msg->max_range && msg->range>=msg->min_range &&
        msg->range<=IR_RANGE_THRESHOLD)
    {        
        _tip_collision.set(true);
        ROS_WARN("[Trajectory_Player] Obstacle on the field of the left hand gripper");
    }
}

void Trajectory_Player::check_joint_states(const sensor_msgs::JointState& msg)
{
    ROS_DEBUG("[Trajectory_Player] Check Joint States - START");
    pthread_mutex_lock(&this->mutex);

    // TODO For some reasons, there are multiple publishers to the same topic. We need to investigate.
    if (msg.name.size() == 17)
    {
        left_arm_state.clear();

        // printf("size of the message %lu\n", msg.name.size());
        for (int i = 0; i < 7; ++i)
        {
            // printf("joint state %i %s %g \n", i, msg.name[i+2].c_str(), msg.position[i+2]);
            left_arm_state[msg.name[i+2]]=msg.position[i+2];
        }   
    }
    else
    {
        ROS_DEBUG("[Trajectory_Player] WARNING Received wrong left arm state from topic.");
    }

    // ROS_INFO("[Trajectory_Player] Check Joint States - ENDING");
    
    pthread_mutex_unlock(&this->mutex);
}

bool Trajectory_Player::grasp()
{
    return _gripper->gripObject();
}

bool Trajectory_Player::release()
{
    return _gripper->releaseObject();
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

    pthread_mutex_lock(&this->mutex);
    std::stringstream s;
    for (int i = 0; i < n_joints; ++i)
    {
        initial_point.positions[i]=left_arm_state[t.joint_names[i]];
        s << "[ " << t.joint_names[i].c_str() << " " << initial_point.positions[i] << " "
          << left_arm_state[t.joint_names[i]] << "] ";
    }

    pthread_mutex_unlock(&this->mutex);
    ROS_INFO("[Trajectory_Player] Trajectory to goal: %s", s.str().c_str());

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
    ROS_INFO("[Trajectory_Player] Running Trajectory..");

    // Action goal
    control_msgs::FollowJointTrajectoryGoal goal=trajectory_to_goal(t);
    _client->sendGoal(goal);

    if(!_client->waitForResult(ros::Duration(40.0))) //timeout for complete the trajectory
    {
        ROS_ERROR("[Trajectory_Player] Timeout! Goal not reached.");
        return false;
    }
    if (_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("[Trajectory_Player] Goal reached. Ready for next trajectory");
        return true;
    }
    else
    {
        ROS_ERROR("[Trajectory_Player] Goal not reached. State is: %s", _client->getState().toString().c_str());
    }

    return true; // THIS IS BAD (by Ale & Olivier)

    return false;
}

bool Trajectory_Player::run_trajectory_and_grasp(trajectory_msgs::JointTrajectory t)
{
    _tip_collision.set(false);
    _left_ir_range_sub = _n.subscribe("/robot/range/left_hand_range", 1,
                                      &Trajectory_Player::check_left_ir_range, this);

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

    if (goal_state==Goal_State::PENDING || goal_state==Goal_State::ACTIVE) 
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
    else // Goal_State::ABORTED || LOST || PREEMPTED || RECALLED || REJECTED)
    {
        ROS_WARN_STREAM("Goal not reached. State is " << goal_state.toString());

    }    

    return true; // THIS IS BAD (by Ale & Olivier)

    return false;
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
    _left_joint_sub.shutdown();

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
