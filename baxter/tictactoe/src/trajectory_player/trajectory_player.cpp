#include "trajectory_player.h"

namespace ttt
{

Trajectory_Player::Trajectory_Player(const char * service_name)
{
    _client = new Client(service_name, true);
    ROS_INFO_STREAM("Waiting to connect to service " << service_name);
    ROS_ASSERT_MSG(_client->waitForServer(ros::Duration(10.0)),"Timeout. Service not available.");
    ROS_INFO("Service CONNECTED");
    _tip_collision.set(false);
}

void Trajectory_Player::check_left_ir_range(const sensor_msgs::RangeConstPtr& msg)
{
    // if the distance is between the min and max values and it is below the threshold
    if(msg->range<=msg->max_range && msg->range>=msg->min_range && msg->range<Trajectory_Player::IR_RANGE_THRESHOLD)
    {
        _tip_collision.set(true);
    }
}

bool Trajectory_Player::run_trajectory(trajectory_msgs::JointTrajectory t)
{
    // Action goal
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = t;
    goal.trajectory.header.stamp = ros::Time::now(); // start now
    _client->sendGoal(goal);
    if(!_client->waitForResult(ros::Duration(40.0))) //timeout for complete the trajectory
    {
        ROS_ERROR("Goal not reached.");
        return false;
    }
    ROS_INFO("Goal reached. Ready for next trajectory");
    return true;
}

bool Trajectory_Player::run_trajectory_and_grasp(trajectory_msgs::JointTrajectory t)
{
    _tip_collision.set(false);
    _left_ir_range_sub = _n.subscribe("/robot/range/left_hand_range", 1, &Trajectory_Player::check_left_ir_range, this); //start checking when the tip of the left hand range touches an obstacle

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = t;
    goal.trajectory.header.stamp = ros::Time::now(); // start now
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
        /*********************/
        /* Code for grasping */
        /*********************/
        return true;
    }
    else if(goal_state==Goal_State::SUCCEEDED) //the trajectory has succesfully ended
    {
        ROS_DEBUG_STREAM("Successful trajectory");
        if(_tip_collision.get()){
            ROS_WARN("The left hand tip has collided with an obstacle");
            /*********************/
            /* Code for grasping */
            /*********************/
            return true;
        }
        return false; //the trajectory has successfully ended but there is not item to grasp
    }
    else // if(goal_state==Goal_State::ABORTED || goal_state==Goal_State::LOST || goal_state==Goal_State::PREEMPTED || goal_state==Goal_State::RECALLED || goal_state==Goal_State::REJECTED)
    {
        ROS_WARN_STREAM("Goal not reached. Goal is " << goal_state.toString());
        return false;
    }    
    return true; // This line never has to be executed
}

bool Trajectory_Player::run_trajectory_and_release(trajectory_msgs::JointTrajectory t)
{
    if(this->run_trajectory(t) && _client->getState()==Goal_State::SUCCEEDED)
    {
        /**********************/
        /* Code for releasing */
        /**********************/
        return true;
    }
    return false;
}

}
