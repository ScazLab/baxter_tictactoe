#ifndef TRAJECTORY_PLAYER_H
#define TRAJECTORY_PLAYER_H

/**
 * \brief Execution of different types of trajectories
 * \author Alvaro Castro Gonzalez, acgonzal@ing.uc3m.es
 *
 * This class will execute trajectories and inform about the result.
 */

#include <pthread.h>

#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "baxterTictactoe/T_ThreadSafe.h"
#include "arm_controller/gripper.h"

namespace ttt
{

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

typedef actionlib::SimpleClientGoalState Goal_State;

class Trajectory_Player
{
    Client* _client; //! a client to play the trajectories

    ThreadSafeVariable<bool> _tip_collision; //! indicates a potential collision of the left hand tip

    ros::NodeHandle _n; //! ROS node handle

    ros::Subscriber _left_ir_range_sub; //! subscriber to receive the messages comming from the ir range on the left hand
    ros::Subscriber _left_joint_sub; // subscriber to get the joints state

    pthread_mutex_t mutex;
    std::map<std::string,double> left_arm_state;

    /**
     * Check the value returned by the ir range from the left hand.
     * If it is below a specific threshold this is considered as a potential collision and it is informed using the flag variable _tip_collision.
     * \param msg the message with the value of the left range
     **/
    void check_left_ir_range(const sensor_msgs::RangeConstPtr& msg);

    void check_joint_states(const sensor_msgs::JointState& msg);

    const static float IR_RANGE_THRESHOLD; //! value to determine an object close enough to the left hand as a collision considering the ir range

    Gripper *_gripper; //! This is used in the trajectories that imply grasping or releasing

    /**
     * It performs the necessary operations to ensure a correct grasping of an object.
     * \return true if the grasping is successful, false otherwise.
     **/
    bool grasp();

    /**
     * It performs the necessary operations to ensure a correct release of an object.
     * \return true if the grasping is successful, false otherwise.
     **/
    bool release();

    /**
     * Takes care of prepending current position to trajectory and setting goal headers.
     */
    control_msgs::FollowJointTrajectoryGoal trajectory_to_goal(trajectory_msgs::JointTrajectory t);

public:
    /**
     * Constructor where trajectories will be executed using an action server.
     * \param service_name the action server that will be in charge of executing the trajectories
     **/
    Trajectory_Player(const char * service_name);

    /**
     * Play a trajectory without considering any possible collision. This is a blocking funtion.
     * \param t trajectory to be played
     * \return false if an error happens; true otherwise. In case of error the correcteness of the operation is not guarranted.
     **/
    bool run_trajectory(trajectory_msgs::JointTrajectory t);

    /**
     * Play a trajectory and hold an item using the vaccum gripper when a collisions of the tip is detected. This is a blocking funtion.
     * Once the collision is detected, the trajectory is cancelled, and the vacuum gripper is used to grasp an item.
     * \param t trajectory to be played
     * \return false if an error happens; true otherwise. In case of error the correcteness of the operation is not guarranted.
     **/
    bool run_trajectory_and_grasp(trajectory_msgs::JointTrajectory t);

    /**
     * Play a trajectory and release the item holded at the vacuum gripper. This is a blocking funtion.
     * Once the trajectory has successfully ended, the held item is disposed. If the trajectory does not success, the item is not released.
     * \param t trajectory to be played
     * \return false if an error happens; true otherwise. In case of error the correcteness of the operation is not guarranted.
     **/
    bool run_trajectory_and_release(trajectory_msgs::JointTrajectory t);

    /**
     * Destructor
     **/
    ~Trajectory_Player();
};

const float Trajectory_Player::IR_RANGE_THRESHOLD = 0.085;

}
#endif // TRAJECTORY_PLAYER_H
