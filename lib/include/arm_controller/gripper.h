#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include <string>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

#include "baxterTictactoe/T_ThreadSafe.h"

namespace ttt
{

class Gripper
{
private:
    std::string _type; // It identifies the vacuum gripper we are using: left or right

    ros::NodeHandle _nh; // ROS node handle

    ros::Subscriber _sub_state; // subscriber to receive the messages related to the state of the vacuum gripper

    ros::Publisher _pub_command;   // publisher for gripping by sucking

    // It is updated every time a new message with information about the gripper state arrives
    ThreadSafeVariable<baxter_core_msgs::EndEffectorState> _state; 

    // function handling gripper state messages. We keep updated our internal variable related to the gripper state
    void gripperStateCb(const baxter_core_msgs::EndEffectorStateConstPtr& msg); 

public:
    /**
     * Constructor of the class
     * \param gripper This indicates if we are using the left or right vacuum gripper. Possible values are just right or left.
     **/
    Gripper(std::string type);

    /**
     * It makes the vacuum gripper to suck so, in case it is in contact with an object, it will grip it.
     **/
    void suck();

    /**
     * It makes the vacuum gripper to blow air so, in case it has an object graspped, it will release it.
     **/
    void blow();

    int get_id();

    /**
     * Returns a value indicating if the vacuum gripper is enable, so it can be operated.
     * @return True if it is enabled, false otherwise.
     **/
    bool is_enabled();

    /**
     * Returns a value indicating that the calibration has completed, so it will operate properly.
     * @return True if calibration is done, false otherwise.
     **/
    bool is_calibrated();

    /**
     * Returns a value indicating if the vacuum gripper is ready for gripping.
     * @return True if it is ready, false otherwise.
     **/
    bool is_ready_to_grip();

    /**
     * Returns a value indicating if the gripper is in error state.
     * @return True if the gripper is in error state, false otherwise.
     **/
    bool has_error();

    /**
     * Returns a value indicating if the vacuum gripper is sucking.
     * @return True if it is sucking, false otherwise.
     **/
    bool is_sucking();

    /**
     * Returns a value indicating if the gripper has something attached.
     * @return True if it is grippring an object, false otherwise.
     **/
    bool is_gripping();

    /**
     * This function returns the vacuum gripper controlling
     * @return a string containing a unique description for the gripper, so you can easily identify the gripper you are refering to.
     **/
    std::string get_type()
    {
        return _type;
    }

};

}

#endif // __GRIPPER_H__
