#ifndef VACUUM_GRIPPER_H
#define VACUUM_GRIPPER_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

#include "src/utils/T_ThreadSafe.h"

namespace ttt
{

typedef enum {left, right} vacuum_gripper_type;

class Vacuum_Gripper
{
private:
    vacuum_gripper_type _gripper; //! It identifies the vacuum gripper we are using: left or right

    ros::NodeHandle _nh; //! ROS node handle

    ros::Subscriber _sub_state; //! subscriber to receive the messages related to the state of the vacuum gripper

    ros::Publisher _pub_command_grip;   //! publisher for gripping by sucking
    ros::Publisher _pub_command_release;//! publisher for releasing by blowing

    ThreadSafeVariable<baxter_core_msgs::EndEffectorState> _state; //! it is update every time a new message with information about the gripper state arrives
    void new_state_msg_handler(const baxter_core_msgs::EndEffectorStateConstPtr& msg); //! function handling gripper state messages. We keep updated our internal variable related to the gripper state

public:
    /**
     * Constructor of the class
     * \param gripper This indicates if we are using the left or right vacuum gripper. Possible values are just right or left.
     **/
    Vacuum_Gripper(vacuum_gripper_type gripper);

    /**
     * It makes the vacuum gripper to suck so, in case it is in contact with an object, it will grip it.
     **/
    void suck();

    /**
     * It makes the vacuum gripper to blow air so, in case it has an object graspped, it will release it.
     **/
    void blow();

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
     * Returns a value indicating if the vacuum gripper is blowing.
     * @return True if it is blowing, false otherwise.
     **/
    bool is_blowing();

    /**
     * Returns a value indicating if the gripper has something attached.
     * @return True if it is grippring an object, false otherwise.
     **/
    bool is_gripping();

    /**
     * Static function to return a string corresponding to a type of vaccum gripper (left or right)
     * @param gripper A variable of type vacuum_gripper_type that will be transformed to a string
     * @return A string corresponding with the value of the parameter gripper. So far, the possible returned values are "left" if gripper==left or "right" if gripper==right
     **/
    static std::string type_to_str(vacuum_gripper_type gripper)
    {
        return gripper==left?"left":"right";
    }

    /**
     * This function returns a descriptive name of the vacuum gripper
     * @return a string containing a unique description for the gripper, so you can easily identify the gripper you are refering to.
     **/
    inline std::string get_name()
    {
        return Vacuum_Gripper::type_to_str(_gripper) + " vacuum gripper";
    }

    /**
     * This function returns the vacuum gripper controlling
     * @return a string containing a unique description for the gripper, so you can easily identify the gripper you are refering to.
     **/
    inline vacuum_gripper_type get_gripper()
    {
        return _gripper;
    }

};

}

#endif // VACUUM_GRIPPER_H
