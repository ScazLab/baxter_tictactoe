#ifndef VACUUM_GRIPPER_H
#define VACUUM_GRIPPER_H

#include <ros/ros.h>

namespace ttt
{

typedef enum {left, right} vacuum_gripper_type;

class Vacuum_Gripper
{
private:
    vacuum_gripper_type _gripper;
    ros::NodeHandle _nh; //! ROS node handle
    ros::Subscriber _sub_state; //! subscriber to receive the messages related to the state of the vacuum gripper
    ros::Publisher _pub_command; //! publisher to command the vacuum gripper: suck or blow.

public:
    /**
     * Constructor of the class
     * \param gripper This indicates if we are using the left or right vacuum gripper. Possible values are just "right" or "left". Other values cause an assertion.
     **/
    Vacuum_Gripper(std::string gripper);
};

}

#endif // VACUUM_GRIPPER_H
