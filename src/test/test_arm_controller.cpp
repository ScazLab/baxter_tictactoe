#include "arm_controller/arm_controller.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");

    ArmController * left_arm_controller = new ArmController("left");
    ArmController * right_arm_controller = new ArmController("right");

    left_arm_controller->moveToRest(left_arm_controller);

    ROS_INFO("parallel to moveToRest");

    while(left_arm_controller->_state != ArmController::REST)
    {
        ros::Duration(0.1).sleep();
    }

    // left_arm_controller->moveToRest(left_arm_controller);
    // right_arm_controller->moveToRest(right_arm_controller);

    // while(left_arm_controller->_state != ArmController::REST || right_arm_controller->_state != ArmController::REST)
    // {
    //     ros::Duration(0.1).sleep();
    // }
    
    ROS_INFO("Arms moved to rest");

    ros::shutdown();
    ros::spin();
    return 0;
}