#include "arm_controller/arm_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");

    ArmController acl("left");
    // acl.moveToRest();
    acl.pickUpToken();

    // ros::shutdown();

    ros::spin();
    return 0;
}