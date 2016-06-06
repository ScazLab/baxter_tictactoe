#include "arm_controller/arm_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");

    ArmController acl("left");
    acl.moveToRest();

    for(int i = 1; i <= 9; i++)
    {
        acl.pickUpToken();
        acl.placeToken(i);       
    }
    ros::shutdown();

    ros::spin();
    return 0;
}