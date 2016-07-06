#include <ros/ros.h>
#include <ros/console.h>
#include "arm_controller/arm_controller.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "thread");

    ArmController * left_ac = new ArmController("left");
    ArmController * right_ac = new ArmController("right");

    // left_ac->moveToRest();
    // right_ac->moveToRest();
    // while(!(left_ac->getState() == REST && right_ac->getState() == REST)) {ros::spinOnce();}


    left_ac->scanBoard();
    while(left_ac->getState() != SCAN){ros::spinOnce();}

    // for(int i = 0; i < 9; i++)
    // {
    //     left_ac->pickUpToken();
    //     while(left_ac->getState() != PICK_UP){ros::spinOnce();}

    //     left_ac->putDownToken(1);
    //     while(left_ac->getState() != PUT_DOWN){ros::spinOnce();}        
    // }

    for(int i = 0; i < 9; i++)
    {
        left_ac->pickUpToken();
        while(left_ac->getState() != PICK_UP){ros::spinOnce();}

        left_ac->putDownToken(i + 1);
        while(left_ac->getState() != PUT_DOWN){ros::spinOnce();}       
    }
  
    // left_ac->moveToRest();
    // while(!(left_ac->getState() == REST)) {ros::spinOnce();}

    ros::shutdown();
    return 0;
}

