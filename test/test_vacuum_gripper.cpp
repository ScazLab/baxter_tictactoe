#include "robot_interface/gripper.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv,"test_vacuum_gripper");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    Gripper *vg=NULL;
    ROS_INFO("Right or left vacuum gripper?[l,r]");
    char c=std::cin.get();
    std::cin.ignore();

    switch(c)
    {
    case 'l':
        vg=new Gripper("left");
        break;
    case 'r':
        vg=new Gripper("right");
        break;
    default:
        ROS_WARN("Wrong option. Run it again.");
        return 0;
    }

    while(ros::ok())
    {
        ROS_INFO("Choose the command to send to the %s", vg->getGripperLimb().c_str());
        ROS_INFO("1.Suck");
        ROS_INFO("2.Blow");
        ROS_INFO("3.Is enabled?");
        ROS_INFO("4.Is calibrated?");
        ROS_INFO("5.Is ready to grip?");
        ROS_INFO("6.Has error?");
        ROS_INFO("7.Is sucking?");
        ROS_INFO("9.Is gripping");
        ROS_INFO("0.Get name");
        ROS_INFO("E.Exit");
        c=std::cin.get();
        ROS_INFO("Option selected: %c", c);
        std::cin.ignore();

        switch(c)
        {
        case '1':
            ROS_INFO("Sucking");
            vg->gripObject();
            break;
        case '2':
            ROS_INFO("Blowing");
            vg->releaseObject();
            break;
        case '3':
            ROS_INFO("Is enabled? %s", (vg->is_enabled()? "Yes" : "No"));
            break;
        case '4':
            ROS_INFO("Is calibrated? %s", vg->is_calibrated()? "Yes" : "No");
            break;
        case '5':
            ROS_INFO("Is ready to grip? %s", vg->is_ready_to_grip()? "Yes" : "No");
            break;
        case '6':
            ROS_INFO("Has error? %s", vg->has_error()? "Yes" : "No");
            break;
        case '7':
            ROS_INFO("Is sucking? %s", vg->is_sucking()? "Yes" : "No");
            break;
        case '9':
            ROS_INFO("Is gripping? %s", vg->is_gripping()? "Yes" : "No");
            break;
        case '0':
            ROS_INFO("Gripper: %s", vg->getGripperLimb().c_str());
            break;
        case 'e':
        case 'E':
            return 0;
        }
        ros::spinOnce();
    }

    delete vg;
    vg = 0;

    return 0;
}

