#include "robot_state.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv,"test_robot_state");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ttt::Robot_State rs;
    char c;
    while(ros::ok())
    {
        ROS_INFO("Choose one option");
        ROS_INFO("1.Is enabled?");
        ROS_INFO("2.Is disabled?");
        ROS_INFO("3.Is stopped?");
        ROS_INFO("4.Has error?");
        ROS_INFO("5.Enabling");
        ROS_INFO("6.Disabling");
        ROS_INFO("7.Reseting");
        ROS_INFO("E.Exit");
        c=std::cin.get();
        ROS_INFO_STREAM("Option selected: " << c);
        std::cin.ignore();
        switch(c)
        {
        case '1':
            ROS_INFO("Is enabled? %s", rs.is_enabled()? "Yes" : "No");
            break;
        case '2':
            ROS_INFO("Is disabled? %s", rs.is_disabled()? "Yes" : "No");
            break;
        case '3':
            ROS_INFO("Is stopped? %s", rs.is_stopped()? "Yes" : "No");
            break;
        case '4':
            ROS_INFO("Has error? %s", rs.has_error()? "Yes" : "No");
            break;
        case '5':
            ROS_INFO("Enabling... %s", rs.enable()? "Yes" : "No");
            break;
        case '6':
            ROS_INFO("Disabling... %s", rs.disable()? "Yes" : "No");
            break;
        case '7':
            ROS_INFO("Reseting... %s", rs.reset()? "Yes" : "No");
            break;
        case 'e':
        case 'E':
            return 0;
        }
    }

}
