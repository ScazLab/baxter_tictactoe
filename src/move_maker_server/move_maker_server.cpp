#include <ros/ros.h>
#include "move_maker/move_maker.h"

int main(int argc, char** argv)
{
    ROS_ASSERT_MSG(argc>=3,"[Move_Maker_Server] You need to specify the xml trajectory file and the service executing it.\nFor example: rosrun tictactoe %s ttt_moves.trajectories /sdk/robot/limb/left/follow_joint_trajectory",argv[0]);
    std::string file=argv[1];
    std::string service=argv[2];

    ros::init(argc, argv, "ttt_move_maker");
    ttt::Move_Maker baxter_moves(file.c_str(),service.c_str());
    ros::Duration(2.0).sleep();
    ros::spinOnce(); 
    ROS_INFO("[Move_Maker_Server] Moving the left arm to home position.");
    baxter_moves.make_a_move(std::vector<std::string> (1,"home"), std::vector<ttt::Trajectory_Type> (1,ttt::PLAIN));

    ros::spin();       

    return 0;
}
