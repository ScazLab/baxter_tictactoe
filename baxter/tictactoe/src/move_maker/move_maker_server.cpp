#include <ros/ros.h>
#include "move_maker.h"

int main(int argc, char** argv)
{
    ROS_ASSERT_MSG(argc>=3,"You need to specify the xml trajectory file and the service executing it.\nFor example: rosrun tictactoe move_maker ttt_moves.trajectories /sdk/robot/limb/left/follow_joint_trajectory");
    std::string file=argv[1];
    std::string service=argv[2];

    ros::init(argc, argv, "ttt_move_maker");
    ttt::Move_Maker baxter_moves(file.c_str(),service.c_str());
    ros::spin();

    return 0;
}
