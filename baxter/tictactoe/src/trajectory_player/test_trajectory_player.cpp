#include <ros/ros.h>
#include "trajectory_player.h"
#include "trajectory_xml_parser.h"

int main(int argc, char** argv)
{
    ROS_ASSERT_MSG(argc>=3,"You need to specify the xml trajectory file and the service executing it.\nFor example: rosrun tictactoe %s ttt_moves.trajectories /sdk/robot/limb/left/follow_joint_trajectory",argv[0]);
    std::string file=argv[1];
    std::string service=argv[2];

    std::vector<trajectory_msgs::JointTrajectory> trajs;
    std::vector<std::string> traj_ids;
    ROS_INFO_STREAM("Reading trajectories from file " << file);
    if(!ttt::trajectory_xml_parser::read_from_file(file,trajs,traj_ids))
    {
        ROS_ERROR("Error reading from file");
        return -1;
    }
    ROS_ASSERT_MSG(trajs.size()==traj_ids.size(),"#trajs != #traj_ids");
    ROS_INFO_STREAM(trajs.size() << " trajectories read");


    ros::init(argc, argv, "test_trajectory_player");
    ttt::Trajectory_Player tp(service.c_str());

    while(true)
    {
        ROS_INFO("Choose the trajectory to play:");
        for (size_t i = 0; i < traj_ids.size(); ++i) {
            ROS_INFO_STREAM(i+1 << "." << traj_ids[i]);
        }
        size_t option;
        std::cin >> option;
        if (option<1 || option>traj_ids.size()) break;
        tp.run_trajectory_and_grasp(trajs[option-1]); //executing plain trajectory
    }

    ros::shutdown();
    return 0;
}
