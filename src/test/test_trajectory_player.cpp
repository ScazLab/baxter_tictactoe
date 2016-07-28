#include <ros/ros.h>
#include "move_maker/trajectory_player.h"
#include "move_maker/trajectory_xml_parser.h"

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
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(true)
    {
        ROS_INFO("Choose the trajectory to play:");
        for (size_t i = 0; i < traj_ids.size(); ++i)
        {
            ROS_INFO_STREAM(i+1 << "." << traj_ids[i]);
        }
        ROS_INFO("Any other number to exit");
        size_t option;
        std::cin >> option;
        if (option<1 || option>traj_ids.size()) break;
        ROS_INFO("1. Plain | 2. Grasp | 3. Release");
        size_t type_traj=0;
        std::cin >> type_traj;
        if (type_traj>=1 && type_traj<=3)
        {
            switch(type_traj)
            {
            case 1:
                tp.run_trajectory(trajs[option-1]); //executing plain trajectory
                break;
            case 2:
                tp.run_trajectory_and_grasp(trajs[option-1]); //executing grasp trajectory
                break;
            case 3:
                tp.run_trajectory_and_release(trajs[option-1]); //executing release trajectory
            }
        }
        else ROS_WARN("Trajectory type not available");

    }

    ros::shutdown();
    return 0;
}
