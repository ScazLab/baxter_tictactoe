#include "trajectory_xml_parser/trajectory_xml_parser.h"

int main(int argc, char ** argv)
{
    ROS_INFO_STREAM("Testing xml parser for loading trajectories from a xml file");
    ROS_ASSERT_MSG(argc>=2, "Trajectory xml file name required as argument");

    std::vector<trajectory_msgs::JointTrajectory> trajs;
    std::vector<std::string> traj_ids;
    std::string file=argv[1];
    ROS_INFO_STREAM("Reading trajectories from file " << file);
    if(!ttt::trajectory_xml_parser::read_from_file(file,trajs,traj_ids))
    {
        ROS_ERROR("Error reading from file");
        return -1;
    }
    ROS_ASSERT_MSG(trajs.size()==traj_ids.size(),"#trajs != #traj_ids");
    ROS_INFO_STREAM(trajs.size() << " trajectories read");

    for (size_t i = 0; i < traj_ids.size(); ++i) {
        ROS_INFO_STREAM(i+1 << "." << traj_ids[i] << std::endl << trajs[i]);
    }

    return 0;
}
