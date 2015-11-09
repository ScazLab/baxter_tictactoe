#include "trajectory_xml_parser.h"

int main(int argc, char ** argv)
{
    ROS_INFO_STREAM("Concatenating several xml trajectory files into just one");
    ROS_ASSERT_MSG(argc>3, "concat_trajectory_xml_files [dest_file] [orig_file1] [orig_file2] ... [orig_fileN]");

    std::string dest_file = argv[1];

    std::vector<trajectory_msgs::JointTrajectory> trajs;
    std::vector<std::string> traj_ids;
    for (int i = 2; i < argc; ++i) {
        std::string orig_file=argv[i];
        ROS_INFO_STREAM("Reading trajectories from file " << orig_file);

        std::vector<trajectory_msgs::JointTrajectory> trajs_file;
        std::vector<std::string> traj_ids_file;
        if(!ttt::trajectory_xml_parser::read_from_file(orig_file,trajs_file,traj_ids_file))
        {
            ROS_ERROR("Error reading from file");
            return -1;
        }
        ROS_ASSERT_MSG(trajs_file.size()==traj_ids_file.size(),"#trajs != #traj_ids");
        ROS_INFO_STREAM(trajs_file.size() << " trajectories read");

        trajs.insert(trajs.end(),trajs_file.begin(),trajs_file.end());
        traj_ids.insert(traj_ids.end(),traj_ids_file.begin(),traj_ids_file.end());

        trajs_file.clear();
        traj_ids_file.clear();
    }

    ROS_INFO_STREAM("Writing " << trajs.size() << " trajectories to file " << dest_file);
    if(!ttt::trajectory_xml_parser::write_to_file(trajs,dest_file,traj_ids))
    {
        ROS_ERROR("Error writing to file");
        return -1;
    }

    return 0;
}
