#include <ros/console.h>

#include "trajectory_xml_parser.h"

int main(int argc, char ** argv)
{
    ROS_INFO_STREAM("Adding a raw trajectory file  to an xmltrajectory file.");

    ROS_ASSERT_MSG(argc>3, "All joint names are required as parameters");

    std::string raw_file=argv[1];
    std::string xml_file=argv[2];
    unsigned int n_joints = argc-3; //number of joints = total arguments - 3 (executable name, raw file, and xml file)
    std::vector<std::string> joint_manes(n_joints);
    for (int i = 3; i < argc; ++i) {
        joint_manes[i-3]=argv[i];
    }

    ROS_INFO_STREAM("Reading points from a file with \"raw\" data extracted with \"rostopic echo\". 7 joints and 2.0s to reach the next point.");
    std::vector<trajectory_msgs::JointTrajectoryPoint> points = ttt::trajectory_xml_parser::read_points_from_raw_file(raw_file, 7, 2.0);
    ROS_INFO_STREAM("Trajectory with " << points.size() << " points:");
    foreach (trajectory_msgs::JointTrajectoryPoint p, points) {
        ROS_INFO_STREAM(p);
    }

    trajectory_msgs::JointTrajectory t;
    t.header.stamp=ros::Time(0);    
    t.joint_names=joint_manes;
    t.points=points;
    std::vector<trajectory_msgs::JointTrajectory> trajs;
    trajs.push_back(t);

    std::vector<std::string> traj_ids;
    traj_ids.push_back(raw_file.substr(0,raw_file.find('.'))); //we use as trajectory id the name of the file without extension

    ROS_INFO_STREAM("Writing trajectory to the xml file " << xml_file);
    if(!ttt::trajectory_xml_parser::write_to_file(trajs,xml_file,traj_ids))
    {
        ROS_ERROR("Error writing to file");
        return -1;
    }


    return 0;
}
