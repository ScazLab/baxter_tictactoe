#ifndef TRAJECTORY_XML_PARSER_H
#define TRAJECTORY_XML_PARSER_H

#include <vector>
#include <qt4/Qt/QtCore>
// ROS includes
#include <ros/ros.h>
#include <ros/duration.h>
// Baxter includes
#include <control_msgs/FollowJointTrajectoryAction.h>

class trajectory_xml_parser
{
private:
    /**
        We assume that all trajectories in the file have the same joint names, and the same number of joints for all points.
        Then, we extract the joint names from the first point from the first trajectory.
    **/
    static bool read_joint_names_from_file(std::string filename, std::vector<std::string>& joint_names);

public:  
    static std::string ros_time_to_str(ros::Time t);

    static bool write_to_file(trajectory_msgs::JointTrajectory traj, std::string filename, std::string trajectory_id);

    static bool write_to_file(std::vector<trajectory_msgs::JointTrajectory> trajs, std::string filename, std::vector<std::string> trajectory_ids);

    static bool read_from_file(std::string filename, std::vector<trajectory_msgs::JointTrajectory>& trajs, std::vector<std::string>& traj_ids);

    static std::vector<trajectory_msgs::JointTrajectoryPoint> get_points_from_file(std::string filename, int n_joints, double time_gap);
};

#endif // TRAJECTORY_XML_PARSER_H
