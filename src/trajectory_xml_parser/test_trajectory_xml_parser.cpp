#include "trajectory_xml_parser.h"

int main(int argc, char ** argv)
{
    ROS_INFO_STREAM("Testing xml parser for trajectories");

    ROS_INFO_STREAM("1. Reading points from a file with \"raw\" data extracted with \"rostopic echo\". 7 joints and 2.0s to reach the next point.");
    std::vector<trajectory_msgs::JointTrajectoryPoint> points = ttt::trajectory_xml_parser::read_points_from_raw_file("../resources/test/raw.positions", 7, 2.0);
    ROS_INFO_STREAM("Trajectory with " << points.size() << " points:");
    foreach (trajectory_msgs::JointTrajectoryPoint p, points) {
        ROS_INFO_STREAM(p);
    }

    trajectory_msgs::JointTrajectory t;
    t.header.stamp=ros::Time(0);
    unsigned int n_joints = 7;
    std::vector<std::string> joint_manes(n_joints);
    joint_manes[0]="left_e0";
    joint_manes[1]="left_e1";
    joint_manes[2]="left_s0";
    joint_manes[3]="left_s1";
    joint_manes[4]="left_w0";
    joint_manes[5]="left_w1";
    joint_manes[6]="left_w2";
    t.joint_names=joint_manes;
    t.points=points;
    std::vector<trajectory_msgs::JointTrajectory> trajs;
    trajs.push_back(t);

    std::vector<std::string> traj_ids;
    traj_ids.push_back("grasp");

    std::string file="example.trajectories";
    ROS_INFO_STREAM("2. Writing trajectory to file " << file);
    if(!ttt::trajectory_xml_parser::write_to_file(trajs,file,traj_ids))
    {
        ROS_ERROR("Error writing to file");
        return -1;
    }

    trajs.clear();
    traj_ids.clear();
    ROS_INFO_STREAM("3. Reading trajectories from file " << file);
    if(!ttt::trajectory_xml_parser::read_from_file(file,trajs,traj_ids))
    {
        ROS_ERROR("Error reading from file");
        return -1;
    }
    ROS_INFO_STREAM(trajs.size() << " trajectories read");
    foreach (std::string t_id, traj_ids) {
        ROS_INFO_STREAM("Trajectory " << t_id);
    }

    return 0;
}
