#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

int main(int argc, char **argv)
{
    ROS_ASSERT_MSG(argc>1,"Parameter required: action_server_name");
    ROS_INFO_STREAM("A client using the " << argv[1] << " action server");
    ros::Duration long_timeout (10.0);

    ros::init(argc, argv, "follow_joint_trajectory_client");
//    Client client("follow_joint_trajectory", true); // true -> don't need to call ros::spin()
    Client client(argv[1], true); // true -> don't need to call ros::spin()
    ROS_INFO("Waiting fot connecting with the server...");
    ROS_ASSERT_MSG(client.waitForServer(long_timeout),"Timeout. Server not available.");
    ROS_INFO("CONNECTED");


    unsigned int n_joints = 7;
    std::vector<std::string> joint_manes(n_joints);
    joint_manes[0]="left_s0";
    joint_manes[1]="left_s1";
    joint_manes[2]="left_e0";
    joint_manes[3]="left_e1";
    joint_manes[4]="left_w0";
    joint_manes[5]="left_w1";
    joint_manes[6]="left_w2";

    // One point trajectory
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints,0.0);
    point.velocities.resize(n_joints,0.0);
    point.accelerations.resize(n_joints,0.0);    
    trajectory_msgs::JointTrajectory traj_home;
    traj_home.joint_names=joint_manes;
//    traj_home.header.stamp = ros::Time(0); // start inmediately
    traj_home.points.resize(1,point);
    traj_home.points[0].positions[0] = 1.57961671452;
    traj_home.points[0].positions[1] = 0.277267027094;
    traj_home.points[0].positions[2] = -1.9684808438;
    traj_home.points[0].positions[3] = 2.46510712332;
    traj_home.points[0].positions[4] = 1.49601476168;
    traj_home.points[0].positions[5] = 1.97768472852;
    traj_home.points[0].positions[6] = -0.0636602026245;
    traj_home.points[0].time_from_start = ros::Duration(10.0);
    // Action goal
    control_msgs::FollowJointTrajectoryGoal goal_home;
    goal_home.trajectory = traj_home;
    ROS_INFO_STREAM("Press ENTER to start to trajectory to home");
    std::cin.get();
    goal_home.trajectory.header.stamp = ros::Time::now(); // start now
    client.sendGoal(goal_home);
    ROS_ASSERT_MSG(client.waitForResult(),"Home not reached.");
    ROS_INFO("Home reached. Ready for next trajectory");
    ROS_INFO_STREAM("action client state is " << client.getState().toString());


    // Three point trajectory
    std::vector<trajectory_msgs::JointTrajectoryPoint> points(3,point);

//    time,       left_s0,      left_s1,       left_e0,      left_e1,      left_w0,      left_w1,      left_w2,
//    0.277026,   1.57961671452,0.277267027094,-1.9684808438,2.46510712332,1.49601476168,1.97768472852,-0.0636602026245
    points[0].positions[0] = 1.57961671452;
    points[0].positions[1] = 0.277267027094;
    points[0].positions[2] = -1.9684808438;
    points[0].positions[3] = 2.46510712332;
    points[0].positions[4] = 1.49601476168;
    points[0].positions[5] = 1.97768472852;
    points[0].positions[6] = -0.0636602026245;
    points[0].time_from_start = ros::Duration(0.277026);

//    time,       left_s0,       left_s1,       left_e0,        left_e1,      left_w0,      left_w1,      left_w2,
//    7.290323,   1.02163120356, 0.145728174683,-1.95889346389, 2.37115080016,1.57156331539,1.637140994,  -0.0644271930176
    points[1].positions[0] = 1.02163120356;
    points[1].positions[1] = 0.145728174683;
    points[1].positions[2] = -1.95889346389;
    points[1].positions[3] = 2.37115080016;
    points[1].positions[4] = 1.57156331539;
    points[1].positions[5] = 1.637140994;
    points[1].positions[6] = -0.0644271930176;
//    points[1].time_from_start = ros::Duration(7.290323);
    points[1].time_from_start = ros::Duration(4);

//    time,       left_s0,          left_s1,       left_e0,        left_e1,      left_w0,      left_w1,      left_w2,
//    13.590337,  -0.0951068087402, 0.107762150226,-2.06435464294, 1.504451656,  2.0129662866, 1.65708274422,-0.0636602026245
    points[2].positions[0] = -0.0951068087402;
    points[2].positions[1] = 0.107762150226;
    points[2].positions[2] = -2.06435464294;
    points[2].positions[3] = 1.504451656;
    points[2].positions[4] = 2.0129662866;
    points[2].positions[5] = 1.65708274422;
    points[2].positions[6] = -0.0636602026245;
//    points[2].time_from_start = ros::Duration(13.590337);
    points[2].time_from_start = ros::Duration(9);

    trajectory_msgs::JointTrajectory traj;
    traj.joint_names=joint_manes;
//    traj.header.stamp = ros::Time(0); // start inmediately
    traj.points=points;
    // Action goals
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj;
    ROS_INFO_STREAM("Press ENTER to start a new trajectory");
    std::cin.get();
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0); // start 1s after now
    client.sendGoal(goal);
    ROS_ASSERT_MSG(client.waitForResult(),"Goal not reached.");
    ROS_INFO("Goal reached!");
    ROS_INFO_STREAM("action client state is " << client.getState().toString());

    ros::shutdown();

}
