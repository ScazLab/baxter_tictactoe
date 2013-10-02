#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <baxter_msgs/SolvePositionIK.h>
#include <baxter_msgs/JointCommandMode.h>

#include <moveit/move_group_interface/move_group.h>

#include <stdlib.h>

#include <Eigen/Geometry>

int main(int argc, char **argv)
{

    ROS_INFO("A simple demo of using MoveIt");

    ROS_DEBUG_STREAM(argc << " params in command line");
    for (int i = 0; i < argc; ++i) {
        ROS_DEBUG_STREAM("Param " << i << ": " << argv[i]);
    }

    ROS_ASSERT_MSG(argc>=9, "Requires group name, position coordinates (x,y,z), and orientation coordinates (x,y,z,w) of the robot's workspace.");
    double x=atof(argv[2]);
    double y=atof(argv[3]);
    double z=atof(argv[4]);
    double ox=atof(argv[5]);
    double oy=atof(argv[6]);
    double oz=atof(argv[7]);
    double ow=atof(argv[8]);
    ROS_INFO_STREAM("Target position to (x=" << x << " y=" << y << " z=" << z << ") (ox=" << ox << " oy=" << oy << " oz=" << oz << " ow=" << ow);

    ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);

    // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // this connecs to a running instance of the move_group node
    moveit::planning_interface::MoveGroup group(argv[1]);

    ROS_DEBUG_STREAM("Joint Tolerance=" << group.getGoalJointTolerance());
    ROS_DEBUG_STREAM("Position Tolerance=" << group.getGoalPositionTolerance() << "m");
    ROS_DEBUG_STREAM("Orientation Tolerance=" << group.getGoalOrientationTolerance() << "rad");

    // specify that our target will be a random one
    //group.setRandomTarget();
    //group.setPositionTarget(x,y,z);
    //group.setOrientationTarget(x,y,z,w);

    geometry_msgs::Pose ps;
    ps.position.x=x;
    ps.position.y=y;
    ps.position.z=z;
    ps.orientation.x=ox;
    ps.orientation.y=oy;
    ps.orientation.z=oz;
    ps.orientation.w=ow;
    ROS_INFO_STREAM("Target pose is \n" << ps);
    group.setPoseTarget(ps);


//    if(argc>=8)
//    {
//        double roll=atof(argv[5]);
//        double pitch=atof(argv[6]);
//        double yaw=atof(argv[7]);
//        ROS_INFO_STREAM("Orienting to roll=" << roll << " pitch=" << pitch << " yaw=" << yaw);
//        group.setRPYTarget(roll,pitch,yaw);
//    }
//    // plan the motion and then move the group to the sampled target
//    ROS_DEBUG_STREAM("End effector=" << group.getEndEffectorLink());
    ROS_DEBUG_STREAM("Move it!");
    group.move();

//    double aux=0;
//    ps.position.x=aux;
//    ps.position.y=0.5;
//    ps.position.z=aux;
//    ROS_INFO_STREAM("Press enter to move to \n" << ps);
//    std::cin.get();
//    group.setPoseTarget(ps);
//    group.move();

    ros::waitForShutdown();
}
