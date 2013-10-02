#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <baxter_msgs/SolvePositionIK.h>
#include <baxter_msgs/JointCommandMode.h>


int main(int argc, char **argv)
{

    ROS_INFO("A simple inverse kinematic client commanding a position at 100Hz rate");
    ros::init(argc, argv, "inv_kin_client");
    std::string service_name = "/sdk/robot/limb/right/solve_ik_position";
    std::string topic_command_right_arm = "/robot/limb/right/command_joint_angles";
    std::string topic_mode_right_arm = "/robot/limb/right/joint_command_mode";

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<baxter_msgs::SolvePositionIK>(service_name);
    ros::Publisher commander = n.advertise<baxter_msgs::JointPositions>(topic_command_right_arm,1);
    ros::Publisher mode = n.advertise<baxter_msgs::JointCommandMode>(topic_mode_right_arm,1);
    ROS_ASSERT_MSG(commander,"Empty publisher");

    ROS_INFO("Press INTRO");
    std::cin.get();

    std_msgs::Header h;
    h.stamp=ros::Time::now();
    h.frame_id="base";

    geometry_msgs::Pose p;
    p.position.x=0.656982770038;
    p.position.y=-0.852598021641;
    p.position.z=0.0388609422173;
    p.orientation.x=0.367048116303;
    p.orientation.y=0.885911751787;
    p.orientation.z=-0.108908281936;
    p.orientation.w=0.261868353356;

    geometry_msgs::PoseStamped ps;
    ps.header=h;
    ps.pose=p;

    baxter_msgs::SolvePositionIK pik;
    pik.request.pose_stamp.push_back(ps);
    if(client.call(pik)){
        ROS_INFO_STREAM("response.joints.size=" << pik.response.joints.size());
        for (size_t i = 0; i < pik.response.joints.size(); ++i) {
            if(pik.response.isValid[i])
            {
                ROS_INFO_STREAM("Joints in response " << i);
                for (size_t var = 0; var < pik.response.joints[i].angles.size(); ++var) {
                    ROS_INFO_STREAM("Joint " << pik.response.joints[i].names[var] << ": angle " << pik.response.joints[i].angles[var]);
                }
            }
            else ROS_WARN_STREAM("Response " << i << " not valid!");
        }
    }
    else {
        ROS_ERROR_STREAM("Erros calling to the service " << service_name);
    }

    baxter_msgs::JointCommandMode m;
    m.mode= baxter_msgs::JointCommandMode::POSITION;

    unsigned int counter=0;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        mode.publish(m);
        commander.publish(pik.response.joints[0]);
        ROS_INFO_STREAM("Published " << ++counter << " times");

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
