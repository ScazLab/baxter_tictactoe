// correct header to use once arm_controller is migrated to /lib after testing
// #include "arm_controller/arm_controller.h"

#include "arm_controller.h"

using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace std;


/*

    TASKS:
    Use endpointState topic approximation to shutdown once destination has been reached
    Adapt to right and left commands

*/


ArmController::ArmController()
{
    joint_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
    ik_client = n.serviceClient<SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");

    NUM_JOINTS = 7;
}

ArmController::~ArmController()
{

}

void ArmController::moveRightToRest() 
{
    // PoseStamped message to be used as request value for IK solver service
    PoseStamped pose_stamped;
    // sets reference frame of IK Solver to Baxter's reference frame 
    pose_stamped.header.frame_id = "base";

    pose_stamped.pose.position.x = 0.473749561242;
    pose_stamped.pose.position.y = -0.690844692814;
    pose_stamped.pose.position.z = 0.408007113257;

    pose_stamped.pose.orientation.x = -0.0574340933229;
    pose_stamped.pose.orientation.y = 0.721856881714;
    pose_stamped.pose.orientation.z = 0.14480378245;
    pose_stamped.pose.orientation.w = 0.674281715483;

    vector<float> joint_angles = getJointAngles(pose_stamped);
    if(!joint_angles.empty()) 
    {
        publishMoveCommand(joint_angles);
    }
}

void ArmController::publishMoveCommand(vector<float> joint_angles) {
    ros::Rate loop_rate(100);
    JointCommand joint_cmd;

    // command in velocity mode
    joint_cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

    // command joints in the order shown in baxter_interface
    joint_cmd.names.push_back("right_s0");
    joint_cmd.names.push_back("right_s1");
    joint_cmd.names.push_back("right_e0");
    joint_cmd.names.push_back("right_e1");
    joint_cmd.names.push_back("right_w0");
    joint_cmd.names.push_back("right_w1");
    joint_cmd.names.push_back("right_w2");

    // set your calculated velocities
    joint_cmd.command.resize(NUM_JOINTS);

    // set joint angles
    for(int i = 0; i < NUM_JOINTS; i++)
    {
        joint_cmd.command[i] = joint_angles[i];
    }

    // move to untucked position (can also be achieved by running 'rosrun baxter_tools tuck_arms.py -u')

    for(int i = 0; i < NUM_JOINTS; i++)
    {
        joint_cmd.command[i] = joint_angles[i];
    }

    ROS_INFO_STREAM(std::cout<<joint_cmd<<std::endl);

    while(ros::ok())
    {
        ROS_INFO("In the loop");
        // send command to arm to move joints into joint angles specified in joint_cmd
        joint_cmd_pub.publish(joint_cmd);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_STREAM(std::cout<<joint_cmd<<std::endl);
    }
}

vector<float> ArmController::getJointAngles(PoseStamped pose_stamped)
{
    // IK solver service
    SolvePositionIK ik_srv;

    ik_srv.request.pose_stamp.push_back(pose_stamped);
    
    // if service is successfully called
    if(ik_client.call(ik_srv))
    {
        ROS_INFO("Service called");
        ROS_INFO_STREAM(std::cout << ik_srv.request << std::endl);
        ROS_INFO_STREAM(std::cout << ik_srv.response << std::endl);

        vector<float> joint_angles;
        joint_angles.resize(NUM_JOINTS);
        for(int i = 0; i < ik_srv.response.joints[0].position.size(); i++)
        {
            joint_angles[i] = ik_srv.response.joints[0].position[i];
        }
        return joint_angles;
    }
    else 
    {
        ROS_ERROR("SolvePositionIK service was unsuccessful");
        vector<float> empty;
        return empty;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "right_arm_joint_test_pub");
    ArmController ac;
    ac.moveRightToRest(); 
    return 0;
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "right_arm_joint_test_pub");
//     ros::NodeHandle n;
//     ros::Publisher right_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
//     ros::ServiceClient ik_client;

//     // ik_client = n.serviceClient<SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");

//     // SolvePositionIK srv;
//     // PoseStamped poseSt;

//     // poseSt.header.frame_id = "base";
//     // poseSt.pose.position.x = 0.473749561242;
//     // poseSt.pose.position.y = -0.690844692814;
//     // poseSt.pose.position.z = 0.408007113257;

//     // poseSt.pose.orientation.x = -0.0574340933229;
//     // poseSt.pose.orientation.y = 0.721856881714;
//     // poseSt.pose.orientation.z = 0.14480378245;
//     // poseSt.pose.orientation.w = 0.674281715483;

    // srv.request.pose_stamp.push_back(poseSt);
    // srv.request.seed_mode = 0;

    // std::vector<float> joint_angles;

    // if(ik_client.call(srv))
    // {
    //     ROS_INFO("Service called");
    //     std::cout << srv.request << std::endl;
    //     std::cout << srv.response << std::endl;
    //     joint_angles.resize(NUM_JOINTS);
    //     for(int i = 0; i < srv.response.joints[0].position.size(); i++)
    //     {
    //         joint_angles[i] = srv.response.joints[0].position[i];
    //     }
    // }

//     // if(joint_angles.size() != NUM_JOINTS)
//     // {
//     //     ROS_ERROR("Service did not return the same number of joint angles as there are joints");
//     //     return -1;
//     // }

//     // for(int i = 0; i < joint_angles.size(); i++)
//     // {
//     //     ROS_INFO("joint_angles[%d]: %0.6f", i, joint_angles[i]);
//     // }  

    // publish at at least 5 Hz, or else Baxter switches back to Position mode and holds position
//     ros::Rate loop_rate(100);
//     baxter_core_msgs::JointCommand cmd;

//     // command in velocity mode
//     cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

//     // command joints in the order shown in baxter_interface
//     cmd.names.push_back("right_s0");
//     cmd.names.push_back("right_s1");
//     cmd.names.push_back("right_e0");
//     cmd.names.push_back("right_e1");
//     cmd.names.push_back("right_w0");
//     cmd.names.push_back("right_w1");
//     cmd.names.push_back("right_w2");

//     // set your calculated velocities
//     cmd.command.resize(NUM_JOINTS);

//     // move to untucked position (can also be achieved by running 'rosrun baxter_tools tuck_arms.py -u')

//     for(int i = 0; i < NUM_JOINTS; i++)
//     {
//         cmd.command[i] = joint_angles[i];
//     }

//     // move to untucked position (can also be achieved by running 'rosrun baxter_tools tuck_arms.py -u')

//     for(int i = 0; i < NUM_JOINTS; i++)
//     {
//         cmd.command[i] = joint_angles[i];
//     }

//     std::cout<<cmd<<std::endl;

//     while(ros::ok())
//     {
//         ROS_INFO("In the loop");
//         //update cmd.command commands here
//         right_cmd_pub.publish(cmd);
//         ros::spinOnce();
//         loop_rate.sleep();
//         std::cout<<cmd<<std::endl;
//     }

//     return 0;
// }