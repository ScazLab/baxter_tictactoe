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
    Change INFO_STREAM to DEBUG_STREAM once testing is done
*/

ArmController::ArmController()
{
    r_joint_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
    l_joint_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);

    r_ik_client = n.serviceClient<SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");
    l_ik_client = n.serviceClient<SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");

    NUM_JOINTS = 7;
}

ArmController::~ArmController() {}

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

    vector<float> joint_angles = getJointAngles(pose_stamped, RIGHT);
    if(!joint_angles.empty()) 
    {
        publishMoveCommand(joint_angles, RIGHT);
    }
}

void ArmController::publishMoveCommand(vector<float> joint_angles, Limb limb) {
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

    while(ros::ok())
    {
        // send command to arm to move joints into joint angles specified in joint_cmd
        switch(limb)
        {
            case RIGHT:
                r_joint_cmd_pub.publish(joint_cmd);
                break;
            case LEFT:
                l_joint_cmd_pub.publish(joint_cmd);
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

vector<float> ArmController::getJointAngles(PoseStamped pose_stamped, Limb limb)
{
    // IK solver service
    SolvePositionIK ik_srv;
    // records whether IK solver service was successfully called
    bool srv_called;

    ik_srv.request.pose_stamp.push_back(pose_stamped);

    // call service and check whether service call succeeded
    switch(limb)
    {
        case RIGHT:
            srv_called = r_ik_client.call(ik_srv);
            break;
        case LEFT:
            srv_called = l_ik_client.call(ik_srv);
            break; 
    }
    
    // if service is successfully called
    if(srv_called)
    {
        ROS_INFO("[Arm_Controller] Service called");
        ROS_INFO_STREAM(std::cout << "[Arm_Controller] " << ik_srv.request << std::endl);
        ROS_INFO_STREAM(std::cout << "[Arm_Controller] " << ik_srv.response << std::endl);

        // store joint angles values received from service
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
        ROS_ERROR("[Arm_Controller] SolvePositionIK service was unsuccessful");
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
