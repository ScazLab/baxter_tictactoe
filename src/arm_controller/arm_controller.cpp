// correct header to use once arm_controller is migrated to /lib after testing
// #include "arm_controller/arm_controller.h"

#include "arm_controller.h"

using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace std;

/*
    TASKS:
    Use endpointState topic approximation to shutdown once destination has been reached
    Change INFO_STREAM to DEBUG_STREAM once testing is done
*/

/******************* Public ************************/

ArmController::ArmController(string limb): limb(limb)
{
    joint_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + limb + "/joint_command", 1);
    endpt_sub = n.subscribe("/robot/limb/" + limb + "/endpoint_state", 1, &ArmController::endpointCallback, this);
    ik_client = n.serviceClient<SolvePositionIK>("/ExternalTools/" + limb + "/PositionKinematicsNode/IKService");

    NUM_JOINTS = 7;
}

ArmController::~ArmController() {}

void ArmController::endpointCallback(const EndpointState& msg)
{
    curr_pose = msg.pose;
}

void ArmController::moveRightToRest() 
{
    // sets reference frame of IK Solver to Baxter's reference frame 
    req_pose_stamped.header.frame_id = "base";

    req_pose_stamped.pose.position.x = 0.473749561242;
    req_pose_stamped.pose.position.y = -0.690844692814;
    req_pose_stamped.pose.position.z = 0.408007113257;

    req_pose_stamped.pose.orientation.x = -0.0574340933229;
    req_pose_stamped.pose.orientation.y = 0.721856881714;
    req_pose_stamped.pose.orientation.z = 0.14480378245;
    req_pose_stamped.pose.orientation.w = 0.674281715483;

    vector<float> joint_angles = getJointAngles(req_pose_stamped);
    if(!joint_angles.empty()) 
    {
        publishMoveCommand(joint_angles);
    }
}

/******************* Private ************************/

vector<float> ArmController::getJointAngles(PoseStamped req_pose_stamped)
{
    // IK solver service
    SolvePositionIK ik_srv;

    ik_srv.request.pose_stamp.push_back(req_pose_stamped);
    ik_client.call(ik_srv);

    ROS_INFO_STREAM(cout << "[Arm_Controller] " << ik_srv.request << endl);
    ROS_INFO_STREAM(cout << "[Arm_Controller] " << ik_srv.response << endl);

    // if service is successfully called
    if(ik_client.call(ik_srv))
    {
        ROS_INFO("[Arm_Controller] Service called");

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

void ArmController::publishMoveCommand(vector<float> joint_angles) 
{
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

    while(ros::ok())
    {
        joint_cmd_pub.publish(joint_cmd);
        ros::spinOnce();
        loop_rate.sleep();

        if(hasMoveCompleted()) break;
    }   
}

bool ArmController::hasMoveCompleted()
{
    bool samePose = true;

    if(!equalTwoDP(curr_pose.position.x, req_pose_stamped.pose.position.x)) samePose = false; 
    if(!equalTwoDP(curr_pose.position.y, req_pose_stamped.pose.position.y)) samePose = false;
    if(!equalTwoDP(curr_pose.position.z, req_pose_stamped.pose.position.z)) samePose = false;
    if(!equalTwoDP(curr_pose.orientation.x, req_pose_stamped.pose.orientation.x)) samePose = false;
    if(!equalTwoDP(curr_pose.orientation.y, req_pose_stamped.pose.orientation.y)) samePose = false;
    if(!equalTwoDP(curr_pose.orientation.z, req_pose_stamped.pose.orientation.z)) samePose = false;
    if(!equalTwoDP(curr_pose.orientation.w, req_pose_stamped.pose.orientation.w)) samePose = false;

    return samePose;    
}

bool ArmController::equalTwoDP(float x, float y) 
{
    float xTwoDP = roundf(x * 100) / 100;
    float yTwoDP = roundf(y * 100) / 100;
    return xTwoDP == yTwoDP ? true : false;
    ROS_INFO_STREAM(cout << xTwoDP << " " << yTwoDP << endl);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "right_arm_joint_test_pub");
    ArmController ac("right");
    ac.moveRightToRest(); 
    return 0;
}
