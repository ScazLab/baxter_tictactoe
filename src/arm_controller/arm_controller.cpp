// correct header to use once arm_controller is migrated to /lib after testing
// #include "arm_controller/arm_controller.h"

#include "arm_controller.h"

using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std;
using namespace ttt;

/*
    TASKS:
    Make arm move at same time
    I would make  ArmController::hoverAboveTokens() return bool if success/failure. 
    And also you should check for the limb, because if by mistake you call the function 
    on the right limb you should check for that in the beginning and return false. 
*/

/******************* Public ************************/

ArmController::ArmController(string limb): img_trp(n), limb(limb)
{

    if(limb != "left" && limb != "right"){
        ROS_ERROR("[Arm Controller] Invalid limb. Acceptable values are: right / left");
        ros::shutdown();
    }

    ROS_DEBUG_STREAM(cout << "[Arm Controller] " << limb << endl);

    // see header file for descriptions
    joint_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + limb + "/joint_command", 1);   
    endpt_sub = n.subscribe("/robot/limb/" + limb + "/endpoint_state", 1, &ArmController::endpointCallback, this);
    img_sub = img_trp.subscribe("/cameras/left_hand_camera/image", 1, &ArmController::imageCallback, this);

    ir_sub = n.subscribe("/robot/range/left_hand_range/state", 1, &ArmController::IRCallback, this);

    ik_client = n.serviceClient<SolvePositionIK>("/ExternalTools/" + limb + "/PositionKinematicsNode/IKService");
    gripper = new Vacuum_Gripper(ttt::left);

    cv::namedWindow("[Arm Controller] hand camera", cv::WINDOW_NORMAL);

    NUM_JOINTS = 7;
    IR_RANGE_THRESHOLD = 0.085;
    curr_range = 0;
    curr_max_range = 0;
    curr_min_range = 0;
}

ArmController::~ArmController() {
    cv::destroyWindow("[Arm Controller] hand camera");
}

void ArmController::endpointCallback(const EndpointState& msg)
{
    curr_pose = msg.pose;
}

void ArmController::imageCallback(const ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("[Arm Controller] cv_bridge exception: %s", e.what());
    }

    cv::imshow("[Arm Controller] hand camera", cv_ptr->image.clone());
    cv::waitKey(30);
}

void ArmController::IRCallback(const RangeConstPtr& msg)
{


    ROS_DEBUG_STREAM(cout << "range: " << msg->range << " max range: " << msg->max_range << " min range: " << msg->min_range << endl);
    ROS_DEBUG_STREAM(cout << "range: " << curr_range << " max range: " << curr_max_range << " min range: " << curr_min_range << endl);
    curr_range = msg->range;
    curr_max_range = msg->max_range;
    curr_min_range = msg->min_range;
}

void ArmController::pickUpToken()
{
    if(limb == "right") 
    {
        ROS_ERROR("[Arm Controller] Right arm should not pick up tokens");
    }
    else if(limb == "left")
    {
        hoverAboveTokens();
        gripToken();
        hoverAboveTokens();        
    }
}

/*

min_range: 0.00400000018999
max_range: 0.40000000596
range: 0.0590000003576

*/

// Moving from Untucked to Rest using IK solver works 1 out of 10~15 times
// Moving from Enabled to Rest using Hardcoded Joint Angle also doesn't work; gets stuck in awkward pose
void ArmController::moveToRest() 
{
    vector<float> joint_angles;
    joint_angles.resize(NUM_JOINTS);

    // requested position and orientation is filled out despite hardcoding of joint angles
    // to double-check if move has been completed using hasPoseCompleted()
    req_pose_stamped.header.frame_id = "base";
    req_pose_stamped.pose.position.x = 0.292391;
    req_pose_stamped.pose.position.z = 0.181133;
    req_pose_stamped.pose.orientation.x = 0.028927;
    req_pose_stamped.pose.orientation.y = 0.686745;
    req_pose_stamped.pose.orientation.z = 0.00352694;
    req_pose_stamped.pose.orientation.w = 0.726314;

    // joint angles are hardcoded as opposed to relying on the IK solver to provide a solution given
    // the requested pose because testing showed that the IK solver would fail approx. 9/10 to find 
    // a joint angles combination for the left arm rest pose.
    if(limb == "left")
    {
        joint_angles[0] = 1.1508690861110316;
        joint_angles[1] = -0.6001699832601681;
        joint_angles[2] = -0.17449031462196582;
        joint_angles[3] = 2.2856313739492666;
        joint_angles[4] = 1.8680051044474626;
        joint_angles[5] = -1.4684031092033123;
        joint_angles[6] = 0.1257864246066039;
        req_pose_stamped.pose.position.y = 0.611039; 
    }
    else if(limb == "right") 
    {
        joint_angles[0] = -1.3322623142784817;
        joint_angles[1] = -0.5786942522297723;
        joint_angles[2] = 0.14266021327334347;
        joint_angles[3] = 2.2695245756764697;
        joint_angles[4] = -1.9945585194480093;
        joint_angles[5] = -1.469170099597255;
        joint_angles[6] = -0.011504855909140603;
        req_pose_stamped.pose.position.y = -0.611039; 
    }

    // vector<float> joint_angles = getJointAngles(req_pose_stamped);
    publishMoveCommand(joint_angles, POSE);
}

/******************* Private ************************/

void ArmController::hoverAboveTokens()
{

    req_pose_stamped.header.frame_id = "base";
    req_pose_stamped.pose.position.x = 0.540298787334;
    req_pose_stamped.pose.position.y = 0.603732369738;

    req_pose_stamped.pose.position.z = 0.13621169853;
    // req_pose_stamped.pose.position.z = -0.13021169853;
    // -0.13... ~= 3 tokens
    // -0.15... ~= board
    req_pose_stamped.pose.orientation.x = 0.712801568376;
    req_pose_stamped.pose.orientation.y = -0.700942136419;
    req_pose_stamped.pose.orientation.z = -0.0127158080742;
    req_pose_stamped.pose.orientation.w = -0.0207931175453;

    vector<float> joint_angles = getJointAngles(req_pose_stamped);
    publishMoveCommand(joint_angles, POSE);

    /* joint angles: left_e0: -2.4160197409195265, left_e1: 0.9921020745648913, left_s0: -0.0947233136519243, left_s1: 0.4571262747898533, left_w0: 2.5272333480412192, left_w1: 1.8699225804323194, left_w2: -1.5044516577186196 */
}

void ArmController::gripToken()
{
    req_pose_stamped.header.frame_id = "base";
    req_pose_stamped.pose.position.x = 0.540298787334;
    req_pose_stamped.pose.position.y = 0.603732369738;

    req_pose_stamped.pose.position.z = -0.10501169853;
    // -0.13... ~= 3 tokens
    // -0.15... ~= board
    req_pose_stamped.pose.orientation.x = 0.712801568376;
    req_pose_stamped.pose.orientation.y = -0.700942136419;
    req_pose_stamped.pose.orientation.z = -0.0127158080742;
    req_pose_stamped.pose.orientation.w = -0.0207931175453;

    // ROS_ERROR("Entered grip token");
    vector<float> joint_angles = getJointAngles(req_pose_stamped);
    publishMoveCommand(joint_angles, COLLISION);
}


// error-check if IK solver gives all zeros?
vector<float> ArmController::getJointAngles(PoseStamped pose_stamped)
{
    // IK solver service
    SolvePositionIK ik_srv;

    ik_srv.request.pose_stamp.push_back(pose_stamped);
    ik_client.call(ik_srv);

    ROS_DEBUG_STREAM(cout << "[Arm Controller] " << ik_srv.request << endl);
    ROS_DEBUG_STREAM(cout << "[Arm Controller] " << ik_srv.response << endl);   
    

    // if service is successfully called
    if(ik_client.call(ik_srv))
    {
        ROS_DEBUG("[Arm Controller] Service called");

        // store joint angles values received from service
        vector<float> joint_angles;
        joint_angles.resize(NUM_JOINTS);
        for(int i = 0; i < ik_srv.response.joints[0].position.size(); i++)
        {
            joint_angles[i] = ik_srv.response.joints[0].position[i];
        }

        for(int i = 0; i < joint_angles.size(); i++){
            ROS_DEBUG("[Arm Controller] Joint angles %d: %0.4f", i, joint_angles[i]);
        }
        ros::Duration(2).sleep(); 
        return joint_angles;
    }
    else 
    {
        ROS_ERROR("[Arm Controller] SolvePositionIK service was unsuccessful");
        vector<float> empty;
        return empty;
    }
}

void ArmController::publishMoveCommand(vector<float> joint_angles, goalType goal) 
{
    ros::Rate loop_rate(100);
    JointCommand joint_cmd;

    // command in velocity mode
    joint_cmd.mode = JointCommand::POSITION_MODE;

    // command joints in the order shown in baxter_interface
    joint_cmd.names.push_back(limb + "_s0");
    joint_cmd.names.push_back(limb + "_s1");
    joint_cmd.names.push_back(limb + "_e0");
    joint_cmd.names.push_back(limb + "_e1");
    joint_cmd.names.push_back(limb + "_w0");
    joint_cmd.names.push_back(limb + "_w1");
    joint_cmd.names.push_back(limb + "_w2");

    // set your calculated velocities
    joint_cmd.command.resize(NUM_JOINTS);

    // set joint angles
    for(int i = 0; i < NUM_JOINTS; i++)
    {
        joint_cmd.command[i] = joint_angles[i];
    }

    while(ros::ok())
    {
        // ROS_DEBUG("[Arm Controller] In the loop");
        joint_cmd_pub.publish(joint_cmd);
        ros::spinOnce();
        loop_rate.sleep();

        if(goal == POSE)
        {
            if(hasPoseCompleted()) 
            {
                ROS_DEBUG("[Arm Controller] Move completed");
                break;
            }              
        }
        else if(goal == COLLISION)
        {
            if(hasCollided())
            {
                gripper->suck();
                break;
            }
            else {
                ROS_DEBUG("No collision yet");
            }
        }
    }   
}

bool ArmController::hasPoseCompleted()
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

bool ArmController::hasCollided()
{
    ROS_DEBUG_STREAM(cout << " range: " << curr_range << " max range: " << curr_max_range << " min range: " << curr_min_range << endl);
    if(curr_range <= curr_max_range && curr_range >= curr_min_range && curr_range <= IR_RANGE_THRESHOLD)
    {
        ROS_INFO("[Arm Controller] Collision");
        return true;
    }
    else {
        return false;
    }
}

bool ArmController::equalTwoDP(float x, float y) 
{
    // ROS_DEBUG_STREAM(cout << "[Arm Controller] curr_pose: " << x << "req_pose_stamped: " << y << endl);
    float xTwoDP = roundf(x * 100) / 100;
    float yTwoDP = roundf(y * 100) / 100;
    return xTwoDP == yTwoDP ? true : false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ArmController acl("left");
    acl.moveToRest();
    acl.pickUpToken();

    ros::spin();
    return 0;
}

/*

[DEBUG] [1464888840.333485258]: 0x6575c8[Arm Controller] joints[]
  joints[0]: 
    header: 
      seq: 0
      stamp: 0.000000000
      frame_id: 
    name[]
    position[]
    velocity[]
    effort[]
isValid[]
  isValid[0]: 0
result_type[]
  result_type[0]: 0

[DEBUG] [1464888843.480391118]: 0x6575c8[Arm Controller] joints[]
  joints[0]: 
    header: 
      seq: 0
      stamp: 0.000000000
      frame_id: 
    name[]
    position[]
    velocity[]
    effort[]
isValid[]
  isValid[0]: 0
result_type[]
  result_type[0]: 0
*/