#include "arm_controller/arm_controller.h"

using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std;
using namespace ttt;
using namespace cv;

/*
    TASKS:
    Make arm move at same time

    Fix image callback crash bug???
    Introduce offset variable (center of camera != center of gripper)
    Token is partially covered/excluded when camera is close
    Calibrate offset multiplier depending on depth
    Threshold out black boundaires of board and remove tokens on board
*/

/**************************** PUBLIC ******************************/

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

    namedWindow("[Arm Controller] raw image", WINDOW_NORMAL);
    namedWindow("[Arm Controller] rough processed image", WINDOW_NORMAL);
    namedWindow("[Arm Controller] processed image", WINDOW_NORMAL);

    NUM_JOINTS = 7;
    OFFSET_CONSTANT = 0.1;
    CENTER_X = 0.655298787334;
    CENTER_Y = 0.205732369738; 
    CELL_SIDE = 0.15;
    IR_RANGE_THRESHOLD = 0.065;

    _curr_x_offset = 0;
    _curr_y_offset = 0;
    curr_range = 0;
    curr_max_range = 0;
    curr_min_range = 0;
}

ArmController::~ArmController() {
    namedWindow("[Arm Controller] raw image");
    namedWindow("[Arm Controller] rough processed image");
    namedWindow("[Arm Controller] processed image");
}

/*************************Callback Functions************************/

void ArmController::endpointCallback(const EndpointState& msg)
{
    // update current pose
    curr_pose = msg.pose;
}

void ArmController::IRCallback(const RangeConstPtr& msg)
{
    // ROS_DEBUG_STREAM(cout << "range: " << msg->range << " max range: " << msg->max_range << " min range: " << msg->min_range << endl);
    // ROS_DEBUG_STREAM(cout << "range: " << curr_range << " max range: " << curr_max_range << " min range: " << curr_min_range << endl);

    // update current range
    curr_range = msg->range;
    curr_max_range = msg->max_range;
    curr_min_range = msg->min_range;
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

    Mat img_hsv;
    Mat img_hsv_blue;
    Mat img_token_rough = Mat::zeros(cv_ptr->image.size(), CV_8UC1);
    Mat img_token = Mat::zeros(cv_ptr->image.size(), CV_8UC1);

    // removes non-blue elements of image 
    cvtColor(cv_ptr->image.clone(), img_hsv, CV_BGR2HSV);
    inRange(img_hsv, Scalar(60,90,10), Scalar(130,256,256), img_hsv_blue);

    vector<vector<cv::Point> > token_contours = getTokenContours(img_hsv_blue);

    // draw 'blue triangles' portion of token
    for(int i = 0; i < token_contours.size(); i++)
    {
        drawContours(img_token_rough, token_contours, i, Scalar(255,255,255), CV_FILLED);
    }

    circle(img_token_rough, cv::Point((img_token.size().width / 2) + 45, 65), 3, Scalar(180, 40, 40), CV_FILLED);

    // if 'noise' contours are present, do nothing
    if(token_contours.size() == 4)
    {
        // find highest and lowest x and y values from token triangles contours
        // to find x-y coordinate of top left token edge and token side length
        float y_min = getTokenPoints(token_contours, "y_min");
        float x_min = getTokenPoints(token_contours, "x_min");
        float y_max = getTokenPoints(token_contours, "y_max");
        float x_max = getTokenPoints(token_contours, "x_max");

        // reconstruct token's square shape
        Rect token(x_min, y_min, y_max - y_min, y_max - y_min);
        rectangle(img_token, token, Scalar(255,255,255), CV_FILLED);

        // find and draw the center of the token and the image
        double x_mid = x_min + ((x_max - x_min) / 2);
        double y_mid = y_min + ((y_max - y_min) / 2);
        circle(img_token, cv::Point(x_mid, y_mid), 3, Scalar(0, 0, 0), CV_FILLED);
        circle(img_token, cv::Point(img_token.size().width / 2, img_token.size().height / 2), 3, Scalar(180, 40, 40), CV_FILLED);

        double token_area = (x_max - x_min) * (y_max - y_min);

        _curr_x_offset = (4.7807/ token_area) * (x_mid - (img_token.size().width / 2));
        _curr_y_offset = ((4.7807 / token_area) * ((img_token.size().height / 2) - y_mid)) - 0.013; /*distance between gripper center and camera center*/;

        // ROS_INFO("x_diff: %0.6f   y_diff: %0.6f", x_mid - (img_token.size().width / 2), y_mid - (img_token.size().height / 2));
        // ROS_INFO("token_area: %0.6f   w: %0.6f", token_area, (44.08 / token_area));
        // ROS_INFO("x_offset: %0.6f y_offset: %0.6f\n", _curr_x_offset, _curr_y_offset);
    }
    // when hand camera is blind due to being too close to token, go straight down;
    else
    {
        _curr_x_offset = 0;
        _curr_y_offset = 0;
    }

    imshow("[Arm Controller] raw image", cv_ptr->image.clone());
    imshow("[Arm Controller] rough processed image", img_token_rough);
    imshow("[Arm Controller] processed image", img_token);

    waitKey(30);
}

/*************************Movement Functions************************/

void ArmController::pickUpToken()
{
    if(limb == "right") 
    {
        ROS_ERROR("[Arm Controller] Right arm should not pick up tokens");
    }
    else if(limb == "left")
    {
        hoverAboveTokens(STRICTPOSE);   
        ROS_INFO("grip token");
        gripToken();
        hoverAboveTokens(STRICTPOSE);  

        // bool no_token = true;
        // while(no_token)
        // {
        //     hoverAboveTokens(STRICTPOSE);
        //     // grip token; record if arm fails successfully gripped token
        //     if(gripToken()) no_token = false;
        //     // if(gripToken()) no_token = false;
        //     hoverAboveTokens(LOOSEPOSE);   
        //     // check if arm successfully gripped token
        //     // (sometimes infrared sensor falls below threshold w/o 
        //     // successfully gripping token)
        //     if(!(gripper->is_gripping()))
        //     {
        //         no_token = true;
        //         // gripper cannot suck w/o blowing first
        //         gripper->blow();   
        //     } 
        // }    
    }
}

void ArmController::placeToken(int cell_num)
{
    if(limb == "right") 
    {
        ROS_ERROR("[Arm Controller] Right arm should not pick up tokens");
    }
    else if(limb == "left")
    {
        hoverAboveBoard();
        releaseToken(cell_num);
        hoverAboveBoard();
        hoverAboveTokens(STRICTPOSE);
    }
}

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

    publishMoveCommand(joint_angles, LOOSEPOSE);
}

/**************************** PRIVATE ******************************/

/*************************Movement Functions************************/

void ArmController::hoverAboveTokens(GoalType goal)
{
    req_pose_stamped.header.frame_id = "base";
    req_pose_stamped.pose.position.x = 0.540;
    req_pose_stamped.pose.position.y = 0.660;    
    req_pose_stamped.pose.position.z = 0.350;

    req_pose_stamped.pose.orientation.x = 0.712801568376;
    req_pose_stamped.pose.orientation.y = -0.700942136419;
    req_pose_stamped.pose.orientation.z = -0.0127158080742;
    // req_pose_stamped.pose.orientation.z = -0.0127158080742;
    req_pose_stamped.pose.orientation.w = -0.0207931175453;
    // req_pose_stamped.pose.orientation.w = -0.0207931175453;

    vector<float> joint_angles = getJointAngles(req_pose_stamped);
    publishMoveCommand(joint_angles, goal);
}

bool ArmController::gripToken()
{
    ros::Time start_time = ros::Time::now();
    
    double prev_x = 0.540;
    double prev_y = 0.660;
    
    while(ros::ok())
    {
        ros::Time curr_time = ros::Time::now();
        ros::Rate loop_rate(500);

        req_pose_stamped.header.frame_id = "base";
        req_pose_stamped.pose.position.x = prev_x + 0.07 * _curr_x_offset; 
        prev_x = req_pose_stamped.pose.position.x;
        req_pose_stamped.pose.position.y = prev_y + 0.07 * _curr_y_offset;
        prev_y = req_pose_stamped.pose.position.y;

                                 // z(t) = z(0) + v * t
        req_pose_stamped.pose.position.z = 0.350 + (-0.07) * (curr_time - start_time).toSec();

        // ROS_INFO("_curr_x_offset: %0.4f", _curr_x_offset);
        // ROS_INFO("x: %0.4f y: %0.4f y: z: %0.4f range: %0.4f", req_pose_stamped.pose.position.x, req_pose_stamped.pose.position.y, req_pose_stamped.pose.position.z, curr_range);

        req_pose_stamped.pose.orientation.x = 0.712801568376;
        req_pose_stamped.pose.orientation.y = -0.700942136419;
        req_pose_stamped.pose.orientation.z = -0.0127158080742;
        req_pose_stamped.pose.orientation.w = -0.0207931175453;

        vector<float> joint_angles = getJointAngles(req_pose_stamped);

        JointCommand joint_cmd;

        // command in position mode
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

        joint_cmd_pub.publish(joint_cmd);
        loop_rate.sleep();

        ros::spinOnce();
        if(hasCollided()) 
        {
            // gripper->suck();e
            ROS_ERROR("entered hasCollided");
            break;
        }
    }

    return true;
}

void ArmController::hoverAboveBoard()
{
    req_pose_stamped.header.frame_id = "base";
    req_pose_stamped.pose.position.x = CENTER_X;
    req_pose_stamped.pose.position.y = CENTER_Y;
    req_pose_stamped.pose.position.z = 0.13621169853;

    req_pose_stamped.pose.orientation.x = 0.712801568376;
    req_pose_stamped.pose.orientation.y = -0.700942136419;
    req_pose_stamped.pose.orientation.z = -0.0127158080742;
    req_pose_stamped.pose.orientation.w = -0.0207931175453;

    vector<float> joint_angles = getJointAngles(req_pose_stamped);
    publishMoveCommand(joint_angles, LOOSEPOSE);   
}

void ArmController::releaseToken(int cell_num)
{
    if((cell_num - 1) / 3 == 0) req_pose_stamped.pose.position.x = CENTER_X + CELL_SIDE;
    if((cell_num - 1) / 3 == 1) req_pose_stamped.pose.position.x = CENTER_X;
    if((cell_num - 1) / 3 == 2) req_pose_stamped.pose.position.x = CENTER_X - CELL_SIDE;
    if(cell_num % 3 == 0) req_pose_stamped.pose.position.y = CENTER_Y - CELL_SIDE;
    if(cell_num % 3 == 1) req_pose_stamped.pose.position.y = CENTER_Y + CELL_SIDE;
    if(cell_num % 3 == 2) req_pose_stamped.pose.position.y = CENTER_Y;

    req_pose_stamped.pose.position.z = -0.13501169853;

    req_pose_stamped.pose.orientation.x = 0.712801568376;
    req_pose_stamped.pose.orientation.y = -0.700942136419;
    req_pose_stamped.pose.orientation.z = -0.0127158080742;
    req_pose_stamped.pose.orientation.w = -0.0207931175453;

    vector<float> joint_angles = getJointAngles(req_pose_stamped);
    publishMoveCommand(joint_angles, STRICTPOSE);   
    gripper->blow();
}

/*************************Location Control Functions************************/

vector<float> ArmController::getJointAngles(PoseStamped pose_stamped)
{
    // IK solver service
    SolvePositionIK ik_srv;

    ik_srv.request.pose_stamp.push_back(pose_stamped);
    ik_client.call(ik_srv);

    // ROS_DEBUG_STREAM(cout << "[Arm Controller] " << ik_srv.request << endl);
    // ROS_DEBUG_STREAM(cout << "[Arm Controller] " << ik_srv.response << endl);   
    
    // if service is successfully called
    if(ik_client.call(ik_srv))
    {
        // ROS_DEBUG("[Arm Controller] Service called");

        // store joint angles values received from service
        vector<float> joint_angles;
        joint_angles.resize(NUM_JOINTS);
        for(int i = 0; i < ik_srv.response.joints[0].position.size(); i++)
        {
            joint_angles[i] = ik_srv.response.joints[0].position[i];
        }

        bool all_zeros = true;
        for(int i = 0; i < joint_angles.size(); i++){
            if(joint_angles[i] == 0) all_zeros = false;
            // ROS_DEBUG("[Arm Controller] Joint angles %d: %0.4f", i, joint_angles[i]);
        }
        if(all_zeros == false) ROS_ERROR("[Arm Controller] Angles are all 0 radians (No solution found)");

        return joint_angles;
    }
    else 
    {
        ROS_ERROR("[Arm Controller] SolvePositionIK service was unsuccessful");
        vector<float> empty;
        return empty;
    }
}

bool ArmController::publishMoveCommand(vector<float> joint_angles, GoalType goal) 
{
    ros::Rate loop_rate(100);
    JointCommand joint_cmd;

    // command in position mode
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

    // record time at which joint angles was published to arm
    ros::Time start_time = ros::Time::now();

    while(ros::ok())
    {
        // ROS_DEBUG("[Arm Controller] In the loop");
        joint_cmd_pub.publish(joint_cmd);
        ros::spinOnce();
        loop_rate.sleep();

        if(goal == GRIPPOSE)
        {
            if(hasPoseCompleted(GRIP)) 
            {
                ROS_DEBUG("[Arm Controller] Move completed\n");
                return true;
            }     
            else if(checkForTimeout(10, GRIPPOSE, start_time)) return false; 
        }
        else if(goal == STRICTPOSE)
        {
            if(hasPoseCompleted(STRICT)) 
            {
                ROS_DEBUG("[Arm Controller] Move completed");
                return true;
            }     
            else if(checkForTimeout(10, STRICTPOSE, start_time)) return false; 
        }
        else if(goal == LOOSEPOSE)
        {
            if(hasPoseCompleted(LOOSE)) 
            {
                ROS_DEBUG("[Arm Controller] Move completed");
                return true;
            }     
            else if(checkForTimeout(8, LOOSEPOSE, start_time)) return false; 
        }
        else if(goal == COLLISION)
        {
            if(hasCollided())
            {
                gripper->suck();
                return true;
            }
            else if(checkForTimeout(10, COLLISION, start_time)) return false; 
        }
     }   
}

/*************************Checking Functions************************/

bool ArmController::checkForTimeout(int len, GoalType goal, ros::Time start_time)
{
    // if (int len) seconds has elapsed and goal has not been achieved,
    // throw an error
    string goal_str = (goal == LOOSEPOSE || goal == STRICTPOSE)? "pose" : "collision";
    ros::Time curr_time = ros::Time::now();
    if((curr_time - start_time).toSec() > len)
    {
        ROS_ERROR("[Arm Controller] %d seconds have elapsed. Goal type [%s] was not achieved", len, goal_str.c_str());
        return true;
    } 
    else {
        // ROS_DEBUG("[Arm Controller] Time limit not reached");
        return false;  
    }
}

bool ArmController::hasPoseCompleted(PoseType pose)
{
    bool same_pose = true;

    // ROS_DEBUG("Checking if pose has been completed. Strategy: %s",pose==STRICT?"strict":"loose");

    if(pose == GRIP)
    {
        // if(!withinXHundredth(curr_pose.position.x, req_pose_stamped.pose.position.x, 2)) same_pose = false; 
        // if(!withinXHundredth(curr_pose.position.y, req_pose_stamped.pose.position.y, 2)) same_pose = false;
        
        float curr_z = curr_pose.position.z;
        float req_z = req_pose_stamped.pose.position.z;

        if(!equalXDP(curr_z, req_z, 2)) same_pose = false;   
        // if(!equalXDP(curr_z, req_z, 3)) same_pose = false;  


        // ROS_DEBUG("curr_z: %0.3f req_z: %0.3f", curr_z, req_z);
        // ROS_DEBUG("(2dp) curr_z: %0.3f req_z: %0.3f", roundf(curr_z * (100)) / (100), roundf(req_z * (100)) / (100));
        // ROS_DEBUG("same_pose: %d", same_pose);

        // if(!withinXHundredth(curr_pose.orientation.x, req_pose_stamped.pose.orientation.x, 4)) same_pose = false;
        // if(!withinXHundredth(curr_pose.orientation.y, req_pose_stamped.pose.orientation.y, 4)) same_pose = false;
        // if(!withinXHundredth(curr_pose.orientation.z, req_pose_stamped.pose.orientation.z, 4)) same_pose = false;
        // if(!withinXHundredth(curr_pose.orientation.w, req_pose_stamped.pose.orientation.w, 4)) same_pose = false;
    }

    if(pose == STRICT)
    {
        if(!withinXHundredth(curr_pose.position.x, req_pose_stamped.pose.position.x, 2)) same_pose = false; 
        if(!withinXHundredth(curr_pose.position.y, req_pose_stamped.pose.position.y, 2)) same_pose = false;
        if(!withinXHundredth(curr_pose.position.z, req_pose_stamped.pose.position.z, 2)) same_pose = false;        
    }
    else if(pose == LOOSE)
    {
        if(!withinXHundredth(curr_pose.position.x, req_pose_stamped.pose.position.x, 4)) same_pose = false;  
        if(!withinXHundredth(curr_pose.position.y, req_pose_stamped.pose.position.y, 4)) same_pose = false;  
        if(!withinXHundredth(curr_pose.position.z, req_pose_stamped.pose.position.z, 4)) same_pose = false;              
    }

    if(pose == STRICT)
    {
        if(!equalXDP(curr_pose.orientation.x, req_pose_stamped.pose.orientation.x, 2)) same_pose = false;
        if(!equalXDP(curr_pose.orientation.y, req_pose_stamped.pose.orientation.y, 2)) same_pose = false;
        if(!equalXDP(curr_pose.orientation.z, req_pose_stamped.pose.orientation.z, 2)) same_pose = false;
        if(!equalXDP(curr_pose.orientation.w, req_pose_stamped.pose.orientation.w, 2)) same_pose = false;
    }
    else if(pose == LOOSE)
    {
        if(!withinXHundredth(curr_pose.orientation.x, req_pose_stamped.pose.orientation.x, 4)) same_pose = false;
        if(!withinXHundredth(curr_pose.orientation.y, req_pose_stamped.pose.orientation.y, 4)) same_pose = false;
        if(!withinXHundredth(curr_pose.orientation.z, req_pose_stamped.pose.orientation.z, 4)) same_pose = false;
        if(!withinXHundredth(curr_pose.orientation.w, req_pose_stamped.pose.orientation.w, 4)) same_pose = false;   
    }

    return same_pose;    
}

bool ArmController::hasCollided()
{
    // ROS_DEBUG_STREAM(cout << " range: " << curr_range << " max range: " << curr_max_range << " min range: " << curr_min_range << endl);
    if(curr_range != 0 && curr_max_range !=0 && curr_min_range != 0)
    {
        if(curr_range <= curr_max_range && curr_range >= curr_min_range && curr_range <= IR_RANGE_THRESHOLD)
        {
            ROS_INFO("[Arm Controller] Collision");
            ROS_INFO("range: %0.3f max range: %0.3f min range: %0.3f", curr_range, curr_max_range, curr_min_range);

            return true;
        }
        else return false;
    }
    else return false;
}

bool ArmController::withinXHundredth(float x, float y, float z)
{
    float diff = abs(x - y);
    float diffTwoDP = roundf(diff * 100) / 100;
    return diffTwoDP <= (0.01 * z) ? true : false;
}

bool ArmController::equalXDP(float x, float y, float z)
{
    // ROS_INFO("x: %0.3f y: %0.3f", x, y);
    float xTwoDP = roundf(x * pow(10, z)) / pow(10, z);
    float yTwoDP = roundf(y * pow(10, z)) / pow(10, z);

    // ROS_INFO("xTwoDP: %0.3f yTwoDP: %0.3f", xTwoDP, yTwoDP);
    return xTwoDP == yTwoDP ? true : false;    
}

/*************************Visual Servoing Functions************************/

vector<vector<cv::Point> > ArmController::getTokenContours(Mat img_hsv_blue)
{
    // find contours of blue elements in image
    vector<vector<cv::Point> > contours;
    vector<vector<cv::Point> > token_contours;
    findContours(img_hsv_blue, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    int largest_index = 0;
    int largest_area = 0;

    for(int i = 0; i < contours.size(); i++)
    {
        // remove any element with y < 65 (gripper area)
        bool not_gripper = true;
        for(int j = 0; j < contours[i].size(); j++)
        {
            vector<cv::Point> contour = contours[i];
            if(contour[j].y < 65)
            {
                not_gripper = false;
                break;
            }
        }

        bool is_triangle = true;
        vector<cv::Point> contour;
        approxPolyDP(contours[i], contour, 0.1 * arcLength(contours[i], true), true);
        if(contour.size() != 3) is_triangle = false;

        // removes 'noise' elements (all approx. have size < 150)
        if(contourArea(contours[i]) > 200 && not_gripper == true && is_triangle == true)
        {
            token_contours.push_back(contours[i]);
        }
    }

    // printf("\nsize: %lu\n", token_contours.size());
    
    return token_contours;
}

float ArmController::getTokenPoints(vector<vector<cv::Point> > token_contours, string point)
{
    // find highest and lowest x and y values from token triangles contours
    // to find x-y coordinate of top left token edge and token side length
    float y_min = (token_contours[0])[0].y;
    float x_min = (token_contours[0])[0].x;
    float y_max = 0;
    float x_max = 0;

    for(int i = 0; i < token_contours.size(); i++)
    {
        vector<cv::Point> contour = token_contours[i];
        for(int j = 0; j < contour.size(); j++)
        {
            if(y_min > contour[j].y) y_min = contour[j].y;
            if(x_min > contour[j].x) x_min = contour[j].x;
            if(y_max < contour[j].y) y_max = contour[j].y;
            if(x_max < contour[j].x) x_max = contour[j].x;
        }
    }

    if(point == "y_min") return y_min;
    if(point == "y_max") return y_max;
    if(point == "x_min") return x_min;
    if(point == "x_max") return x_max;
}
