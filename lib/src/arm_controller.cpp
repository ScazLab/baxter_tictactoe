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

ArmController::ArmController(string limb): _img_trp(_n), _limb(limb)
{ 
    if(_limb != "left" && _limb != "right"){
        ROS_ERROR("[Arm Controller] Invalid limb. Acceptable values are: right / left");
        ros::shutdown();
    }

    ROS_DEBUG_STREAM(cout << "[Arm Controller] " << _limb << endl);

    // see header file for descriptions
    _joint_cmd_pub = _n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + _limb + "/joint_command", 1);   
    _endpt_sub = _n.subscribe("/robot/limb/" + _limb + "/endpoint_state", 1, &ArmController::endpointCallback, this);
    _img_sub = _img_trp.subscribe("/cameras/left_hand_camera/image", 1, &ArmController::imageCallback, this);

    _ir_sub = _n.subscribe("/robot/range/left_hand_range/state", 1, &ArmController::IRCallback, this);

    _ik_client = _n.serviceClient<SolvePositionIK>("/ExternalTools/" + _limb + "/PositionKinematicsNode/IKService");
    _gripper = new Vacuum_Gripper(ttt::left);

    namedWindow("[Arm Controller] raw image", WINDOW_NORMAL);
    namedWindow("[Arm Controller] rough processed image", WINDOW_NORMAL);
    namedWindow("[Arm Controller] processed image", WINDOW_NORMAL);

    NUM_JOINTS = 7;
    OFFSET_CONSTANT = 0.1;
    CENTER_X = 0.725;
    CENTER_Y = 0.200; 
    CELL_SIDE = 0.15;
    IR_RANGE_THRESHOLD = 0.060;

    _release_mode = false;
    _grip_mode = false;
    _token_present = false;

    for(int i = 0; i < 9; i++)
    {
        _offset_cell.push_back(cv::Point(0,0));
    }
    _offset_token = cv::Point(0,0);
    _curr_range = 0;
    _curr_max_range = 0;
    _curr_min_range = 0;
}

ArmController::~ArmController() {
    destroyWindow("[Arm Controller] raw image");
    destroyWindow("[Arm Controller] rough processed image");
    destroyWindow("[Arm Controller] processed image");
}

/*************************Callback Functions************************/

string int_to_string( const int a )
{
    stringstream ss;
    ss << a;
    return ss.str();
}

bool x_descending(vector<cv::Point> i, vector<cv::Point> j) 
{
    double x_i = moments(i, false).m10 / cv::moments(i, false).m00;
    double x_j = moments(j, false).m10 / cv::moments(j, false).m00;

    return x_i > x_j;
}

void ArmController::endpointCallback(const EndpointState& msg)
{
    // update current pose
    _curr_pose = msg.pose;
}

void ArmController::IRCallback(const RangeConstPtr& msg)
{
    // ROS_DEBUG_STREAM(cout << "range: " << msg->range << " max range: " << msg->max_range << " min range: " << msg->min_range << endl);
    // ROS_DEBUG_STREAM(cout << "range: " << _curr_range << " max range: " << _curr_max_range << " min range: " << _curr_min_range << endl);

    // update current range
    _curr_range = msg->range;
    _curr_max_range = msg->max_range;
    _curr_min_range = msg->min_range;
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

    Mat img_gray;
    Mat img_binary;
    Mat img_board = Mat::zeros(cv_ptr->image.size(), CV_8UC1);

    // convert image color model from BGR to grayscale
    cvtColor(cv_ptr->image.clone(), img_gray, CV_BGR2GRAY);
    // convert grayscale image to binary image, using 155 threshold value to 
    // isolate white-colored board
    threshold(img_gray, img_binary, 70, 255, cv::THRESH_BINARY);

    // a contour is an array of x-y coordinates describing the boundaries of an object
    vector<vector<cv::Point> > board_contours;
    // Vec4i = vectors w/ 4 ints
    vector<cv::Vec4i> hierarchy;

    // find white edges of outer board by finding contours (i.e boundaries)
    findContours(img_binary, board_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    double largest_area = 0, next_largest_area = 0;
    int largest_area_index = 0, next_largest_area_index = 0;

    // iterate through contours and keeps track of contour w/ 2nd-largest area
    for(int i = 0; i < board_contours.size(); i++)
    {
        if(contourArea(board_contours[i], false) > largest_area)
        {
            next_largest_area = largest_area;
            next_largest_area_index = largest_area_index;
            largest_area = contourArea(board_contours[i], false);
            largest_area_index = i;
        }
        else if(next_largest_area < contourArea(board_contours[i], false) && contourArea(board_contours[i], false) < largest_area)
        {
            next_largest_area = contourArea(board_contours[i], false);
            next_largest_area_index = i;
        }
    }

    float board_area = contourArea(board_contours[largest_area_index], false);

    drawContours(img_board, board_contours, next_largest_area_index, Scalar(255,255,255), CV_FILLED, 8, hierarchy);
    findContours(img_board, board_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    largest_area = 0;
    largest_area_index = 0;

    // iterate through contours and keeps track of contour w/ largest area
    for(int i = 0; i < board_contours.size(); i++)
    {
        if(contourArea(board_contours[i], false) > largest_area)
        {
            largest_area = contourArea(board_contours[i], false);
            largest_area_index = i;
        }
    } 

    // remove outer board contours
    board_contours.erase(board_contours.begin() + largest_area_index);

    img_board = Mat::zeros(cv_ptr->image.size(), CV_8UC1);
    for(int i = 0; i < board_contours.size(); i++)
    {
        drawContours(img_board, board_contours, i, Scalar(255,255,255), CV_FILLED);
    }

    cv::Point img_center(img_board.size().width / 2, img_board.size().height / 2);
    circle(img_board, img_center, 1, Scalar(180,40,40), CV_FILLED);

    for(int i = 0; i < board_contours.size(); i++)
    {
        if(contourArea(board_contours[i], false) < 150)
        {
            board_contours.erase(board_contours.begin() + i);
        } 
    }

    for(int i = 0; i <= 6; i+= 3)
    {
        sort(board_contours.begin() + i, board_contours.begin() + i + 3, x_descending);        
    }

    // sort(board_contours.begin(), board_contours.begin() + 3, x_descending);        

    for(int i = board_contours.size() - 1; i >= 0; i--)
    {
        double x = moments(board_contours[i], false).m10 / cv::moments(board_contours[i], false).m00;
        double y = moments(board_contours[i], false).m01 / cv::moments(board_contours[i], false).m00;
        cv::Point centroid(x,y);  
        cv::putText(img_board, int_to_string(board_contours.size() - i), centroid, cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(180,40,40));
    }

    // sort(board_contours.begin(), board_contours.end(), x_descending);        

    // for(int i = board_contours.size() - 1; i >= 0; i--)
    // {
    //     double x = moments(board_contours[i], false).m10 / cv::moments(board_contours[i], false).m00;
    //     double y = moments(board_contours[i], false).m01 / cv::moments(board_contours[i], false).m00;
    //     cv::Point centroid(x,y);  

    //     cv::putText(img_board, int_to_string(board_contours.size() - i), centroid, cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(180,40,40));

    //     _offset_cell[board_contours.size() - 1 - i].x = (4.7807 /*constant*/ / board_area) * (img_center.x - centroid.x);
    //     _offset_cell[board_contours.size() - 1 - i].y = (4.7807 /*constant*/ / board_area) * (img_center.y - centroid.y);        
    // }

    // ROS_INFO("offset_x: %0.4f offset_y: %0.4f", img_center.x - centroid.x, img_center.y - centroid.x);

    imshow("[Arm Controller] rough processed image", img_binary);
    imshow("[Arm Controller] processed image", img_board);

    if(_grip_mode)
    {
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

        // check if token is within camera's field of view when hovering above tokens
        geometry_msgs::Pose above_tokens;
        above_tokens.position.x = 0.540;
        above_tokens.position.x = 0.540;
        above_tokens.position.y = 0.660;    
        above_tokens.position.z = 0.350;
        above_tokens.orientation.x = 0.712801568376;
        above_tokens.orientation.y = -0.700942136419;
        above_tokens.orientation.z = -0.0127158080742;
        above_tokens.orientation.z = -0.0127158080742;
        above_tokens.orientation.w = -0.0207931175453;

        if(hasPoseCompleted(STRICT, above_tokens) && token_contours.size() == 4) _token_present = true;
        else _token_present = false;

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
            _offset_token.x = (4.7807 /*constant*/ / token_area) * (x_mid - (img_token.size().width / 2));
            _offset_token.y = ((4.7807 /*constant*/ / token_area) * ((img_token.size().height / 2) - y_mid)) - 0.013; /*distance between gripper center and camera center*/

            // ROS_INFO("x_diff: %0.6f   y_diff: %0.6f", x_mid - (img_token.size().width / 2), y_mid - (img_token.size().height / 2));
            // ROS_INFO("token_area: %0.6f   w: %0.6f", token_area, (44.08 / token_area));
            // ROS_INFO("x_offset: %0.6f y_offset: %0.6f\n", _x_offset_token, _y_offset_token);
        }
        // when hand camera is blind due to being too close to token, go straight down;
        else
        {
            _offset_token.x = 0;
            _offset_token.y = 0;
        }
        imshow("[Arm Controller] rough processed image", img_token_rough);
        imshow("[Arm Controller] processed image", img_token);
    }

    imshow("[Arm Controller] raw image", cv_ptr->image.clone());

    waitKey(30);
}

/*************************Movement Functions************************/

void ArmController::pickUpToken()
{
    _grip_mode = true;

    if(_limb == "right") 
    {
        ROS_ERROR("[Arm Controller] Right arm should not pick up tokens");
    }
    else if(_limb == "left")
    {
        ROS_WARN("Arm will not move until token is in the hand camera's field of view");
        bool no_token = true;
        while(no_token)
        {
            hoverAboveTokens(STRICTPOSE);
            ros::Duration(2).sleep();
            if(gripToken()) no_token = false;
            hoverAboveTokens(LOOSEPOSE);                        
        }
    }

    _grip_mode = false;
}

void ArmController::placeToken(int cell_num)
{
    _release_mode = true;

    if(_limb == "right") 
    {
        ROS_ERROR("[Arm Controller] Right arm should not pick up tokens");
    }
    else if(_limb == "left")
    {
        ros::Duration(0.25).sleep();
        hoverAboveBoard();
        // hoverAboveCell(cell_num);


        // ros::Duration(0.5).sleep();
        // releaseToken(cell_num);
        // ros::Duration(0.25).sleep();
        // hoverAboveBoard();
        // ros::Duration(0.25).sleep();
        // hoverAboveTokens(STRICTPOSE);
    }

    _release_mode = false;
}

void ArmController::moveToRest() 
{
    vector<float> joint_angles;
    joint_angles.resize(NUM_JOINTS);

    // requested position and orientation is filled out despite hardcoding of joint angles
    // to double-check if move has been completed using hasPoseCompleted()
    _req_pose_stamped.header.frame_id = "base";
    _req_pose_stamped.pose.position.x = 0.292391;
    _req_pose_stamped.pose.position.z = 0.181133;
    _req_pose_stamped.pose.orientation.x = 0.028927;
    _req_pose_stamped.pose.orientation.y = 0.686745;
    _req_pose_stamped.pose.orientation.z = 0.00352694;
    _req_pose_stamped.pose.orientation.w = 0.726314;

    // joint angles are hardcoded as opposed to relying on the IK solver to provide a solution given
    // the requested pose because testing showed that the IK solver would fail approx. 9/10 to find 
    // a joint angles combination for the left arm rest pose.
    if(_limb == "left")
    {
        joint_angles[0] = 1.1508690861110316;
        joint_angles[1] = -0.6001699832601681;
        joint_angles[2] = -0.17449031462196582;
        joint_angles[3] = 2.2856313739492666;
        joint_angles[4] = 1.8680051044474626;
        joint_angles[5] = -1.4684031092033123;
        joint_angles[6] = 0.1257864246066039;
        _req_pose_stamped.pose.position.y = 0.611039; 
    }
    else if(_limb == "right") 
    {
        joint_angles[0] = -1.3322623142784817;
        joint_angles[1] = -0.5786942522297723;
        joint_angles[2] = 0.14266021327334347;
        joint_angles[3] = 2.2695245756764697;
        joint_angles[4] = -1.9945585194480093;
        joint_angles[5] = -1.469170099597255;
        joint_angles[6] = -0.011504855909140603;
        _req_pose_stamped.pose.position.y = -0.611039; 
    }

    publishMoveCommand(joint_angles, LOOSEPOSE);
}

/**************************** PRIVATE ******************************/

/*************************Movement Functions************************/

void ArmController::hoverAboveTokens(GoalType goal)
{
    _req_pose_stamped.header.frame_id = "base";
    _req_pose_stamped.pose.position.x = 0.540;
    _req_pose_stamped.pose.position.y = 0.660;    
    _req_pose_stamped.pose.position.z = 0.350;

    _req_pose_stamped.pose.orientation.x = 0.712801568376;
    _req_pose_stamped.pose.orientation.y = -0.700942136419;
    _req_pose_stamped.pose.orientation.z = -0.0127158080742;
    // req_pose_stamped.pose.orientation.z = -0.0127158080742;
    _req_pose_stamped.pose.orientation.w = -0.0207931175453;
    // req_pose_stamped.pose.orientation.w = -0.0207931175453;

    vector<float> joint_angles = getJointAngles(_req_pose_stamped);
    publishMoveCommand(joint_angles, goal);
}

bool ArmController::gripToken()
{
    ros::Time start_time = ros::Time::now();
    
    double prev_x = 0.540;
    double prev_y = 0.660;
    
    if(_token_present == false) return false;
    
    while(ros::ok())
    {
        ros::Time curr_time = ros::Time::now();
        ros::Rate loop_rate(500);

        _req_pose_stamped.header.frame_id = "base";
        _req_pose_stamped.pose.position.x = prev_x + 0.07 * _offset_token.x;
        prev_x = _req_pose_stamped.pose.position.x;
        _req_pose_stamped.pose.position.y = prev_y + 0.07 * _offset_token.y;
        prev_y = _req_pose_stamped.pose.position.y;

                                 // z(t) = z(0) + v * t
        _req_pose_stamped.pose.position.z = 0.350 + (-0.05) * (curr_time - start_time).toSec();

        // ROS_INFO("_x_offset_token: %0.4f", _x_offset_token);
        // ROS_INFO("x: %0.4f y: %0.4f y: z: %0.4f range: %0.4f", req_pose_stamped.pose.position.x, req_pose_stamped.pose.position.y, req_pose_stamped.pose.position.z, _curr_range);

        _req_pose_stamped.pose.orientation.x = 0.712801568376;
        _req_pose_stamped.pose.orientation.y = -0.700942136419;
        _req_pose_stamped.pose.orientation.z = -0.0127158080742;
        _req_pose_stamped.pose.orientation.w = -0.0207931175453;

        vector<float> joint_angles = getJointAngles(_req_pose_stamped);

        JointCommand joint_cmd;

        // command in position mode
        joint_cmd.mode = JointCommand::POSITION_MODE;

        // command joints in the order shown in baxter_interface
        joint_cmd.names.push_back(_limb + "_s0");
        joint_cmd.names.push_back(_limb + "_s1");
        joint_cmd.names.push_back(_limb + "_e0");
        joint_cmd.names.push_back(_limb + "_e1");
        joint_cmd.names.push_back(_limb + "_w0");
        joint_cmd.names.push_back(_limb + "_w1");
        joint_cmd.names.push_back(_limb + "_w2");

        // set your calculated velocities
        joint_cmd.command.resize(NUM_JOINTS);

        // set joint angles
        for(int i = 0; i < NUM_JOINTS; i++)
        {
            joint_cmd.command[i] = joint_angles[i];
        }

        _joint_cmd_pub.publish(joint_cmd);
        loop_rate.sleep();

        ros::spinOnce();
        if(hasCollided()) 
        {
            _gripper->suck();
            break;
        }
    }

    return true;
}

void ArmController::hoverAboveBoard()
{
    _req_pose_stamped.header.frame_id = "base";
    _req_pose_stamped.pose.position.x = CENTER_X;
    _req_pose_stamped.pose.position.y = CENTER_Y;
    _req_pose_stamped.pose.position.z = 0.500;

    _req_pose_stamped.pose.orientation.x = -0.00148564331811;
    _req_pose_stamped.pose.orientation.y = 0.999783174154;
    _req_pose_stamped.pose.orientation.z = -0.0153224515183;
    _req_pose_stamped.pose.orientation.w = 0.0140221261632;

    vector<float> joint_angles = getJointAngles(_req_pose_stamped);
    publishMoveCommand(joint_angles, LOOSEPOSE);   
}

void ArmController::releaseToken(int cell_num)
{
    if((cell_num - 1) / 3 == 0) _req_pose_stamped.pose.position.x = CENTER_X + CELL_SIDE;
    if((cell_num - 1) / 3 == 1) _req_pose_stamped.pose.position.x = CENTER_X;
    if((cell_num - 1) / 3 == 2) _req_pose_stamped.pose.position.x = CENTER_X - CELL_SIDE;
    if(cell_num % 3 == 0) _req_pose_stamped.pose.position.y = CENTER_Y - CELL_SIDE;
    if(cell_num % 3 == 1) _req_pose_stamped.pose.position.y = CENTER_Y + CELL_SIDE;
    if(cell_num % 3 == 2) _req_pose_stamped.pose.position.y = CENTER_Y;

    _req_pose_stamped.pose.position.z = -0.13501169853;

    _req_pose_stamped.pose.orientation.x = 0.712801568376;
    _req_pose_stamped.pose.orientation.y = -0.700942136419;
    _req_pose_stamped.pose.orientation.z = -0.0127158080742;
    _req_pose_stamped.pose.orientation.w = -0.0207931175453;

    vector<float> joint_angles = getJointAngles(_req_pose_stamped);
    publishMoveCommand(joint_angles, STRICTPOSE);   
    _gripper->blow();
}

/*************************Location Control Functions************************/

vector<float> ArmController::getJointAngles(PoseStamped pose_stamped)
{
    // IK solver service
    SolvePositionIK ik_srv;

    ik_srv.request.pose_stamp.push_back(pose_stamped);
    _ik_client.call(ik_srv);

    // ROS_DEBUG_STREAM(cout << "[Arm Controller] " << ik_srv.request << endl);
    // ROS_DEBUG_STREAM(cout << "[Arm Controller] " << ik_srv.response << endl);   
    
    // if service is successfully called
    if(_ik_client.call(ik_srv))
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
    joint_cmd.names.push_back(_limb + "_s0");
    joint_cmd.names.push_back(_limb + "_s1");
    joint_cmd.names.push_back(_limb + "_e0");
    joint_cmd.names.push_back(_limb + "_e1");
    joint_cmd.names.push_back(_limb + "_w0");
    joint_cmd.names.push_back(_limb + "_w1");
    joint_cmd.names.push_back(_limb + "_w2");

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
        _joint_cmd_pub.publish(joint_cmd);
        ros::spinOnce();
        loop_rate.sleep();

        if(goal == STRICTPOSE)
        {
            if(hasPoseCompleted(STRICT, _req_pose_stamped.pose)) 
            {
                ROS_DEBUG("[Arm Controller] Move completed");
                return true;
            }     
            else if(checkForTimeout(10, STRICTPOSE, start_time)) return false; 
        }
        else if(goal == LOOSEPOSE)
        {
            if(hasPoseCompleted(LOOSE, _req_pose_stamped.pose)) 
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
                _gripper->suck();
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

bool ArmController::hasPoseCompleted(PoseType type, Pose req_pose)
{
    bool same_pose = true;

    if(type == STRICT)
    {
        if(!withinXHundredth(_curr_pose.position.x, req_pose.position.x, 1)  || 
            !withinXHundredth(_curr_pose.position.y, req_pose.position.y, 1) || 
            !withinXHundredth(_curr_pose.position.z, req_pose.position.z, 2) ||   
            !equalXDP(_curr_pose.orientation.x, req_pose.orientation.x, 2)   || 
            !equalXDP(_curr_pose.orientation.y, req_pose.orientation.y, 2)   || 
            !equalXDP(_curr_pose.orientation.z, req_pose.orientation.z, 2)   || 
            !equalXDP(_curr_pose.orientation.w, req_pose.orientation.w, 2)) same_pose = false;
    }
    else if(type == LOOSE)
    {
        if(!withinXHundredth(_curr_pose.position.x, req_pose.position.x, 4)         ||
            !withinXHundredth(_curr_pose.position.y, req_pose.position.y, 4)        ||
            !withinXHundredth(_curr_pose.position.z, req_pose.position.z, 4)        ||
            !withinXHundredth(_curr_pose.orientation.x, req_pose.orientation.x, 4)  ||
            !withinXHundredth(_curr_pose.orientation.y, req_pose.orientation.y, 4)  ||
            !withinXHundredth(_curr_pose.orientation.z, req_pose.orientation.z, 4)  ||
            !withinXHundredth(_curr_pose.orientation.w, req_pose.orientation.w, 4)) same_pose = false;
    }

    return same_pose;    
}

bool ArmController::hasCollided()
{
    // ROS_DEBUG_STREAM(cout << " range: " << _curr_range << " max range: " << _curr_max_range << " min range: " << _curr_min_range << endl);
    if(_curr_range != 0 && _curr_max_range !=0 && _curr_min_range != 0)
    {
        if(_curr_range <= _curr_max_range && _curr_range >= _curr_min_range && _curr_range <= IR_RANGE_THRESHOLD)
        {
            ROS_INFO("[Arm Controller] Collision");
            ROS_INFO("range: %0.3f max range: %0.3f min range: %0.3f", _curr_range, _curr_max_range, _curr_min_range);

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
