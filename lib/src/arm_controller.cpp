#include "arm_controller/arm_controller.h"

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace cv;

/*
    pthread locks
*/

/**************************************************************************/
/*                               Utils                                    */
/**************************************************************************/

bool Utils::hasCollided(float range, float max_range, float min_range, string mode)
{
    float threshold;
    if(mode == "strict") threshold = 0.050;
    if(mode == "loose") threshold = 0.067;
    if(range <= max_range && range >= min_range && range <= threshold) return true;
    else return false;
}

bool Utils::hasPoseCompleted(Pose a, Pose b, string mode)
{
    bool same_pose = true;

    if(mode == "strict")
    {
        if(!equalXDP(a.position.x, b.position.x, 3)) {same_pose = false;} 
        if(!equalXDP(a.position.y, b.position.y, 3)) {same_pose = false;} 
    }
    else if(mode == "loose")
    {
        if(!equalXDP(a.position.x, b.position.x, 2)) {same_pose = false;} 
        if(!equalXDP(a.position.y, b.position.y, 2)) {same_pose = false;} 
    }

    if(!withinXHundredth(a.position.z, b.position.z, 1))       {same_pose = false;}    
    if(!withinXHundredth(a.orientation.x, b.orientation.x, 2)) {same_pose = false;}  
    if(!withinXHundredth(a.orientation.y, b.orientation.y, 2)) {same_pose = false;}  
    if(!withinXHundredth(a.orientation.z, b.orientation.z, 2)) {same_pose = false;}  
    if(!withinXHundredth(a.orientation.w, b.orientation.w, 2)) {same_pose = false;}

    return same_pose; 
}

bool Utils::withinXHundredth(float x, float y, float z)
{
    float diff = abs(x - y);
    float diffTwoDP = roundf(diff * 100) / 100;
    return diffTwoDP <= (0.01 * z) ? true : false;
}

bool Utils::equalXDP(float x, float y, float z)
{
    float xTwoDP = roundf(x * pow(10, z)) / pow(10, z);
    float yTwoDP = roundf(y * pow(10, z)) / pow(10, z);
    return xTwoDP == yTwoDP ? true : false;    
}

void Utils::setPosition(Pose * pose, float x, float y, float z)
{
    (*pose).position.x = x;
    (*pose).position.y = y;
    (*pose).position.z = z;
}
 
void Utils::setOrientation(Pose * pose, float x, float y, float z, float w)
{
    (*pose).orientation.x = x;
    (*pose).orientation.y = y;
    (*pose).orientation.z = z;
    (*pose).orientation.w = w;
}

void Utils::setNames(JointCommand * joint_cmd, string limb)
{
    (*joint_cmd).names.push_back(limb + "_s0");
    (*joint_cmd).names.push_back(limb + "_s1");
    (*joint_cmd).names.push_back(limb + "_e0");
    (*joint_cmd).names.push_back(limb + "_e1");
    (*joint_cmd).names.push_back(limb + "_w0");
    (*joint_cmd).names.push_back(limb + "_w1");
    (*joint_cmd).names.push_back(limb + "_w2");
}

string Utils::intToString( const int a )
{
    stringstream ss;
    ss << a;
    return ss.str();
}

/**************************************************************************/
/*                          ROSThreadClass                                */
/**************************************************************************/

// Public
ROSThreadClass::ROSThreadClass(string limb): _img_trp(_n), _limb(limb)
{
    _joint_cmd_pub = _n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + _limb + "/joint_command", 1);   
    _endpt_sub = _n.subscribe("/robot/limb/" + _limb + "/endpoint_state", 1, &ROSThreadClass::endpointCallback, this);
    _img_sub = _img_trp.subscribe("/cameras/left_hand_camera/image", 1, &ROSThreadClass::imageCallback, this);
    _ir_sub = _n.subscribe("/robot/range/left_hand_range/state", 1, &ROSThreadClass::IRCallback, this);
    _ik_client = _n.serviceClient<SolvePositionIK>("/ExternalTools/" + _limb + "/PositionKinematicsNode/IKService");
    _gripper = new ttt::Vacuum_Gripper(ttt::left);
    _init_time = ros::Time::now();
    _state.x = START;
    _state.y = 0;
}
ROSThreadClass::~ROSThreadClass() {/* empty */}

bool ROSThreadClass::StartInternalThread() {return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);}

void ROSThreadClass::WaitForInternalThreadToExit() {(void) pthread_join(_thread, NULL);}

void ROSThreadClass::endpointCallback(const baxter_core_msgs::EndpointState& msg) {_curr_pose = msg.pose;}

void ROSThreadClass::IRCallback(const sensor_msgs::RangeConstPtr& msg) 
{
    _curr_range = msg->range; 
    _curr_max_range = msg->max_range; 
    _curr_min_range = msg->min_range;
}

void ROSThreadClass::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try {cv_ptr = cv_bridge::toCvShare(msg);}
    catch(cv_bridge::Exception& e) {ROS_ERROR("[Arm Controller] cv_bridge exception: %s", e.what());}
    _curr_img = cv_ptr->image.clone();
}

geometry_msgs::Point ROSThreadClass::getState() {return _state;}

// Protected
void ROSThreadClass::InternalThreadEntry() {};

void ROSThreadClass::goToPose(PoseStamped req_pose_stamped)
{
    vector<double> joint_angles = getJointAngles(&req_pose_stamped);

    while(ros::ok)
    {
        JointCommand joint_cmd;
        joint_cmd.mode = JointCommand::POSITION_MODE;

        // joint_cmd.names
        Utils::setNames(&joint_cmd, _limb);
        joint_cmd.command.resize(7);
        // joint_cmd.angles
        for(int i = 0; i < joint_angles.size(); i++) {
            joint_cmd.command[i] = joint_angles[i];
        }

        _joint_cmd_pub.publish(joint_cmd);
        ros::Rate(500).sleep();

        if(Utils::hasPoseCompleted(_curr_pose, req_pose_stamped.pose, "loose")) {break;}
    }
}

void ROSThreadClass::goToPose(PoseStamped req_pose_stamped, string mode)
{
    vector<double> joint_angles = getJointAngles(&req_pose_stamped);

    while(ros::ok)
    {
        JointCommand joint_cmd;
        joint_cmd.mode = JointCommand::POSITION_MODE;

        // joint_cmd.names
        Utils::setNames(&joint_cmd, _limb);
        joint_cmd.command.resize(7);
        // joint_cmd.angles
        for(int i = 0; i < joint_angles.size(); i++) {
            joint_cmd.command[i] = joint_angles[i];
        }

        _joint_cmd_pub.publish(joint_cmd);
        ros::Rate(500).sleep();

        if(Utils::hasPoseCompleted(_curr_pose, req_pose_stamped.pose, mode)) {break;}
    }
}

vector<double> ROSThreadClass::getJointAngles(PoseStamped * pose_stamped)
{
    vector<double> joint_angles;
    bool all_zeros = true;
    ros::Time start = ros::Time::now();
    float thresh_z = (*pose_stamped).pose.position.z + 0.055;

    while(all_zeros)
    {
        SolvePositionIK ik_srv;
        ik_srv.request.pose_stamp.push_back(*pose_stamped);
        
        if(_ik_client.call(ik_srv))
        {
            joint_angles = ik_srv.response.joints[0].position;
            for(int i = 0; i < joint_angles.size(); i++)
            {
                if(joint_angles[i] != 0) 
                {
                    all_zeros = false; 
                    break;
                }
            }

            if(all_zeros == true) 
            {
                (*pose_stamped).pose.position.z += 0.005;
            }   
        }

        if((ros::Time::now() - start).toSec() > 5 || (*pose_stamped).pose.position.z > thresh_z) 
        {
            ROS_ERROR("[Arm Controller] Arm cannot reach requested position. Releasing any object being gripped by arm...");
            _gripper->blow();
            break;
        }
    }

    return joint_angles;
}

void ROSThreadClass::setState(int state)
{
    _state.x = state;
    _state.y = (ros::Time::now() - _init_time).toSec();
}

// Private
void * ROSThreadClass::InternalThreadEntryFunc(void * This) 
{
    ((ROSThreadClass *)This)->InternalThreadEntry(); 
    return NULL;
}

/**************************************************************************/
/*                         MoveToRestClass                                */
/**************************************************************************/

// Public
MoveToRestClass::MoveToRestClass(string limb): ROSThreadClass(limb) {}
MoveToRestClass::~MoveToRestClass(){}

// Protected
void MoveToRestClass::InternalThreadEntry()
{
    while(_curr_pose.position.x == 0 && _curr_pose.position.y == 0 && _curr_pose.position.z == 0)
    {
        ros::Rate(100).sleep();
    }

    PoseStamped req_pose_stamped;
    
    req_pose_stamped.header.frame_id = "base";
    Utils::setPosition(   &req_pose_stamped.pose, 0.292391, _limb == "left" ? 0.611039 : -0.611039, 0.181133);
    Utils::setOrientation(&req_pose_stamped.pose, 0.028927, 0.686745, 0.00352694, 0.726314);

    while(ros::ok())
    {
        JointCommand joint_cmd;
        joint_cmd.mode = JointCommand::POSITION_MODE;

        // joint_cmd.names
        Utils::setNames(&joint_cmd, _limb);
        joint_cmd.command.resize(7);
        // joint_cmd.angles
        joint_cmd.command[0] = _limb == "left" ? 1.1508690861110316   : -1.3322623142784817;
        joint_cmd.command[1] = _limb == "left" ? -0.6001699832601681  : -0.5786942522297723;
        joint_cmd.command[2] = _limb == "left" ? -0.17449031462196582 : 0.14266021327334347;
        joint_cmd.command[3] = _limb == "left" ? 2.2856313739492666   : 2.2695245756764697 ;
        joint_cmd.command[4] = _limb == "left" ? 1.8680051044474626   : -1.9945585194480093;
        joint_cmd.command[5] = _limb == "left" ? -1.4684031092033123  : -1.469170099597255 ;
        joint_cmd.command[6] = _limb == "left" ? 0.1257864246066039   : -0.011504855909140603;

        _joint_cmd_pub.publish(joint_cmd);
        ros::Rate(500).sleep();
        if(Utils::hasPoseCompleted(_curr_pose, req_pose_stamped.pose, "loose")) break;
    }

    setState(REST);
    pthread_exit(NULL);  
}  

/**************************************************************************/
/*                         PickUpTokenClass                               */
/**************************************************************************/

// Public
PickUpTokenClass::PickUpTokenClass(string limb): ROSThreadClass(limb) {}
PickUpTokenClass::~PickUpTokenClass() {}

// Protected
void PickUpTokenClass::InternalThreadEntry()
{
    while((_curr_range == 0 && _curr_min_range == 0 && _curr_max_range == 0) || _curr_img.empty())
    {
        ros::Rate(100).sleep();
    }

    hoverAboveTokens("high");
    gripToken();
    hoverAboveTokens("low");

    setState(PICK_UP);
    pthread_exit(NULL);  
}

// Private
typedef vector<vector<cv::Point> > Contours;

void PickUpTokenClass::gripToken()
{
    namedWindow("[PickUpToken] Processed", WINDOW_NORMAL);
    namedWindow("[PickUpToken] Rough", WINDOW_NORMAL);

    cv::Point2d offset;
    checkForToken(&offset);

    PoseStamped req_pose_stamped;
    ros::Time start_time = ros::Time::now();                
    cv::Point2d prev_offset(0.540, 0.540);

    while(ros::ok())
    {
        processImage(&offset);
        ros::Time now_time = ros::Time::now();

        req_pose_stamped.header.frame_id = "base";

        Utils::setPosition(&req_pose_stamped.pose, 
                            prev_offset.x + 0.07 * offset.x,
                            prev_offset.y + 0.07 * offset.y,
                            0.375 + (-0.05) * (now_time - start_time).toSec());

        prev_offset.x = prev_offset.x + 0.07 * offset.x; //cv::Point(req_pose_stamped.pose.position.x, req_pose_stamped.pose.position.y);
        prev_offset.y = prev_offset.y + 0.07 * offset.y;

        Utils::setOrientation(&req_pose_stamped.pose, 0.712801568376, -0.700942136419, -0.0127158080742, -0.0207931175453);

        vector<double> joint_angles = getJointAngles(&req_pose_stamped);

        JointCommand joint_cmd;
        joint_cmd.mode = JointCommand::POSITION_MODE;

        Utils::setNames(&joint_cmd, _limb);
        joint_cmd.command.resize(7);

        for(int i = 0; i < 7; i++) {
            joint_cmd.command[i] = joint_angles[i];
        }

        _joint_cmd_pub.publish(joint_cmd);
        ros::Rate(500).sleep();
     
        if(Utils::hasCollided(_curr_range, _curr_max_range, _curr_min_range, "strict")) {break;}
    }
    _gripper->suck();

    destroyWindow("[PickUpToken] Processed"); 
    destroyWindow("[PickUpToken] Rough");
}   

void PickUpTokenClass::hoverAboveTokens(std::string height)
{
    PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    Utils::setPosition(   &req_pose_stamped.pose, 0.540, 0.570, height == "high" ? 0.400 : 0.150);
    Utils::setOrientation(&req_pose_stamped.pose, 0.712801568376, -0.700942136419, -0.0127158080742, -0.0207931175453);
    goToPose(req_pose_stamped);
}

void PickUpTokenClass::checkForToken(cv::Point2d * offset)
{
    ros::Time start_time = ros::Time::now();
    while(offset->x == 0 && offset->y == 0)
    {
        processImage(offset);
        if((ros::Time::now() - start_time).toSec() > 1){break;}
    }

    while(offset->x == 0 && offset->y == 0)
    {
        ROS_WARN("No token detected by hand camera. Place token and press ENTER");
        char c = cin.get();
        processImage(offset);
    }
}

void PickUpTokenClass::processImage(cv::Point2d * offset)
{
    Mat black, blue, token_rough, token, board; 
    Contours contours;
    int board_y;

    isolateBlack(&black);
    isolateBoard(black.clone(), &board, &board_y);

    isolateBlue(&blue);
    isolateToken(blue.clone(), board_y, &token_rough, &contours);
    setOffset(contours, offset, &token);

    imshow("[PickUpToken] Processed", _curr_img.clone());
    imshow("[PickUpToken] Rough", token_rough);
    waitKey(30);        
}

void PickUpTokenClass::isolateBlue(Mat * output)
{
    Mat hsv;
    cvtColor(_curr_img, hsv, CV_BGR2HSV);
    inRange(hsv, Scalar(60,90,10), Scalar(130,256,256), *output);  
}

void PickUpTokenClass::isolateBlack(Mat * output)
{
    Mat gray;
    cvtColor(_curr_img, gray, CV_BGR2GRAY);
    threshold(gray, *output, 55, 255, cv::THRESH_BINARY_INV);
}

void PickUpTokenClass::isolateBoard(Mat input, Mat * output, int * board_y)
{
    *output = Mat::zeros(_curr_img.size(), CV_8UC1);
    vector<cv::Vec4i> hierarchy; // captures contours within contours 
    Contours contours;

    findContours(input, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    double largest = 0, next_largest = 0;
    int largest_index = 0, next_largest_index = 0;

    // iterate through contours and keeps track of contour w/ 2nd-largest area
    for(int i = 0; i < contours.size(); i++)
    {
        if(contourArea(contours[i], false) > largest)
        {
            next_largest = largest;
            next_largest_index = largest_index;
            largest = contourArea(contours[i], false);
            largest_index = i;
        }
        else if(next_largest < contourArea(contours[i], false) && contourArea(contours[i], false) < largest)
        {
            next_largest = contourArea(contours[i], false);
            next_largest_index = i;
        }
    }

    *output = Mat::zeros(_curr_img.size(), CV_8UC1);

    vector<cv::Point> contour = contours[next_largest_index];
    int low_y = _curr_img.size().height;
    
    for(int i = 0; i < contour.size(); i++)
    {
        if(contour[i].y < low_y) low_y = contour[i].y;
    }

    *board_y = low_y;

    line(*output, cv::Point(0, low_y), cv::Point(_curr_img.size().width, low_y), cv::Scalar(130,256,256));
    line(*output, cv::Point(0, low_y - 10), cv::Point(_curr_img.size().width - 10, low_y - 10), cv::Scalar(130,256,256));
}

void PickUpTokenClass::isolateToken(Mat input, int board_y, Mat *output, Contours *contours)
{
    Contours raw_contours, clean_contours;
    findContours(input, raw_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    int largest_index = 0, largest_area = 0;
    for(int i = 0; i < raw_contours.size(); i++)
    {
        bool not_gripper = true;
        for(int j = 0; j < raw_contours[i].size(); j++)
        {
            vector<cv::Point> contour = raw_contours[i];
            if(contour[j].y < 65) {not_gripper = false; break;}
        }

        bool is_triangle = true;
        vector<cv::Point> contour;
        approxPolyDP(raw_contours[i], contour, 0.11 * arcLength(raw_contours[i], true), true);

        if(contour.size() != 3) is_triangle = false;

        if(contourArea(raw_contours[i]) > 200 && not_gripper == true && is_triangle == true)
        {
            clean_contours.push_back(raw_contours[i]);
        }
    }

    for(int i = 0; i < clean_contours.size(); i++) 
    {
        bool within_board = false;
        vector<cv::Point> contour = clean_contours[i];
        for(int j = 0; j < contour.size(); j++)
        {
            cv::Point pt = contour[j];
            if(pt.y > board_y)
            {
                within_board = true;
                break;
            }
        }

        if(within_board == false) 
        {
            (*contours).push_back(contour);
        }
    }

    *output = Mat::zeros(_curr_img.size(), CV_8UC1);
    for(int i = 0; i < (*contours).size(); i++)
    {
        drawContours(*output, (*contours), i, Scalar(255,255,255), CV_FILLED);
    }

    line(*output, cv::Point(0, board_y), cv::Point(_curr_img.size().width, board_y), cv::Scalar(130,256,256));
}              

void PickUpTokenClass::setOffset(Contours contours, cv::Point2d *offset, Mat *output)
{
    *output = Mat::zeros(_curr_img.size(), CV_8UC1);
    
    for(int i = 0; i <contours.size(); i++)
    {
        drawContours(*output, contours, i, Scalar(255,255,255), CV_FILLED);
    }
    
    // if 'noise' contours are present, do nothing
    if(contours.size() == 4)
    {
        // find highest and lowest x and y values from token triangles contours
        // to find x-y coordinate of top left token edge and token side length
        double y_min = (contours[0])[0].y;
        double x_min = (contours[0])[0].x;
        double y_max = 0;
        double x_max = 0;

        for(int i = 0; i < contours.size(); i++)
        {
            vector<cv::Point> contour = contours[i];
            for(int j = 0; j < contour.size(); j++)
            {
                if(y_min > contour[j].y) y_min = contour[j].y;
                if(x_min > contour[j].x) x_min = contour[j].x;
                if(y_max < contour[j].y) y_max = contour[j].y;
                if(x_max < contour[j].x) x_max = contour[j].x;
            }
        }

        // reconstruct token's square shape
        Rect token(x_min, y_min, y_max - y_min, y_max - y_min);
        rectangle(*output, token, Scalar(255,255,255), CV_FILLED);

        // find and draw the center of the token and the image
        double x_mid = x_min + ((x_max - x_min) / 2);
        double y_mid = y_min + ((y_max - y_min) / 2);
        circle(*output, cv::Point(x_mid, y_mid), 3, Scalar(0, 0, 0), CV_FILLED);
        circle(*output, cv::Point(_curr_img.size().width / 2, _curr_img.size().height / 2), 3, Scalar(180, 40, 40), CV_FILLED);

        double token_area = (x_max - x_min) * (y_max - y_min);

        (*offset).x = (4.7807 /*constant*/ / token_area) * (x_mid - (_curr_img.size().width / 2)); 
        (*offset).y = (4.7807 /*constant*/ / token_area) * ((_curr_img.size().height / 2) - y_mid) - 0.013; /*distance between gripper center and camera center*/
    }
    // when hand camera is blind due to being too close to token, go straight down;
    else if(contours.size() < 4)
    {
        *offset = cv::Point2d(0,0);
    }
}

/**************************************************************************/
/*                          ScanBoardClass                                */
/**************************************************************************/

// Public
ScanBoardClass::ScanBoardClass(string limb): ROSThreadClass(limb) {}
ScanBoardClass::~ScanBoardClass() {}

vector<geometry_msgs::Point> ScanBoardClass::getOffsets() {return _offsets;}

// Protected
void ScanBoardClass::InternalThreadEntry()
{
    hoverAboveBoard();
    while(_curr_img.empty()) 
    {
        ros::Rate(100).sleep();
    }
    scan();
    // hoverAboveTokens();

    setState(SCAN);
    pthread_exit(NULL);
}

// Private
void ScanBoardClass::hoverAboveTokens()
{
    PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    Utils::setPosition(   &req_pose_stamped.pose, 0.540, 0.570, 0.400);
    Utils::setOrientation(&req_pose_stamped.pose, 0.712801568376, -0.700942136419, -0.0127158080742, -0.0207931175453);
    goToPose(req_pose_stamped);
}

void ScanBoardClass::hoverAboveBoard()
{
    PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    Utils::setPosition(   &req_pose_stamped.pose, 0.575, 0.100, 0.445);
    Utils::setOrientation(&req_pose_stamped.pose, 0.99962, -0.02741, 0, 0);
    goToPose(req_pose_stamped);
}

void ScanBoardClass::scan()
{
    float dist;
    setDepth(&dist);
    hoverAboveBoard();
    processImage("test", dist);        

}

void ScanBoardClass::setDepth(float *dist)
{
    geometry_msgs::Point init_pos = _curr_pose.position;
    ros::Time start_time = ros::Time::now();                

    while(ros::ok())
    {
        PoseStamped req_pose_stamped;
        req_pose_stamped.header.frame_id = "base";

        Utils::setPosition(&req_pose_stamped.pose, 
                            init_pos.x,
                            init_pos.y,
                            init_pos.z + (-0.07) * (ros::Time::now() - start_time).toSec());

        Utils::setOrientation(&req_pose_stamped.pose, 0.99962, -0.02741, 0, 0);

        vector<double> joint_angles = getJointAngles(&req_pose_stamped);

        JointCommand joint_cmd;
        joint_cmd.mode = JointCommand::POSITION_MODE;

        Utils::setNames(&joint_cmd, _limb);
        joint_cmd.command.resize(7);

        for(int i = 0; i < 7; i++) {
            joint_cmd.command[i] = joint_angles[i];
        }

        _joint_cmd_pub.publish(joint_cmd);
        ros::Rate(500).sleep();
     
        if(Utils::hasCollided(_curr_range, _curr_max_range, _curr_min_range, "loose")) {break;}
    }
    *dist = init_pos.z - _curr_pose.position.z + 0.04;
}

void ScanBoardClass::processImage(string mode, float dist)
{
    namedWindow("[ScanBoard] Rough", WINDOW_NORMAL);
    namedWindow("[ScanBoard] Processed", WINDOW_NORMAL);
    ros::Time start_time = ros::Time::now();

    while(ros::ok())
    {
        Contours contours;
        vector<cv::Point> centroids;
        Mat binary, board, zone = _curr_img.clone();
        int board_area;

        isolateBlack(&binary);
        isolateBoard(&contours, &board_area, binary.clone(), &board);

        waitKey(3);

        if(contours.size() == 9)
        {
            setOffsets(board_area, contours, dist, &board, &centroids);
            imshow("[ScanBoard] Processed", board);
        
            if(offsetsReachable() && mode == "run"){
                cout << "Board is positioned correctly! Proceed with game" << endl;
                break;
            }
            else if(!offsetsReachable()) {
                cout << "Board is outside the reachable zone" << endl;
                setZone(contours, dist, &centroids);

                // calls to IK solver in drawZone takes too long; makes the image update
                // very slow and hard for users to calibrate board position, which is why
                // and i nner loop is needed
                while(ros::ok())
                {

                    zone = _curr_img.clone();
                    line(zone, centroids[0], centroids[2], cv::Scalar(0,0,255), 1);
                    line(zone, centroids[2], centroids[8], cv::Scalar(0,0,255), 1);
                    line(zone, centroids[0], centroids[6], cv::Scalar(0,0,255), 1);
                    line(zone, centroids[6], centroids[8], cv::Scalar(0,0,255), 1);

                    double y_min = (contours[0])[0].y;
                    double x_min = (contours[0])[0].x;
                    double y_max = 0;
                    double x_max = 0;

                    for(int i = 0; i < contours.size(); i++)
                    {
                        vector<cv::Point> contour = contours[i];
                        for(int j = 0; j < contour.size(); j++)
                        {
                            if(y_min > contour[j].y) y_min = contour[j].y;
                            if(x_min > contour[j].x) x_min = contour[j].x;
                            if(y_max < contour[j].y) y_max = contour[j].y;
                            if(x_max < contour[j].x) x_max = contour[j].x;
                        }
                    }

                    line(zone, cv::Point(x_max, y_min), cv::Point(x_max, y_min), cv::Scalar(0,0,255), 5);

                    imshow("[ScanBoard] Rough", zone);
                    waitKey(3);
                }
            }
        }

        imshow("[ScanBoard] Rough", zone);
        imshow("[ScanBoard] Processed", board);
    }

    destroyWindow("[ScanBoard] Rough");
    destroyWindow("[ScanBoard] Processed");
}

void ScanBoardClass::isolateBlack(Mat * output)
{
    Mat gray;
    cvtColor(_curr_img, gray, CV_BGR2GRAY);
    threshold(gray, *output, 55, 255, cv::THRESH_BINARY);
}

void ScanBoardClass::isolateBoard(Contours * contours, int * board_area, Mat input, Mat * output)
{
    *output = Mat::zeros(_curr_img.size(), CV_8UC1);
    vector<cv::Vec4i> hierarchy; // captures contours within contours 

    findContours(input, *contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    double largest = 0, next_largest = 0;
    int largest_index = 0, next_largest_index = 0;

    // iterate through contours and keeps track of contour w/ 2nd-largest area
    for(int i = 0; i < (*contours).size(); i++)
    {
        if(contourArea((*contours)[i], false) > largest)
        {
            next_largest = largest;
            next_largest_index = largest_index;
            largest = contourArea((*contours)[i], false);
            largest_index = i;
        }
        else if(next_largest < contourArea((*contours)[i], false) && contourArea((*contours)[i], false) < largest)
        {
            next_largest = contourArea((*contours)[i], false);
            next_largest_index = i;
        }
    }

    *board_area = contourArea((*contours)[next_largest_index], false);

    drawContours(*output, *contours, next_largest_index, Scalar(255,255,255), CV_FILLED, 8, hierarchy);

    findContours(*output, *contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    largest = 0;
    largest_index = 0;

    // iterate through contours and keeps track of contour w/ largest area
    for(int i = 0; i < (*contours).size(); i++)
    {
        if(contourArea((*contours)[i], false) > largest)
        {
            largest = contourArea((*contours)[i], false);
            largest_index = i;
        }
    } 

    // remove outer board contours
    (*contours).erase((*contours).begin() + largest_index);

    for(int i = 0; i < (*contours).size(); i++)
    {
        if(contourArea((*contours)[i], false) < 200)
        {
            (*contours).erase((*contours).begin() + i);
        } 
    }

    for(int i = 0; i < (*contours).size(); i++)
    {
        drawContours(*output, *contours, i, Scalar(255,255,255), CV_FILLED);
    }
}

bool ScanBoardClass::descendingX(vector<cv::Point> i, vector<cv::Point> j) 
{
    double x_i = moments(i, false).m10 / moments(i, false).m00;
    double x_j = moments(j, false).m10 / moments(j, false).m00;

    return x_i > x_j;
}

void ScanBoardClass::setOffsets(int board_area, Contours contours, float dist, Mat *output, vector<cv::Point> *centroids)
{
    cv::Point center(_curr_img.size().width / 2, _curr_img.size().height / 2);
    circle(*output, center, 3, Scalar(180,40,40), CV_FILLED);
    cv::putText(*output, "Center", center, cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(180,40,40));

    for(int i = contours.size(); i >= 3; i -= 3)
    {
        std::sort(contours.begin() + (i - 3), contours.begin() + i, descendingX);        
    }

    _offsets.resize(9);
    (*centroids).resize(9);
    for(int i = contours.size() - 1; i >= 0; i--)
    {
        double x = moments(contours[i], false).m10 / cv::moments(contours[i], false).m00;
        double y = moments(contours[i], false).m01 / cv::moments(contours[i], false).m00;
        cv::Point centroid(x,y);  

        (*centroids)[i] = centroid;

        cv::putText(*output, Utils::intToString(i), centroid, cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(180,40,40));
        // circle(*output, centroid, 2, Scalar(180,40,40), CV_FILLED);

        _offsets[i].x = (centroid.y - center.y) * 0.0025 * dist + 0.04;  
        _offsets[i].y = (centroid.x - center.x) * 0.0025 * dist;
        _offsets[i].z = dist - 0.085;
    }
}

void ScanBoardClass::setZone(Contours contours, float dist, vector<cv::Point> * centroids)
{

    while(pointReachable((*centroids)[0], dist)) {(*centroids)[0].x += 10.0;}
    while(pointReachable((*centroids)[2], dist)) {(*centroids)[2].x -= 10.0;}
    // while(pointReachable((*centroids)[6], dist)) {(*centroids)[6].x += 10.0;}
    // while(pointReachable((*centroids)[8], dist)) {(*centroids)[8].x -= 10.0;}

    while(!pointReachable((*centroids)[0], dist)) {(*centroids)[0].x -= 5.0;}
    while(!pointReachable((*centroids)[2], dist)) {(*centroids)[2].x += 5.0;}
    while(!pointReachable((*centroids)[6], dist)) {(*centroids)[6].x -= 5.0;}
    while(!pointReachable((*centroids)[8], dist)) {(*centroids)[8].x += 5.0;}




}

bool ScanBoardClass::offsetsReachable()
{
    for(int i = 0; i < 9; i++)
    {
        PoseStamped req_pose_stamped;
        req_pose_stamped.header.frame_id = "base";
        Utils::setPosition( &req_pose_stamped.pose, 
                            0.575 + _offsets[i].x, 
                            0.100 + _offsets[i].y, 
                            0.445 - _offsets[i].z);
        Utils::setOrientation(&req_pose_stamped.pose, 0.712801568376, -0.700942136419, -0.0127158080742, -0.0207931175453);

        vector<double> joint_angles = getJointAngles(&req_pose_stamped);
        bool all_zeros = true;
        for(int j = 0; j < joint_angles.size(); j++)
        {
            if(joint_angles[j] != 0) 
            {
                all_zeros = false;
                break;
            }
        }    
        if(all_zeros) return false;    
    }
    return true;
}

bool ScanBoardClass::pointReachable(cv::Point centroid, float dist)
{
    cv::Point center(_curr_img.size().width / 2, _curr_img.size().height / 2);
    geometry_msgs::Point offset;

    offset.x = (centroid.y - center.y) * 0.0025 * dist + 0.04;  
    offset.y = (centroid.x - center.x) * 0.0025 * dist;
    offset.z = dist - 0.085;

    PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "base";
    Utils::setPosition( &pose_stamped.pose, 
                        0.575 + offset.x, 0.100 + offset.y, 0.445 - offset.z);
    Utils::setOrientation(&pose_stamped.pose, 0.712801568376, -0.700942136419, -0.0127158080742, -0.0207931175453);

    vector<double> joint_angles = getJointAngles(&pose_stamped);
    bool all_zeros = true;
    for(int j = 0; j < joint_angles.size(); j++)
    {
        if(joint_angles[j] != 0) 
        {
            all_zeros = false;
            break;
        }
    }    
    
    return all_zeros ? false : true;
}

/**************************************************************************/
/*                         PutDownTokenClass                              */
/**************************************************************************/

// Public
PutDownTokenClass::PutDownTokenClass(string limb): ROSThreadClass(limb) {}        
PutDownTokenClass::~PutDownTokenClass() {}

void PutDownTokenClass::setCell(int cell) {_cell = cell;}
void PutDownTokenClass::setOffsets(vector<geometry_msgs::Point> offsets) {_offsets = offsets;}

// Protected
void PutDownTokenClass::InternalThreadEntry()
{
    hoverAboveBoard();
    ros::Duration(0.5).sleep();
    hoverAboveCell();
    ros::Duration(0.5).sleep();
    _gripper->blow();
    hoverAboveBoard();
    hoverAboveTokens();

    setState(PUT_DOWN);
    pthread_exit(NULL);  
}  

// Private
void PutDownTokenClass::hoverAboveCell()
{
    PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    Utils::setPosition( &req_pose_stamped.pose, 0.575 + _offsets[_cell - 1].x, 
                        0.100 + _offsets[_cell - 1].y, 
                        0.445 - _offsets[_cell - 1].z);
    Utils::setOrientation(&req_pose_stamped.pose, 0.712801568376, -0.700942136419, -0.0127158080742, -0.0207931175453);

    goToPose(req_pose_stamped);
}

void PutDownTokenClass::hoverAboveBoard()
{
    PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    Utils::setPosition( &req_pose_stamped.pose, 0.575 + _offsets[4].x, 
                        0.100 + _offsets[4].y, 
                        0.200);
    Utils::setOrientation(&req_pose_stamped.pose, 0.712801568376, -0.700942136419, -0.0127158080742, -0.0207931175453);
    goToPose(req_pose_stamped);
}

void PutDownTokenClass::hoverAboveTokens()
{
    PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    Utils::setPosition(   &req_pose_stamped.pose, 0.540, 0.570, 0.400);
    Utils::setOrientation(&req_pose_stamped.pose, 0.712801568376, -0.700942136419, -0.0127158080742, -0.0207931175453);
    goToPose(req_pose_stamped);
}

/**************************************************************************/
/*                            ArmController                               */
/**************************************************************************/

ArmController::ArmController(string limb): _limb(limb) 
{
    _rest_class = new MoveToRestClass(_limb);
    _pick_class = new PickUpTokenClass(_limb);
    _scan_class = new ScanBoardClass(_limb);
    _put_class = new PutDownTokenClass(_limb);
}
ArmController::~ArmController(){}

int ArmController::getState()
{
    float len_time = 0;
    int state = 0;

    if(_rest_class->getState().y > len_time) 
    {
        len_time = _rest_class->getState().y; 
        state = _rest_class->getState().x;            
    }

    if(_pick_class->getState().y > len_time) 
    {
        len_time = _pick_class->getState().y; 
        state = _pick_class->getState().x;
    }

    if(_scan_class->getState().y > len_time) 
    {
        len_time = _scan_class->getState().y; 
        state = _scan_class->getState().x;
    }

    if(_put_class->getState().y > len_time) 
    {
        len_time = _put_class->getState().y; 
        state = _put_class->getState().x;            
    }

    return state;
}

void ArmController::moveToRest() {_rest_class->StartInternalThread();}

void ArmController::pickUpToken() {_pick_class->StartInternalThread();}

void ArmController::scanBoard() {_scan_class->StartInternalThread();}

void ArmController::putDownToken(int cell) 
{
    _put_class->setOffsets(_scan_class->getOffsets());
    _put_class->setCell(cell);
    _put_class->StartInternalThread();
}    

