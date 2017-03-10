#include "baxter_tictactoe/ttt_controller.h"

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace cv;

/**************************************************************************/
/*                            TTTController                               */
/**************************************************************************/

TTTController::TTTController(string name, string limb, bool no_robot, bool use_forces):
                             ArmCtrl(name, limb, no_robot, use_forces, false),
                             r(100), _img_trp(_n), _is_img_empty(true)
{
    pthread_mutexattr_t _mutex_attr;
    pthread_mutexattr_init(&_mutex_attr);
    pthread_mutexattr_settype(&_mutex_attr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(&_mutex_img, &_mutex_attr);

    XmlRpc::XmlRpcValue hsv_red_symbols;
    ROS_ASSERT_MSG(_n.getParam("hsv_red",hsv_red_symbols), "No HSV params for RED!");
    hsv_red=hsvColorRange(hsv_red_symbols);

    XmlRpc::XmlRpcValue hsv_blue_symbols;
    ROS_ASSERT_MSG(_n.getParam("hsv_blue",hsv_blue_symbols), "No HSV params for BLUE!");
    hsv_blue=hsvColorRange(hsv_blue_symbols);

    setHomeConfiguration();

    insertAction(ACTION_SCAN,    static_cast<f_action>(&TTTController::scanBoardImpl));
    insertAction(ACTION_PICKUP,  static_cast<f_action>(&TTTController::pickUpTokenImpl));
    insertAction(ACTION_PUTDOWN, static_cast<f_action>(&TTTController::putDownTokenImpl));

    if (getLimb() == "left")
    {
        _img_sub = _img_trp.subscribe("/cameras/"+getLimb()+"_hand_camera/image",
                               SUBSCRIBER_BUFFER, &TTTController::imageCb, this);
    }

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

/**************************************************************************/
/*                         PickUpToken                                    */
/**************************************************************************/
bool TTTController::gripToken()
{
    createCVWindows();
    cv::Point offset(0,0);
    // check if token is present before starting movement loop
    // (prevent gripper from colliding with play surface)
    while(RobotInterface::ok())
    {
        if(computeTokenOffset(offset)) break;

        ROS_WARN_THROTTLE(2,"No token detected by hand camera.");
        r.sleep();
    }

    offset = cv::Point(0,0);
    ros::Time start_time = ros::Time::now();
    cv::Point2d prev_offset(0.540, 0.540);

    double start_z = getPos().z;

    while(RobotInterface::ok())
    {
        computeTokenOffset(offset);

        // move incrementally towards token
        double px = getPos().x - 0.07 * offset.y / 500;  // fixed constant to avoid going too fast
        double py = getPos().y - 0.07 * offset.x / 500;  // fixed constant to avoid going too fast
        double pz = start_z    - 0.08 * (ros::Time::now() - start_time).toSec();

        prev_offset.x = px;
        prev_offset.y = py;

        goToPoseNoCheck(px,py,pz,VERTICAL_ORI_L);

        if(pz < -0.2)
        {
            ROS_WARN("I went too low! Exiting.");
            return false;
        }

        if(hasCollidedIR("loose")) break;
        r.sleep();
    }

    destroyCVWindows();
    gripObject();
    return true;
}

bool TTTController::computeTokenOffset(cv::Point &offset)
{
    Mat token(_img_size, CV_8UC1);
    Mat pool (_img_size, CV_8UC1);

    pool = detectPool();
    token = isolateToken(pool);

    bool res = false;

    ttt::Contours contours;
    vector<cv::Vec4i> hierarchy;
    findContours(token, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    token = Mat::zeros(_img_size, CV_8UC1);

    // when hand camera is blind due to being too close to token, go straight down;
    if(contours.size() < 2)
    {
        offset = cv::Point(0,0);
    }
    else if(contours.size() <= 4)
    {
        // find highest and lowest x and y values from token triangles contours
        // to find x-y coordinate of top left token edge and token side length
        int y_min = (contours[0])[0].y;
        int x_min = (contours[0])[0].x;
        int y_max = 0;
        int x_max = 0;

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
        rectangle(token, Rect(x_min, y_min, y_max - y_min, y_max - y_min),
                                          Scalar(255,255,255), CV_FILLED);

        // find and draw the center of the token and the image
        int x_mid = int((x_max + x_min) / 2);
        int y_mid = int((y_max + y_min) / 2);

        int x_trg = int(_img_size.width/2+20);   // some offset to center the tile on the gripper
        int y_trg = int(_img_size.height/2-40);  // some offset to center the tile on the gripper

        circle(token, cv::Point(x_mid, y_mid), 3, Scalar(0, 0, 0), CV_FILLED);
        circle(token, cv::Point(x_trg, y_trg), 3, Scalar(180, 40, 40), CV_FILLED);

        offset.x = x_mid - x_trg;
        offset.y = y_mid - y_trg;

        ROS_DEBUG_THROTTLE(1, "Offset %i %i", offset.x, offset.y);

        res=true;
    }

    imshow("Processed", token);
    waitKey(10);

    return res;
}

cv::Mat TTTController::detectPool()
{
    Mat black = Mat::zeros(_img_size, CV_8UC1);
    Mat   out = Mat::zeros(_img_size, CV_8UC1);

    isolateBlack(black);
    // imshow("black", black);

    vector<cv::Vec4i> hierarchy;
    ttt::Contours contours;

    // find outer board contours
    findContours(black, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    double largest = 0;
    int largest_index = 0;

    // iterate through contours and keeps track of contour w/ largest area
    for(int i = 0; i < contours.size(); i++)
    {
        if(contourArea(contours[i], false) > largest)
        {
            largest = contourArea(contours[i], false);
            largest_index = i;
        }
    }

    // contour w/ largest area is most likely the inner board
    vector<cv::Point> contour = contours[largest_index];

    drawContours(out, contours, largest_index, Scalar(255,255,255), CV_FILLED);
    return out;
}

Mat TTTController::isolateToken(Mat pool)
{
    Mat blue = Mat::zeros(_img_size, CV_8UC1);
    Mat out  = Mat::zeros(_img_size, CV_8UC1);

    isolateBlue(blue);    // imshow("blue", blue);

    bitwise_and(blue, pool, out);

    // Some morphological operations to remove noise and clean up the image
    for (int i = 0; i < 2; ++i) erode(out, out, Mat());
    for (int i = 0; i < 4; ++i) dilate(out, out, Mat());
    for (int i = 0; i < 2; ++i) erode(out, out, Mat());

    imshow("Rough", out);

    return out;
}

/**************************************************************************/
/*                               ScanBoard                                */
/**************************************************************************/

void TTTController::setDepth(float &dist)
{
    ROS_INFO("Computing depth..");

    geometry_msgs::Point init_pos = getPos();

    ros::Time start_time = ros::Time::now();

    // move downwards until collision with surface
    while(RobotInterface::ok())
    {
        double px = init_pos.x;
        double py = init_pos.y;
        double pz = init_pos.z - (ARM_SPEED+0.2) * (ros::Time::now() - start_time).toSec();

        double ox =   1.0;
        double oy = -0.03;
        double oz =   0.0;
        double ow =   0.0;

        goToPoseNoCheck(px,py,pz,ox,oy,oz,ow);

        if(hasCollidedIR("loose")) break;
        r.sleep();
    }

    // offset to account for height difference between IR camera and tip of vacuum gripper
    dist = init_pos.z - getPos().z + 0.04;
    ROS_INFO("Dist is %g", dist);
}

void TTTController::processImage(float dist)
{
    createCVWindows();
    while(RobotInterface::ok())
    {
        ttt::Contours contours;
        vector<cv::Point> centroids, board_corners, cell_to_corner;

        Mat binary, board;

        int board_area;

        isolateBlack(binary);
        isolateBoard(contours, board_area, board_corners, binary, board);

        waitKey(3);

        if(contours.size() == 9)
        {
            setOffsets(board_area, contours, dist, &board, &centroids);
            // imshow("[ScanBoard] Processed", board);

            if(offsetsReachable())
            {
                ROS_INFO_THROTTLE(2, "[Scan Board] Board is positioned correctly! Proceed with game\n");
                break;
            }
            else if(!offsetsReachable())
            {
                ROS_WARN("[Scan Board] Please move board within reachable zone\n");
                setZone(contours, dist, board_corners, centroids, &cell_to_corner);

                // calls to IK solver in setZone takes too long; makes the image update
                // very slow and hard for users to calibrate board position, which is why
                // and inner loop is needed
                ros::Time start = ros::Time::now();
                int interval = 10;
                while(RobotInterface::ok())
                {
                    Mat zone = _curr_img.clone();

                    line(zone, centroids[0] + cell_to_corner[0],
                               centroids[2] + cell_to_corner[1], cv::Scalar(0,0,255), 1);
                    line(zone, centroids[2] + cell_to_corner[1],
                               centroids[8] + cell_to_corner[3], cv::Scalar(0,0,255), 1);
                    line(zone, centroids[0] + cell_to_corner[0],
                               centroids[6] + cell_to_corner[2], cv::Scalar(0,0,255), 1);
                    line(zone, centroids[6] + cell_to_corner[2],
                               centroids[8] + cell_to_corner[3], cv::Scalar(0,0,255), 1);

                    if((ros::Time::now() - start).toSec() > interval)
                    {
                        vector<cv::Point> temp_centroids, temp_board_corners;
                        isolateBlack(binary);
                        isolateBoard(contours, board_area, temp_board_corners, binary.clone(), board);
                        if(contours.size() == 9)
                        {
                            setOffsets(board_area, contours, dist, &board, &temp_centroids);
                        }
                        if(offsetsReachable())
                        {
                            break;
                        }
                        interval += 5;
                    }

                    imshow("Rough", zone);
                    waitKey(3);
                    r.sleep();
                }
            }
        }

        imshow("Processed", binary);
    }
    destroyCVWindows();
}

void TTTController::isolateBlack(Mat &output)
{
    Mat gray;
    pthread_mutex_lock(&_mutex_img);
    cvtColor(_curr_img, gray, CV_BGR2GRAY);
    pthread_mutex_unlock(&_mutex_img);
    threshold(gray, output, 55, 255, cv::THRESH_BINARY);
}

void TTTController::isolateBlue(Mat &output)
{
    Mat hsv(_img_size, CV_8UC1);

    pthread_mutex_lock(&_mutex_img);
    cvtColor(_curr_img, hsv, CV_BGR2HSV);
    pthread_mutex_unlock(&_mutex_img);

    output = hsvThreshold(hsv, hsv_blue);
}

void TTTController::isolateBoard(ttt::Contours &contours, int &board_area,
                                 vector<cv::Point> &board_corners, Mat input, Mat &output)
{
    output = Mat::zeros(_img_size, CV_8UC1);

    vector<cv::Vec4i> hierarchy; // captures contours within contours

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

    board_area = contourArea(contours[next_largest_index], false);

    drawContours(output, contours, next_largest_index, Scalar(255,255,255), CV_FILLED, 8, hierarchy);

    findContours(output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    largest = 0;
    largest_index = 0;

    // iterate through contours and keeps track of contour w/ largest area
    for(int i = 0; i < contours.size(); i++)
    {
        if(contourArea(contours[i], false) > largest)
        {
            largest = contourArea(contours[i], false);
            largest_index = i;
        }
    }

    vector<cv::Point> board_outline = contours[largest_index];

    /* Set board corners and board area*/
    double y_min = board_outline[0].y;
    double x_min = board_outline[0].x;
    double y_max = 0;
    double x_max = 0;

    for(int i = 0; i < board_outline.size(); i++)
    {
        if(y_min > board_outline[i].y) y_min = board_outline[i].y;
        if(x_min > board_outline[i].x) x_min = board_outline[i].x;
        if(y_max < board_outline[i].y) y_max = board_outline[i].y;
        if(x_max < board_outline[i].x) x_max = board_outline[i].x;
    }

    board_corners.push_back(cv::Point(x_max, y_max));
    board_corners.push_back(cv::Point(x_min, y_max));
    board_corners.push_back(cv::Point(x_max, y_min));
    board_corners.push_back(cv::Point(x_min, y_min));

    // remove outer board contours
    contours.erase(contours.begin() + largest_index);

    for(int i = 0; i < contours.size(); i++)
    {
        if(contourArea(contours[i], false) < 200)
        {
            contours.erase(contours.begin() + i);
        }
    }

    for(int i = 0; i < contours.size(); i++)
    {
        drawContours(output, contours, i, Scalar(255,255,255), CV_FILLED);
    }
}

bool TTTController::descendingX(vector<cv::Point> i, vector<cv::Point> j)
{
    double x_i = moments(i, false).m10 / moments(i, false).m00;
    double x_j = moments(j, false).m10 / moments(j, false).m00;

    return x_i > x_j;
}

void TTTController::setOffsets(int board_area, ttt::Contours contours, float dist, Mat *output, vector<cv::Point> *centroids)
{
    cv::Point center(_img_size.width / 2, _img_size.height / 2);

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
        double x = moments(contours[i], false).m10 / moments(contours[i], false).m00;
        double y = moments(contours[i], false).m01 / moments(contours[i], false).m00;
        cv::Point centroid(x,y);

        (*centroids)[i] = centroid;

        // cv::putText(*output, intToString(i), centroid, cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(180,40,40));
        // circle(*output, centroid, 2, Scalar(180,40,40), CV_FILLED);
        line(*output, centroid, center, cv::Scalar(180,40,40), 1);

        _offsets[i].x = (centroid.y - center.y) * 0.0025 * dist + 0.04;
        _offsets[i].y = (centroid.x - center.x) * 0.0025 * dist;
        _offsets[i].z = dist - 0.065;
    }
}

void TTTController::setZone(ttt::Contours contours, float dist, vector<cv::Point> board_corners,
                            vector<cv::Point> c, vector<cv::Point> * cell_to_corner)
{
    (*cell_to_corner).resize(4);

    // calculate offset between the center of corner cells and the corners of the board
    (*cell_to_corner)[0] = cv::Point(board_corners[0].x - c[0].x, board_corners[0].y - c[0].y);
    (*cell_to_corner)[1] = cv::Point(board_corners[1].x - c[2].x, board_corners[1].y - c[2].y);
    (*cell_to_corner)[2] = cv::Point(board_corners[2].x - c[6].x, board_corners[2].y - c[6].y);
    (*cell_to_corner)[3] = cv::Point(board_corners[3].x - c[8].x, board_corners[3].y - c[8].y);

    // if the centroid of a corner cell is reachable,
    // iterate and check if a location 10 pixels further from arm is still reachable
    // to establish a boundary of how far Baxter's arm can reach
    while(pointReachable(c[0], dist)) {c[0].x += 10.0;}
    while(pointReachable(c[2], dist)) {c[2].x -= 10.0;}
    while(pointReachable(c[6], dist)) {c[6].x += 10.0;}
    while(pointReachable(c[8], dist)) {c[8].x -= 10.0;}

    // if the centroid of a corner cell is unreachable,
    // iterate and check if a location 10 pixels closer is reachable
    while(!pointReachable(c[0], dist)) {c[0].x -= 5.0;}
    while(!pointReachable(c[2], dist)) {c[2].x += 5.0;}
    while(!pointReachable(c[6], dist)) {c[6].x -= 5.0;}
    while(!pointReachable(c[8], dist)) {c[8].x += 5.0;}
}

bool TTTController::offsetsReachable()
{
    for(int i = 0; i < NUMBER_OF_CELLS; i++)
    {
        double px = getPos().x + _offsets[i].x;
        double py = getPos().y + _offsets[i].y;
        double pz = getPos().z - _offsets[i].z;

        vector<double> joint_angles;
        if (!computeIK(px,py,pz,VERTICAL_ORI_L,joint_angles))
        {
            ROS_INFO("Offset number %i not reachable", i);
            return false;
        }
    }
    return true;
}

bool TTTController::pointReachable(cv::Point centroid, float dist)
{
    // convert image location into real world pose coordinates
    cv::Point center(_img_size.width / 2, _img_size.height / 2);

    geometry_msgs::Point offset;

    offset.x = (centroid.y - center.y) * 0.0025 * dist + 0.04;
    offset.y = (centroid.x - center.x) * 0.0025 * dist;
    offset.z = dist - 0.085;

    double px = HOVER_BOARD_X + offset.x;
    double py = HOVER_BOARD_Y + offset.y;
    double pz = HOVER_BOARD_Z - offset.z;

    vector<double> joint_angles;
    return computeIK(px,py,pz,VERTICAL_ORI_L,joint_angles);
}

/**************************************************************************/
/*                               MISC                                     */
/**************************************************************************/

bool TTTController::createCVWindows()
{
    namedWindow("Hand Camera", WINDOW_NORMAL);
    namedWindow("Rough", WINDOW_NORMAL);
    namedWindow("Processed", WINDOW_NORMAL);
    resizeWindow("Hand Camera", 480, 300);
    resizeWindow("Rough",   480, 300);
    resizeWindow("Processed",   480, 300);
    moveWindow("Hand Camera", 10, 10);
    moveWindow("Rough", 10, 370);
    moveWindow("Processed", 10, 720);
    waitKey(10);
}

bool TTTController::destroyCVWindows()
{
    destroyWindow("Hand Camera");
    destroyWindow("Processed");
    destroyWindow("Rough");
}

bool TTTController::goHome()
{
    return ArmCtrl::goHome();
}

void TTTController::setHomeConfiguration()
{
    if (getLimb() == "left")
    {
        setHomeConf( 0.688, -0.858, -1.607, 1.371, 0.742, 1.733, 0.007);
        // setHomeConf( 0.022, -0.503, -2.071, 1.466,` 1.075, 2.065, -0.915);
    }
    else if (getLimb() == "right")
    {
        setHomeConf( -1.332, -0.579, 0.143, 2.270, -1.995, -1.469, -0.012);
    }
}

bool TTTController::startAction(string a, int o)
{
    baxter_collaboration_msgs::DoAction::Request  req;
    baxter_collaboration_msgs::DoAction::Response res;
    req.action = a;
    req.objects.push_back(o);

    serviceCb(req,res);
    return res.success;
}

bool TTTController::hoverAboveBoard()
{
    ROS_INFO("Hovering above board..");
    // return goToPose(HOVER_BOARD_X, 0.220, HOVER_BOARD_Z, 0.0,  1.0,  0.0,  0.0);
    return goToPose(HOVER_BOARD_X, HOVER_BOARD_Y, HOVER_BOARD_Z, 1.0, -0.03, 0, 0);
}

bool TTTController::hoverAboveCenterOfBoard()
{
    ROS_INFO("Hovering above center of board..");
    return goToPose(HOVER_BOARD_X + _offsets[4].x,
                    HOVER_BOARD_Y + _offsets[4].y,
                    HOVER_BOARD_Z - _offsets[4].z + 0.3,    // TODO this minus sign is a bug
                    VERTICAL_ORI_L);
}

bool TTTController::hoverAboveCell()
{
    ROS_INFO("Hovering above cell..");
    return goToPose(HOVER_BOARD_X + _offsets[getObjectID() - 1].x,
                    HOVER_BOARD_Y + _offsets[getObjectID() - 1].y,
                    HOVER_BOARD_Z - _offsets[getObjectID() - 1].z + 0.05,
                    VERTICAL_ORI_L);
}

bool TTTController::hoverAboveTokens(double height)
{
    return goToPose(0.540, 0.570, height, VERTICAL_ORI_L);
}

bool TTTController::scanBoardImpl()
{
    ROS_INFO("Scanning depth..");
    if (!hoverAboveBoard()) return false;

    // wait for image callback
    while(RobotInterface::ok())
    {
        if(!_is_img_empty) break;

        r.sleep();
    }

    float dist;
    setDepth(dist);
    if (!hoverAboveBoard()) return false;
    processImage(dist);

    ROS_INFO("Hovering above tokens..");
    hoverAboveTokens(Z_HIGH);

    return true;
}

bool TTTController::pickUpTokenImpl()
{
    ROS_INFO("Picking up token..");
    setTracIK(true);

    while(RobotInterface::ok())
    {
        if(is_ir_ok()) break;
        r.sleep();
    }

    // wait for image callback
    while(RobotInterface::ok())
    {
        if(!_is_img_empty) break;
        r.sleep();
    }

    hoverAboveTokens(Z_HIGH);
    gripToken();
    hoverAboveTokens(Z_LOW);

    setTracIK(false);

    return true;
}

bool TTTController::putDownTokenImpl()
{
    ROS_INFO("Putting down token..");
    if (!hoverAboveCenterOfBoard()) return false;
    if (!hoverAboveCell()) return false;
    ros::Duration(0.1).sleep();
    if (!releaseObject()) return false;
    if (!hoverAboveCenterOfBoard()) return false;
    hoverAboveTokens(Z_HIGH);

    return true;
}

void TTTController::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("[TTT Controller] cv_bridge exception: %s", e.what());
        return;
    }

    pthread_mutex_lock(&_mutex_img);
    _curr_img     = cv_ptr->image.clone();
    _img_size     =      _curr_img.size();
    _is_img_empty =     _curr_img.empty();
    imshow("Hand Camera", _curr_img.clone());
    pthread_mutex_unlock(&_mutex_img);
}

TTTController::~TTTController()
{
    destroyCVWindows();

    pthread_mutex_destroy(&_mutex_img);
}
