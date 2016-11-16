#include "ttt_controllers/ttt_controllers.h"
#include <kdl/chainiksolverpos_nr_jl.hpp>

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace cv;

/*
    BoardStateSensing error at start
    Drop token faster
    Flag option for turning off display windows when run as part of baxterTictactoe
    Get rid of imageScreen node error and show something else via boardScheme
    Error-checking when cellsDefinitionAuto does not see board
*/

/**************************************************************************/
/*                         PickUpToken                                    */
/**************************************************************************/
void TTTController::gripToken()
{
    cv::Point2d offset;
    // check if token is present before starting movement loop
    // (prevent gripper from colliding with play surface)
    checkForToken(offset);

    ros::Time start_time = ros::Time::now();
    cv::Point2d prev_offset(0.540, 0.540);

    while(RobotInterface::ok())
    {
        processTokenImage(offset);
        ros::Time now_time = ros::Time::now();

        // move incrementally towards token
        double px = prev_offset.x + 0.07 * offset.x;
        double py = prev_offset.y + 0.07 * offset.y;
        double pz = 0.375 + /*(-0.05)*/ -0.08 * (now_time - start_time).toSec();

        prev_offset.x = prev_offset.x + 0.07 * offset.x;
        prev_offset.y = prev_offset.y + 0.07 * offset.y;

        vector<double> joint_angles;
        computeIK(px,py,pz,VERTICAL_ORI_L,joint_angles);
        goToPoseNoCheck(joint_angles);

        r.sleep();

        // if(getPos().z < -0.05) break;

        if(hasCollided("strict"))
        {
            break;
        }
    }
    gripObject();
}

void TTTController::checkForToken(cv::Point2d &offset)
{
    ros::Time start_time = ros::Time::now();

    while(RobotInterface::ok())
    {
        processTokenImage(offset);

        if(!(offset.x == 0 && offset.y == 0) || (ros::Time::now() - start_time).toSec() > 1)
        {
            break;
        }
    }

    while(RobotInterface::ok())
    {
        if(!(offset.x == 0 && offset.y == 0)) break;

        ROS_WARN("No token detected by hand camera.");
        r.sleep();
        processTokenImage(offset);
    }
}

void TTTController::processTokenImage(cv::Point2d &offset)
{
    Mat black, blue, token_rough, token, board;
    Contours contours;
    int board_y;

    isolateBlack(black);
    isolateTokenBoard(black, board, board_y);
    imshow("black", black);

    isolateBlue(blue);
    imshow("Rough", blue);
    isolateToken(blue, board_y, token_rough, contours);
    setTokenOffset(contours, offset, token);

    imshow("Processed", token_rough);
}

void TTTController::isolateBlue(Mat &output)
{
    Mat hsv;

    pthread_mutex_lock(&_mutex_img);
    cvtColor(_curr_img, hsv, CV_BGR2HSV);
    pthread_mutex_unlock(&_mutex_img);

    inRange(hsv, Scalar(60,90,10), Scalar(130,256,256), output);
}

void TTTController::isolateTokenBoard(Mat input, Mat &output, int &board_y)
{
    output = Mat::zeros(_img_size, CV_8UC1);

    vector<cv::Vec4i> hierarchy; // captures contours within contours
    Contours contours;

    // find outer board contours
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

    output = Mat::zeros(_img_size, CV_8UC1);

    // contour w/ 2nd largest area is most likely the inner board
    vector<cv::Point> contour = contours[next_largest_index];

    drawContours(output, contours, next_largest_index, Scalar(255,255,255), CV_FILLED);

    // find the lowest y-coordinate of the board; to be used as a cutoff point above which
    // all contours are ignored (e.g token contours that are above low_y are already placed
    // on the board and should not be picked up)
    int low_y = _img_size.height;
    int x_min = (contours[0])[0].x;
    int x_max = 0;

    for(int i = 0; i < contour.size(); i++)
    {
        if(contour[i].y < low_y) low_y = contour[i].y;
        if(contour[i].x < x_min) x_min = contour[i].x;
        if(contour[i].x > x_max) x_max = contour[i].x;
    }

    // if width of the contour is narrower than 275, 2nd largest contour
    // is NOT the board (and board is out of the image's view). Hence,
    // no cutoff point needs to be specified
    if(x_max - x_min > 275) {
        board_y = low_y;
    }
    else
    {
        board_y = _img_size.height;
    }

    line(output, cv::Point(0, board_y), cv::Point(_img_size.width, board_y), cv::Scalar(130,256,256), 5);
}

void TTTController::isolateToken(Mat input, int board_y, Mat &output, Contours &contours)
{
    Contours raw_contours, clean_contours, apx_contours, gripper_contours;
    findContours(input, raw_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    int gripper_area = -1;
    int gripper_index = 0;

    // find gripper contours. gripper contours are always attached to bottom part of image
    // (Note that imshow will show the bottom (x=0) part inverted and
    // on top of the display window). if there are multiple contours that contain points w/ x=0
    // the gripper contour is the contour with the largest area as a combination of the contours
    // of the gripper AND  a token fragment will always be larger than just a token fragment
    for(int i = 0; i < raw_contours.size(); i++)
    {
        vector<cv::Point> contour = raw_contours[i];
        for(int j = 0; j < contour.size(); j++)
        {
            if(contour[j].y == 1)
            {
                if(gripper_area == -1)
                {
                    gripper_area = contourArea(contour, false);
                    gripper_index = i;
                }
                else if(contourArea(contour, false) > gripper_area)
                {
                    gripper_area = contourArea(contour, false);
                    gripper_index = i;
                }
                break;
            }
        }
    }

    // remove gripper contour
    raw_contours.erase(raw_contours.begin() + gripper_index);

    // remove contours that have areas that are too small (noise) and
    // contours that do not have an approx. triangular shape (not token fragment)
    int largest_index = 0, largest_area = 0;
    for(int i = 0; i < raw_contours.size(); i++)
    {
        bool is_triangle = true;
        vector<cv::Point> contour;
        approxPolyDP(raw_contours[i], contour, 0.11 * arcLength(raw_contours[i], true), true);

        if(contour.size() != 3) is_triangle = false;

        if(contourArea(raw_contours[i]) > 200 && is_triangle == true)
        {
            apx_contours.push_back(contour);
            clean_contours.push_back(raw_contours[i]);
        }
    }

    // remove contours that are inside the board (e.g token placed on a cell)
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
            (contours).push_back(contour);
        }
    }

    output = Mat::zeros(_img_size, CV_8UC1);
    for(int i = 0; i < (contours).size(); i++)
    {
        drawContours(output, contours, i, Scalar(255,255,255), CV_FILLED);
    }

    line(output, cv::Point(0, board_y), cv::Point(_img_size.width, board_y), cv::Scalar(130,256,256));
}

void TTTController::setTokenOffset(Contours contours, cv::Point2d &offset, Mat &output)
{
    output = Mat::zeros(_img_size, CV_8UC1);

    // when hand camera is blind due to being too close to token, go straight down;
    if(contours.size() < 2)
    {
        offset = cv::Point2d(0,0);
    }
    else if(contours.size() <= 4)
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
        rectangle(output, token, Scalar(255,255,255), CV_FILLED);

        // find and draw the center of the token and the image
        double x_mid = x_min + ((x_max - x_min) / 2);
        double y_mid = y_min + ((y_max - y_min) / 2);
        circle(output, cv::Point(x_mid, y_mid), 3, Scalar(0, 0, 0), CV_FILLED);

        circle(output, cv::Point(_img_size.width / 2, _img_size.height / 2), 3, Scalar(180, 40, 40), CV_FILLED);

        double token_area = (x_max - x_min) * (y_max - y_min);

        (offset).x = (/*4.7807*/ 5 / token_area) * (x_mid - (_img_size.width / 2));
        // distance between gripper center and camera center
        (offset).y = (/*4.7807*/ 5 / token_area) * ((_img_size.height / 2) - y_mid) - 0.0075;
    }
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
        double pz = init_pos.z + (-0.07) * (ros::Time::now() - start_time).toSec();

        double ox =   1.0;
        double oy = -0.03;
        double oz =   0.0;
        double ow =   0.0;

        vector<double> joint_angles;
        computeIK(px,py,pz,ox,oy,oz,ow, joint_angles);
        goToPoseNoCheck(joint_angles);
        r.sleep();

        if(hasCollided("loose"))
        {
            break;
        }
    }

    // offset to account for height difference between IR camera and tip of vacuum gripper
    dist = init_pos.z - getPos().z + 0.04;
    ROS_INFO("Dist is %g", dist);
}

void TTTController::processImage(string mode, float dist)
{
    while(RobotInterface::ok())
    {
        Contours contours;
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

            if(offsetsReachable() && mode == "run"){
                ROS_INFO("[Scan Board] Board is positioned correctly! Proceed with game\n");
                break;
            }
            else if(!offsetsReachable()) {
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
                }
            }
        }

        imshow("Processed", binary);
    }
}

void TTTController::isolateBlack(Mat &output)
{
    Mat gray;
    pthread_mutex_lock(&_mutex_img);
    cvtColor(_curr_img, gray, CV_BGR2GRAY);
    pthread_mutex_unlock(&_mutex_img);
    threshold(gray, output, 55, 255, cv::THRESH_BINARY);
}

void TTTController::isolateBoard(Contours &contours, int &board_area,
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

void TTTController::setOffsets(int board_area, Contours contours, float dist, Mat *output, vector<cv::Point> *centroids)
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

void TTTController::setZone(Contours contours, float dist, vector<cv::Point> board_corners,
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
    for(int i = 0; i < 9; i++)
    {
        double px = getPos().x + _offsets[i].x;
        double py = getPos().y + _offsets[i].y;
        double pz = getPos().z - _offsets[i].z;

        vector<double> joint_angles;
        if (!computeIK(px,py,pz,VERTICAL_ORI_L,joint_angles)) return false;

        // // if IK solver returns a joint angles solution with all zeros,
        // // then no solution was found
        // bool all_zeros = true;
        // for(int j = 0; j < joint_angles.size(); j++)
        // {
        //     if(joint_angles[j] != 0)
        //     {
        //         all_zeros = false;
        //         break;
        //     }
        // }
        // if(all_zeros) return false;
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
/*                            TTTController                               */
/**************************************************************************/

TTTController::TTTController(string name, string limb, bool no_robot, bool use_forces):
                             ArmCtrl(name, limb, no_robot, use_forces, false),
                             r(100), _img_trp(_n), _is_img_empty(true)
{
    namedWindow("Hand Camera", WINDOW_NORMAL);
    namedWindow("Processed", WINDOW_NORMAL);
    namedWindow("Rough", WINDOW_NORMAL);
    resizeWindow("Hand Camera", 700, 500);
    resizeWindow("Processed",   700, 500);
    resizeWindow("Rough",       700, 500);
    waitKey(10);

    setHomeConfiguration();

    insertAction(ACTION_SCAN,    static_cast<f_action>(&TTTController::scanBoardImpl));
    insertAction(ACTION_PICKUP,  static_cast<f_action>(&TTTController::pickUpTokenImpl));
    insertAction(ACTION_PUTDOWN, static_cast<f_action>(&TTTController::putDownTokenImpl));

    if (getLimb() == "left")
    {
        _img_sub = _img_trp.subscribe("/cameras/"+getLimb()+"_hand_camera/image",
                               SUBSCRIBER_BUFFER, &TTTController::imageCb, this);
    }
    pthread_mutex_init(&_mutex_img, NULL);

    KDL::JntArray ll, ul; //lower joint limits, upper joint limits
    getIKLimits(ll,ul);

    // double s1l = -1.1;
    // double s1u =  1.0;
    // ROS_INFO("[%s] Setting custom joint limits for %s_s1: [%g %g]", getLimb().c_str(), getLimb().c_str(), s1l, s1u);
    // ll.data[1] =  s1l;
    // ul.data[1] =  s1u;
    // setIKLimits(ll,ul);
    // getIKLimits(ll,ul);

    // printf("ll.rows cols %i %i\t", ll.rows(), ll.columns());
    // for (int i = 0; i < ll.rows(); ++i)
    // {
    //     printf("%g\t", ll.data[i]);
    // }
    // printf("\n");

    // printf("ul.rows cols %i %i\t", ul.rows(), ul.columns());
    // for (int i = 0; i < ul.rows(); ++i)
    // {
    //     printf("%g\t", ul.data[i]);
    // }
    // printf("\n");

    if (!callAction(ACTION_HOME)) setState(ERROR);
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
    baxter_collaboration::DoAction::Request  req;
    baxter_collaboration::DoAction::Response res;
    req.action = a;
    req.object = o;

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
                    HOVER_BOARD_Z - _offsets[4].z + 0.1,    // TODO this minus sign is a bug
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
    processImage("run", dist);

    ROS_INFO("Hovering above tokens..");
    hoverAboveTokens(Z_HIGH);

    return true;
}

bool TTTController::pickUpTokenImpl()
{
    ROS_INFO("Picking up token..");
    // wait for IR sensor callback
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

    return true;
}

bool TTTController::putDownTokenImpl()
{
    ROS_INFO("Putting down token..");
    if (!hoverAboveCenterOfBoard()) return false;
    if (!hoverAboveCell()) return false;
    ros::Duration(0.5).sleep();
    if (!releaseObject()) return false;
    if (!hoverAboveCenterOfBoard()) return false;
    if (!hoverAboveTokens(Z_HIGH)) return false;

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
    pthread_mutex_destroy(&_mutex_img);

    destroyWindow("Hand Camera");
    destroyWindow("Processed");
    destroyWindow("Rough");
}
