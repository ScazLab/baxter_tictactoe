#include "boardState.h"

using namespace ttt;
using namespace baxter_tictactoe;
using namespace std;

bool operator==(MsgBoard board1, MsgBoard board2)
{
    if (board1.cells.size()!=board2.cells.size()) return false;

    for (size_t i = 0; i < board1.cells.size(); ++i) {
        if (board1.cells[i].state!=board2.cells[i].state) {
            return false;
        }
    }
    return true;
}

bool operator!=(MsgBoard board1, MsgBoard board2)
{
    return !(board1==board2);
}

BoardState::BoardState(string name, bool _show) :
                       ROSThreadImage(name), doShow(_show), board_state(STATE_INIT), r(40) // 40Hz
{
    board_state_pub = _n.advertise<MsgBoard>("/baxter_tictactoe/board_state", 1);
    brain_state_sub = _n.subscribe("/baxter_tictactoe/ttt_brain_state", SUBSCRIBER_BUFFER,
                                   &BoardState::brainStateCb, this);
    img_pub         = _img_trp.advertise("/baxter_tictactoe/board_state_img", 1);

    XmlRpc::XmlRpcValue hsv_red_symbols;
    ROS_ASSERT_MSG(_n.getParam("hsv_red",hsv_red_symbols), "No HSV params for RED!");
    hsv_red=hsvColorRange(hsv_red_symbols);

    XmlRpc::XmlRpcValue hsv_blue_symbols;
    ROS_ASSERT_MSG(_n.getParam("hsv_blue",hsv_blue_symbols), "No HSV params for BLUE!");
    hsv_blue=hsvColorRange(hsv_blue_symbols);

    ROS_ASSERT_MSG(_n.getParam("area_threshold",area_threshold), "No area threshold!");
    for(int i = 0; i < last_msg_board.cells.size(); i++)
    {
        last_msg_board.cells[i].state = undefined;
    }

    col_red   = cv::Scalar(  40,  40, 150);  // BGR color code
    col_empty = cv::Scalar(  60, 160,  60);
    col_blue  = cv::Scalar( 180,  40,  40);

    ROS_INFO("Red  tokens in\t%s", hsv_red.toString().c_str());
    ROS_INFO("Blue tokens in\t%s", hsv_blue.toString().c_str());
    ROS_INFO("Area threshold: %g", area_threshold);
    ROS_INFO("Show param set to %i", doShow);

    if (doShow)
    {
        cv::namedWindow("[Board_State_Sensor] cell outlines");
        cv::namedWindow("[Board_State_Sensor] red  mask of the board");
        cv::namedWindow("[Board_State_Sensor] blue mask of the board");
    }
}

void BoardState::InternalThreadEntry()
{
    while(ros::ok())
    {
        cv::Mat img_in;
        cv::Mat img_out;
        if (!_img_empty)
        {
            pthread_mutex_lock(&_mutex_img);
            img_in=_curr_img;
            pthread_mutex_unlock(&_mutex_img);
            img_out = img_in.clone();
        }

        if (board_state == STATE_INIT)
        {
            ROS_INFO_THROTTLE(1,"[%i] Initializing..", board_state);
            if (brain_state == TTTBrainState::READY) ++board_state;
        }
        else if (board_state == STATE_CALIB && !ros::isShuttingDown())
        {
            ROS_INFO_THROTTLE(1,"[%i] Calibrating board..", board_state);
            if (!_img_empty)
            {
                cv::Mat img_gray;
                cv::Mat img_binary;

                // convert image color model from BGR to grayscale
                cv::cvtColor(img_in, img_gray, CV_BGR2GRAY);

                // convert grayscale image to binary image, using 155 threshold value to
                // isolate white-colored board
                cv::threshold(img_gray, img_binary, 70, 255, cv::THRESH_BINARY);

                // a contour is an array of x-y coordinates describing the boundaries of an object
                vector<vector<cv::Point> > contours;
                // Vec4i = vectors w/ 4 ints
                vector<cv::Vec4i> hierarchy;

                // find white edges of outer board by finding contours (i.e boundaries)
                cv::findContours(img_binary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

                // isolate contour w/ the largest area to separate outer board from other objects in
                // image (assuming outer board is largest object in image)
                int largest_idx = getIthIndex(contours, LARGEST_IDX);

                // draw outer board contour (i.e boundaries) onto zero matrix (i.e black image)
                cv::Mat outer_board = cv::Mat::zeros(img_binary.size(), CV_8UC1);
                drawContours(outer_board, contours, largest_idx,
                             cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy);

                // find black edges of inner board by finding contours
                cv::findContours(outer_board, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

                // isolate inner board contour by finding contour w/ second largest area (given that
                // outer board contour has the largest area)
                int next_largest_idx = getIthIndex(contours, NEXT_LARGEST_IDX);

                // draw inner board contour onto zero matrix
                cv::Mat inner_board = cv::Mat::zeros(outer_board.size(), CV_8UC1);
                drawContours(inner_board, contours, next_largest_idx,
                             cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy);

                cv::findContours(inner_board, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

                largest_idx = getIthIndex(contours, LARGEST_IDX);

                // drawn board cells onto zero matrix by drawing all contour except the
                // the largest-area contour (which is the inner board contour)
                cv::Mat board_cells = cv::Mat::zeros(inner_board.size(), CV_8UC1);

                // find and display cell
                if(contours.size() == NUMBER_OF_CELLS + 1)
                {
                    contours.erase(contours.begin() + largest_idx);
                    board.resetState();

                    // aproximate cell contours to quadrilaterals
                    vector<vector<cv::Point> > apx_contours;
                    for(int i = 0; i < contours.size(); i++)
                    {
                        double epsilon = arcLength(contours[i], true);
                        vector<cv::Point> apx_cell;
                        approxPolyDP(contours[i], apx_cell, 0.1 * epsilon, true);
                        apx_contours.push_back(apx_cell);
                    }

                    for(int i = 0; i < apx_contours.size(); i++)
                    {
                        drawContours(board_cells, apx_contours, i, cv::Scalar(255,255,255), CV_FILLED, 8);
                    }

                    // sort cell_contours in descending order of y-coordinate (not needed as test shows
                    // that findContours apparently finds contours in ascending order of y-coordinate already)
                    std::sort(apx_contours.begin(), apx_contours.end(), ascendingY);

                    for(int i = 0; i <=6; i += 3)
                    {
                        std::sort(apx_contours.begin() + i, apx_contours.begin() + i + 3, ascendingX);
                    }

                    for(int i = 0; i < apx_contours.size(); i++)
                    {
                        Cell cell(apx_contours[i]);
                        board.cells.push_back(cell);
                    }

                    if(doShow) cv::imshow("[Cells_Definition] cell boundaries", board_cells);
                    cv::waitKey(3);
                    ++board_state;
                }
            }
        }
        else if (board_state == STATE_READY && !ros::isShuttingDown())
        {
            ROS_INFO_THROTTLE(1, "[%i] Detecting Board State..", board_state);
            if (!_img_empty)
            {
                board.resetState();
                cv::Mat img_copy = img_in.clone();

                if (board.cells_size() == NUMBER_OF_CELLS)
                {
                    MsgBoard msg_board;
                    for(int i = 0; i < msg_board.cells.size(); i++)
                    {
                        msg_board.cells[i].state = MsgCell::EMPTY;
                    }
                    // msg_board.header.stamp = msg->header.stamp;
                    // msg_board.header.frame_id = msg->header.frame_id;

                    // convert the original image to hsv color space
                    cv::Mat img_hsv;
                    cv::cvtColor(img_copy,img_hsv,CV_BGR2HSV);

                    // mask the original image to the board
                    cv::Mat img_hsv_mask = board.mask_image(img_hsv);

                    for (int i = 0; i < 2; ++i)
                    {
                        cv::Mat hsv_filt_mask = hsv_threshold(img_hsv_mask, i==0?hsv_red:hsv_blue);
                        if (doShow)
                        {
                            if (i==0) cv::imshow("[Board_State_Sensor] red  mask of the board", hsv_filt_mask);
                            if (i==1) cv::imshow("[Board_State_Sensor] blue mask of the board", hsv_filt_mask);
                        }
                        for (int j = 0; j < board.cells_size(); ++j)
                        {
                            Cell *cell = &(board.cells[j]);
                            cv::Mat crop = cell->mask_image(hsv_filt_mask);

                            /* we smooth the image to reduce the noise */
                            cv::GaussianBlur(crop.clone(),crop,cv::Size(3,3),0,0);

                            // the area formed by the remaining pixels is computed based on the moments
                            double cell_area=cv::moments(crop,true).m00;

                            if (cell_area > area_threshold)
                            {
                                if (i==0)  cell->cell_area_red =cell_area;
                                else       cell->cell_area_blue=cell_area;
                            }
                        }
                    }

                    cv::Mat bg = cv::Mat::zeros(img_hsv.size(), CV_8UC1);
                    for(int i = 0; i < board.cells_size(); i++)
                    {
                        vector<vector<cv::Point> > contours;
                        contours.push_back(board.cells[i].contours);
                        drawContours(bg, contours, -1, cv::Scalar(255,255,255), CV_FILLED, 8);
                    }

                    for (int j = 0; j < board.cells_size(); ++j)
                    {
                        Cell *cell = &(board.cells[j]);
                        if (cell->cell_area_red || cell->cell_area_blue)
                        {
                            cell->cell_area_red>cell->cell_area_blue?cell->state=red:cell->state=blue;
                        }

                        msg_board.cells[j].state=cell->state;
                    }

                    board_state_pub.publish(msg_board);
                    last_msg_board=msg_board;
                    // ROS_INFO("New board state published");

                    for(int i = 0; i < board.cells_size(); i++)
                    {
                        cv::Point cell_centroid;
                        board.cells[i].get_cell_centroid(cell_centroid);
                        //cv::circle(img_out, p,5,cv::Scalar(0,0, 255),-1);
                        // cv::line(img_out, cell_centroid, cell_centroid, cv::Scalar(255,255,0), 2, 8);
                        cv::Scalar col = col_empty;
                        if (board.cells[i].state ==  red)  col = col_red;
                        if (board.cells[i].state == blue) col = col_blue;

                        ttt::Contours contours;
                        contours.push_back(board.cells[i].get_contours());

                        cv::drawContours(img_out, contours,-1, col, CV_FILLED); // drawing just the borders
                        cv::putText(img_out, intToString(i+1), cell_centroid, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar::all(255), 2);
                    }

                    if (doShow) cv::imshow("[Board_State_Sensor] cell outlines", img_out);
                    cv::waitKey(10);
                }
            }
        }

        if (!_img_empty)
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_out).toImageMsg();
            img_pub.publish(msg);
        }
        r.sleep();
    }
    ROS_INFO("[%i] Finishing", board_state);
}

void BoardState::brainStateCb(const baxter_tictactoe::TTTBrainState & msg)
{
    // ROS_INFO("[%i] brainStateCb %i", board_state, msg.state);
    brain_state = msg.state;

    if (msg.state == TTTBrainState::GAME_FINISHED)
    {
        board_state = STATE_INIT;
    }
}

int BoardState::getIthIndex(vector<vector<cv::Point> > contours, int ith)
{
    double largest_area = 0;
    int largest_idx = 0;
    double next_largest_area = 0;
    int next_largest_idx = 0;

    // iterate through contours and keeps track of contour w/ largest and 2nd-largest area
    for(int i = 0; i < contours.size(); i++)
    {
        if(contourArea(contours[i], false) > largest_area)
        {
            next_largest_area = largest_area;
            next_largest_idx = largest_idx;
            largest_area = contourArea(contours[i], false);
            largest_idx = i;
        }
        else if(next_largest_area < contourArea(contours[i], false) &&
                contourArea(contours[i], false) < largest_area)
        {
            next_largest_area = contourArea(contours[i], false);
            next_largest_idx = i;
        }
    }

    return ith==LARGEST_IDX?largest_idx:next_largest_idx;
}

std::string BoardState::intToString( const int a )
{
    stringstream ss;
    ss << a;
    return ss.str();
}

bool BoardState::ascendingY(vector<cv::Point> i, vector<cv::Point> j)
{
    double y_i = moments(i, false).m01 / moments(i, false).m00;
    double y_j = moments(j, false).m01 / moments(j, false).m00;

    return y_i < y_j;
}

bool BoardState::ascendingX(vector<cv::Point> i, vector<cv::Point> j)
{
    double x_i = moments(i, false).m10 / moments(i, false).m00;
    double x_j = moments(j, false).m10 / moments(j, false).m00;

    return x_i < x_j;
}

BoardState::~BoardState()
{
    if (doShow)
    {
        cv::destroyWindow("[Board_State_Sensor] cell outlines");
        cv::destroyWindow("[Board_State_Sensor] red  mask of the board");
        cv::destroyWindow("[Board_State_Sensor] blue mask of the board");
    }
}
