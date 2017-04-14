#include "boardState.h"

using namespace ttt;
using namespace baxter_tictactoe;
using namespace std;

bool operator==(MsgBoard board1, MsgBoard board2)
{
    if (board1.cells.size()!=board2.cells.size()) { return false; }

    for (size_t i = 0; i < board1.cells.size(); ++i)
    {
        if (board1.cells[i].state!=board2.cells[i].state) { return false; }
    }

    return true;
}

bool operator!=(MsgBoard board1, MsgBoard board2)
{
    return not (board1==board2);
}

BoardState::BoardState(string name, bool _show) : ROSThreadImage(name),
               doShow(_show), board_state(STATE_INIT), brain_state(-1)
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
        if (not _img_empty)
        {
            pthread_mutex_lock(&_mutex_img);
            img_in=_curr_img;
            pthread_mutex_unlock(&_mutex_img);
            img_out = img_in.clone();
        }

        if (board_state == STATE_INIT)
        {
            ROS_DEBUG_THROTTLE(1,"[%i] Initializing..", board_state);
            if (brain_state == TTTBrainState::READY || brain_state == TTTBrainState::GAME_STARTED) { ++board_state; }
        }
        else if (board_state == STATE_CALIB && not ros::isShuttingDown())
        {
            ROS_DEBUG_THROTTLE(1,"[%i] Calibrating board..", board_state);
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
                Contours contours;
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
                    board.resetBoard();

                    // aproximate cell contours to quadrilaterals
                    Contours apx_contours;
                    for (size_t i = 0; i < contours.size(); i++)
                    {
                        double epsilon = arcLength(contours[i], true);
                        Contour apx_contour_cell;
                        approxPolyDP(contours[i], apx_contour_cell, 0.1 * epsilon, true);
                        apx_contours.push_back(apx_contour_cell);
                    }

                    for (size_t i = 0; i < apx_contours.size(); i++)
                    {
                        drawContours(board_cells, apx_contours, i, cv::Scalar(255,255,255), CV_FILLED, 8);
                    }

                    // sort cell_contours in descending order of y-coordinate (not needed as test shows
                    // that findContours apparently finds contours in ascending order of y-coordinate already)
                    std::sort(apx_contours.begin(), apx_contours.end(), ascendingY);

                    for (size_t i = 0; i <=6; i += 3)
                    {
                        std::sort(apx_contours.begin() + i, apx_contours.begin() + i + 3, ascendingX);
                    }

                    for (size_t i = 0; i < apx_contours.size(); i++)
                    {
                        board.addCell(Cell(apx_contours[i]));
                    }

                    if(doShow) { cv::imshow("[Cells_Definition] cell boundaries", board_cells); }

                    cv::waitKey(3);

                    if (isBoardSane()) { ++board_state; }
                }
            }
        }
        else if (board_state == STATE_READY && not ros::isShuttingDown())
        {
            ROS_DEBUG_THROTTLE(1, "[%i] Detecting Board State.. NumCells %i", board_state, board.getNumCells());
            if (not _img_empty)
            {
                board.resetCellStates();
                cv::Mat img_copy = img_in.clone();

                if (board.getNumCells() == NUMBER_OF_CELLS)
                {
                    // convert the original image to hsv color space
                    cv::Mat img_hsv;
                    cv::cvtColor(img_copy,img_hsv,CV_BGR2HSV);

                    // mask the original image to the board
                    cv::Mat img_hsv_mask = board.maskImage(img_hsv);

                    for (size_t i = 0; i < 2; ++i)
                    {
                        cv::Mat hsv_filt_mask = hsvThreshold(img_hsv_mask, i==0?hsv_red:hsv_blue);
                        if (doShow)
                        {
                            if (i==0) { cv::imshow("[Board_State_Sensor] red  mask of the board", hsv_filt_mask); }
                            if (i==1) { cv::imshow("[Board_State_Sensor] blue mask of the board", hsv_filt_mask); }
                        }

                        for (size_t j = 0; j < board.getNumCells(); ++j)
                        {
                            Cell &cell = board.getCell(j);
                            cv::Mat crop = cell.maskImage(hsv_filt_mask);

                            // Let's smooth the image to reduce noise
                            cv::GaussianBlur(crop.clone(),crop,cv::Size(3,3),0,0);

                            // the area formed by the remaining pixels is computed based on the moments
                            int col_area = cv::moments(crop,true).m00;

                            if (col_area > area_threshold)
                            {
                                if (i==0)  { cell. setRedArea(col_area); }
                                else       { cell.setBlueArea(col_area); }
                            }
                        }
                    }

                    board.computeState();
                    board_state_pub.publish(board.toMsgBoard());

                    // ROS_INFO("New board state published");

                    for (size_t i = 0; i < board.getNumCells(); i++)
                    {
                        cv::Scalar col = col_empty;
                        if (board.getCellState(i) ==  COL_RED) { col =  col_red; }
                        if (board.getCellState(i) == COL_BLUE) { col = col_blue; }

                        ttt::Contours contours;
                        contours.push_back(board.getCellContour(i));

                        cv::drawContours(img_out, contours,-1, col, CV_FILLED); // drawing just the borders
                        cv::putText(img_out, intToString(i+1), board.getCellCentroid(i),
                                             cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar::all(255), 2);
                    }

                    if (doShow) { cv::imshow("[Board_State_Sensor] cell outlines", img_out); }
                    cv::waitKey(1);
                }
            }
        }

        if (not _img_empty)
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_out).toImageMsg();
            img_pub.publish(msg);
        }
        r.sleep();
    }
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

bool BoardState::isBoardSane()
{
    for (size_t i = 0; i < board.getNumCells(); ++i)
    {
        // Check if area of cell is big enough
        int cell_area = board.getCellArea(i);
        if ( cell_area < area_threshold)
        {
            ROS_WARN("Cell #%lu has area %i (smaller than area_threshold)", i, cell_area);
            return false;
        }

        // Check if centroid of cells is not contained in another cell
        for (size_t j = 0; j < board.getNumCells(); ++j)
        {
            if (j != i)
            {
                if (cv::pointPolygonTest(board.getCellContour(j), board.getCellCentroid(i), true) >= 0)
                {
                    ROS_WARN("Point #%lu is inside cell #%lu", i, j);
                    return false;
                }
            }
        }
    }
    return true;
}

int BoardState::getIthIndex(Contours contours, int ith)
{
    double largest_area = 0;
    int     largest_idx = 0;
    double next_largest_area = 0;
    int     next_largest_idx = 0;

    // iterate through contours and keeps track of contour w/ largest and 2nd-largest area
    for (size_t i = 0; i < contours.size(); i++)
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

bool BoardState::ascendingY(Contour i, Contour j)
{
    double y_i = moments(i, false).m01 / moments(i, false).m00;
    double y_j = moments(j, false).m01 / moments(j, false).m00;

    return y_i < y_j;
}

bool BoardState::ascendingX(Contour i, Contour j)
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
