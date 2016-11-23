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
                       ROSThreadImage(name), doShow(_show), state(STATE_INIT), r(40) // 40Hz
{
    pthread_mutexattr_t _mutex_attr;
    pthread_mutexattr_init(&_mutex_attr);
    pthread_mutexattr_settype(&_mutex_attr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(&mutex_b, &_mutex_attr);

    cells_client = _n.serviceClient<DefineCells>("/define_cells");
    service = _n.advertiseService("/define_cells", &BoardState::serviceCb, this);

    board_publisher  = _n.advertise<MsgBoard>("/new_board", 1);
    ROS_ASSERT_MSG(board_publisher,"Empty publisher");

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
        if (state == STATE_INIT)
        {
            ROS_INFO("[%i] InternalThreadEntry CD", state);
            if (!_img_empty)
            {
                cv::Mat img_gray;
                cv::Mat img_binary;

                // convert image color model from BGR to grayscale
                pthread_mutex_lock(&_mutex_img);
                cv::cvtColor(_curr_img, img_gray, CV_BGR2GRAY);
                pthread_mutex_unlock(&_mutex_img);
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

                // find and display cell centroids
                if(contours.size() == 10)
                {
                    contours.erase(contours.begin() + largest_idx);
                    pthread_mutex_lock(&mutex_b);
                    board.cells.clear();
                    pthread_mutex_unlock(&mutex_b);

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
                        pthread_mutex_lock(&mutex_b);
                        board.cells.push_back(cell);
                        pthread_mutex_unlock(&mutex_b);
                    }

                    cv::imshow("[Cells_Definition] cell boundaries", board_cells);
                    cv::waitKey(3);
                    ++state;
                }
            }
        }
        else if (state == STATE_CALIB)
        {
            ROS_INFO("[%i] InternalThreadEntry BS", state);
            if (!_img_empty)
            {
                board.resetState();
                cv::Mat img_copy;

                pthread_mutex_lock(&_mutex_img);
                img_copy=_curr_img;
                pthread_mutex_unlock(&_mutex_img);

                DefineCells cells_srv;
                bool res = cells_client.call(cells_srv);
                ROS_INFO("[%i] InternalThreadEntry BS. Res: %i", state, res);

                if(res)
                {
                    board.resetState();
                    int cells_num = cells_srv.response.board.cells.size();
                    for(int i = 0; i < cells_num; i++)
                    {
                        cell.contours.clear();
                        int edges_num = cells_srv.response.board.cells[i].contours.size();
                        for(int j = 0; j < edges_num; j++)
                        {

                            cv::Point point(cells_srv.response.board.cells[i].contours[j].x, cells_srv.response.board.cells[i].contours[j].y);
                            cell.contours.push_back(point);
                        }

                        switch(cells_srv.response.board.cells[i].state)
                        {
                            case MsgCell::EMPTY:
                                cell.state = empty;
                                break;
                            case MsgCell::RED:
                                cell.state = red;
                                break;
                            case MsgCell::BLUE:
                                cell.state = blue;
                                break;
                            case MsgCell::UNDEFINED:
                                cell.state = undefined;
                                break;
                        }
                        board.cells.push_back(cell);
                    }
                }

                if (board.cells.size() == 9)
                {
                    MsgBoard msg_board;
                    for(int i = 0; i < msg_board.cells.size(); i++){
                        // msg_board.cells[i].state = MsgCell::UNDEFINED;
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
                        for (int j = 0; j < board.cells.size(); ++j)
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
                    for(int i = 0; i < board.cells.size(); i++)
                    {
                        vector<vector<cv::Point> > contours;
                        contours.push_back(board.cells[i].contours);
                        drawContours(bg, contours, -1, cv::Scalar(255,255,255), CV_FILLED, 8);
                    }

                    for (int j = 0; j < board.cells.size(); ++j)
                    {
                        Cell *cell = &(board.cells[j]);
                        if (cell->cell_area_red || cell->cell_area_blue)
                        {
                            cell->cell_area_red>cell->cell_area_blue?cell->state=red:cell->state=blue;
                        }

                        msg_board.cells[j].state=cell->state;
                    }

                    board_publisher.publish(msg_board);
                    last_msg_board=msg_board;
                    ROS_DEBUG("New board state published");

                    cv::Mat img = img_copy.clone();

                    // drawing all cells of the board game
                    cv::drawContours(img,board.as_vector_of_vectors(),-1, cv::Scalar(123,125,0),2); // drawing just the borders
                    for(int i = 0; i < board.cells.size(); i++)
                    {
                        cv::Point cell_centroid;
                        board.cells[i].get_cell_centroid(cell_centroid);
                        //cv::circle(img, p,5,cv::Scalar(0,0, 255),-1);
                        // cv::line(img, cell_centroid, cell_centroid, cv::Scalar(255,255,0), 2, 8);
                        cv::putText(img, intToString(i+1), cell_centroid, cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(255,255,0));
                    }

                    if (true) cv::imshow("[Board_State_Sensor] cell outlines", img);
                    cv::waitKey(10);
                }
            }
        }

        r.sleep();
    }
    ROS_INFO("[%i] Finishing", state);
}

bool BoardState::serviceCb(DefineCells::Request &req, DefineCells::Response &res)
{
    // ROS_INFO("[%i] serviceCb", state);
    if(state == STATE_CALIB)
    {
        MsgCell cell;

        pthread_mutex_lock(&mutex_b);
        for(int i = 0; i < board.cells.size(); i++)
        {
            cell.state = MsgCell::EMPTY;
            for(int j = 0; j < board.cells[i].contours.size(); j++)
            {
                cell.contours[j].x = board.cells[i].contours[j].x;
                cell.contours[j].y = board.cells[i].contours[j].y;
            }
            res.board.cells[i] = cell;
        }
        pthread_mutex_unlock(&mutex_b);

        return true;
    }

    return false;
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
