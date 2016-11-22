#include "cellsDefinition.h"

using namespace ttt;
using namespace baxter_tictactoe;
using namespace std;

cellsDefinition::cellsDefinition(string name) : ROSThreadImage(name), r(40) // 40Hz
{
    img_loaded = false;
    pthread_mutex_init(&mutex_b, NULL);

    service = _n.advertiseService("/define_cells",
                                   &cellsDefinition::serviceCb, this);

    // cv::namedWindow("[Cells_Definition] cell boundaries", cv::WINDOW_NORMAL);
    // cv::resizeWindow("[Cells_Definition] cell boundaries", 960, 600);
}

bool cellsDefinition::serviceCb(DefineCells::Request &req, DefineCells::Response &res)
{
    // ROS_INFO("serviceCb");
    if(img_loaded == true)
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

int cellsDefinition::getIthIndex(vector<vector<cv::Point> > contours, Index ith)
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

    return ith==LARGEST?largest_idx:next_largest_idx;
}

std::string cellsDefinition::intToString( const int a )
{
    stringstream ss;
    ss << a;
    return ss.str();
}

bool cellsDefinition::ascendingY(vector<cv::Point> i, vector<cv::Point> j)
{
    double y_i = moments(i, false).m01 / moments(i, false).m00;
    double y_j = moments(j, false).m01 / moments(j, false).m00;

    return y_i < y_j;
}

bool cellsDefinition::ascendingX(vector<cv::Point> i, vector<cv::Point> j)
{
    double x_i = moments(i, false).m10 / moments(i, false).m00;
    double x_j = moments(j, false).m10 / moments(j, false).m00;

    return x_i < x_j;
}

void cellsDefinition::InternalThreadEntry()
{
    while(ros::ok())
    {
        ROS_INFO("InternalThreadEntry CD");
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
            int largest_idx = getIthIndex(contours, LARGEST);

            // draw outer board contour (i.e boundaries) onto zero matrix (i.e black image)
            cv::Mat outer_board = cv::Mat::zeros(img_binary.size(), CV_8UC1);
            drawContours(outer_board, contours, largest_idx,
                         cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy);

            // find black edges of inner board by finding contours
            cv::findContours(outer_board, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

            // isolate inner board contour by finding contour w/ second largest area (given that
            // outer board contour has the largest area)
            int next_largest_idx = getIthIndex(contours, NEXT_LARGEST);

            // draw inner board contour onto zero matrix
            cv::Mat inner_board = cv::Mat::zeros(outer_board.size(), CV_8UC1);
            drawContours(inner_board, contours, next_largest_idx,
                         cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy);

            cv::findContours(inner_board, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

            largest_idx = getIthIndex(contours, LARGEST);

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

                img_loaded = true;
            }
            else
            {
                img_loaded = false;
            }

            cv::imshow("[Cells_Definition] cell boundaries", board_cells);
            cv::waitKey(3);
        }

        r.sleep();
    }
}

cellsDefinition::~cellsDefinition()
{
    // cv::destroyWindow("[Cells_Definition] cell boundaries");
}

