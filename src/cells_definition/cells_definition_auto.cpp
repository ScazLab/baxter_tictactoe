#include "cells_definition_auto.h"

using namespace ttt;
using namespace baxter_tictactoe;
using namespace std;

cellsDefinition::cellsDefinition() : image_transport(node_handle)
{
    img_loaded = false;
    pthread_mutex_init(&mutex_b, NULL);
    pthread_mutex_init(&mutex_i, NULL);
    
	image_subscriber = image_transport.subscribe("image_in", 1, &cellsDefinition::imageCallback, this);  
    
    service = node_handle.advertiseService("baxter_tictactoe/define_cells", &cellsDefinition::defineCells, this);

    // cv::namedWindow("[Cells_Definition_Auto] cell boundaries", cv::WINDOW_NORMAL);
    // cv::resizeWindow("[Cells_Definition_Auto] cell boundaries", 1000, 1000);

    // cv::namedWindow("[Cells_Definition_Auto] raw", cv::WINDOW_NORMAL);
    // cv::resizeWindow("[Cells_Definition_Auto] raw", 1000, 1000);
}

cellsDefinition::~cellsDefinition() 
{
    // cv::destroyWindow("[Cells_Definition_Auto] cell boundaries");
    // cv::destroyWindow("[Cells_Definition_Auto] raw");
}

bool cellsDefinition::defineCells(DefineCells::Request &req, DefineCells::Response &res)
{
    pthread_mutex_lock(&mutex_i);
    bool img_loaded_copy = img_loaded;
    pthread_mutex_unlock(&mutex_i);

    if(img_loaded_copy == true)
    {
        MsgCell cell;

        pthread_mutex_lock(&mutex_b);
        for(int i = 0; i < board.cells.size(); i++)
        {
            cell.state = MsgCell::EMPTY;
            for(int j = 0; j < board.cells[i].contours.size(); j++)
            {
                // ROS_INFO("[BOARD] x: %d y: %d", board.cells[i].contours[j].x, board.cells[i].contours[j].y);
                cell.contours[j].x = board.cells[i].contours[j].x;
                cell.contours[j].y = board.cells[i].contours[j].y;
                // ROS_INFO("[CELL] x: %d y: %d", cell.contours[j].x, cell.contours[j].y);
            }
            res.board.cells[i] = cell;
        }
        pthread_mutex_unlock(&mutex_b);

        return true;    
    }
    else {return false;}
}

int cellsDefinition::getIthIndex(vector<vector<cv::Point> > contours, Index ith)
{	
	
	if(ith != LARGEST && ith != NEXT_LARGEST)
    {
		ROS_ERROR("[Cells_Definition_Auto] Index value is invalid. Valid inputs are limited to LARGEST = 1, NEXT_LARGEST = 2");
	};

	double largest_area = 0;
	int largest_area_index = 0;
	double next_largest_area = 0;
	int next_largest_area_index = 0;

	// iterate through contours and keeps track of contour w/ largest and 2nd-largest area
	for(int i = 0; i < contours.size(); i++)
    {
		if(contourArea(contours[i], false) > largest_area)
        {
			next_largest_area = largest_area;
			next_largest_area_index = largest_area_index;
			largest_area = contourArea(contours[i], false);
			largest_area_index = i;
		}
		else if(next_largest_area < contourArea(contours[i], false) && contourArea(contours[i], false) < largest_area)
        {
			next_largest_area = contourArea(contours[i], false);
			next_largest_area_index = i;
		}
	}

	return ith==LARGEST?largest_area_index:next_largest_area_index;
}

cv::Point cellsDefinition::findCentroid(vector<cv::Point> contour)
{
	double x = cv::moments(contour, false).m10 / cv::moments(contour, false).m00;
	double y = cv::moments(contour, false).m01 / cv::moments(contour, false).m00;
	cv::Point point(x,y);
	return point;
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

void cellsDefinition::onMouseClick( int event, int x, int y, int, void* param)
{
    if( event != cv::EVENT_LBUTTONDOWN )
        return;

    cv::Point p = cv::Point(x,y);

    ROS_INFO_STREAM("Point: " << p.x << " , " << p.y << "." );
    

    return;
}

void cellsDefinition::imageCallback(const sensor_msgs::ImageConstPtr& msg){

    // convert ROS image to Cv::Mat
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat img_gray;
	cv::Mat img_binary;

	// convert image color model from BGR to grayscale
	cv::cvtColor(cv_ptr->image.clone(), img_gray, CV_BGR2GRAY);
	// convert grayscale image to binary image, using 155 threshold value to 
	// isolate white-colored board
	cv::threshold(img_gray, img_binary, 140, 255, cv::THRESH_BINARY);

	// a contour is an array of x-y coordinates describing the boundaries of an object
	vector<vector<cv::Point> > contours;
	// Vec4i = vectors w/ 4 ints
	vector<cv::Vec4i> hierarchy;

	// find white edges of outer board by finding contours (i.e boundaries)
	cv::findContours(img_binary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	// isolate contour w/ the largest area to separate outer board from other objects in
	// image (assuming outer board is largest object in image)
	int largest_area_index = getIthIndex(contours, LARGEST);

	// draw outer board contour (i.e boundaries) onto zero matrix (i.e black image)
	cv::Mat outer_board = cv::Mat::zeros(img_binary.size(), CV_8UC1);
	drawContours(outer_board, contours, largest_area_index, 
				cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy);

	// find black edges of inner board by finding contours
	cv::findContours(outer_board, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	// isolate inner board contour by finding contour w/ second largest area (given that
	// outer board contour has the largest area)
	int next_largest_area_index = getIthIndex(contours, NEXT_LARGEST);

	// draw inner board contour onto zero matrix
	cv::Mat inner_board = cv::Mat::zeros(outer_board.size(), CV_8UC1);
	drawContours(inner_board, contours, next_largest_area_index, 
				cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy);

	cv::findContours(inner_board, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	largest_area_index = getIthIndex(contours, LARGEST);

	// drawn board cells onto zero matrix by drawing all contour except the 
	// the largest-area contour (which is the inner board contour)
	cv::Mat board_cells = cv::Mat::zeros(inner_board.size(), CV_8UC1);

	// find and display cell centroids 
	contours.erase(contours.begin() + largest_area_index);

    if(contours.size() == 9)
    {
        pthread_mutex_lock(&mutex_b);
        board.cells.clear();
        pthread_mutex_unlock(&mutex_b);

        // aproximate cell contours to quadrilaterals
        vector<vector<cv::Point> > apx_contours;
        for(int i = 0; i < contours.size(); i++)
        {
            double epsilon = arcLength(contours[i], true);
            // printf("epsilon: %0.3f\n", epsilon);
            vector<cv::Point> apx_cell;
            approxPolyDP(contours[i], apx_cell, 0.1 * epsilon, true);
            apx_contours.push_back(apx_cell);
        }

        for(int i = 0; i < apx_contours.size(); i++)
        {
            drawContours(board_cells, apx_contours, i, cv::Scalar(255,255,255), CV_FILLED, 8);
        }

        vector<cv::Point> cell_centroids;

        for(int i = 0; i < apx_contours.size(); i++)
        {
            cell_centroids.push_back(findCentroid(apx_contours[i])); 
        }

        // sort cell_contours and cell_centroids in descending order of y-coordinate (not needed as empirical
        // test shows that findContours apparently finds contours in ascending order of y-coordinate already)
            
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

        pthread_mutex_lock(&mutex_i);
        img_loaded = true;
        pthread_mutex_unlock(&mutex_i);
    }
    else
    {
        pthread_mutex_lock(&mutex_i);
        img_loaded = false;
        pthread_mutex_unlock(&mutex_i);
        // ROS_ERROR("[imageCallback(server node)] Image callback was not executed");
    }


    cv::setMouseCallback("[Cells_Definition_Auto] cell boundaries", onMouseClick, this);
    // cv::imshow("[Cells_Definition_Auto] cell boundaries", board_cells);
    // cv::imshow("[Cells_Definition_Auto] raw", cv_ptr->image.clone());

    cv::waitKey(3);
}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "cells_definition_auto");
    // ros::NodeHandle n;
	cellsDefinition cd;

	ros::spin();
	return 0;
}