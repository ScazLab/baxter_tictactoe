/* outline for detection of cell areas

3 Options to Isolate a Colored Segment

(1) inRange
Parameters: void inRange(InputArray src, Scalar lowerb, Scalar upperb, OutputArray dst)
Functionality: sets element to black if outside of range, white if inside
Example: https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/

(2) threshold
Parameters: double threshold(InputArray src, OutputArray dst, double threshval, double maxval, int type)
Functionality: if type == THRESH_BINARY, dst(x) = maxval if src(x) > thresh, 0 if src(x) <= thresh

(3) findContours
   		
OpenCV bug with converting ROS image into a cv::Mat

Faulty approach

        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img_aux = cv_ptr->image.clone();

Recommended approach via https://github.com/sociallyassistiverobotics/clm_ros_wrapper/blob/master/src/CLMWrapper.cpp
    
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msgIn);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::cvtColor(cv_ptr->image.clone(), captured_image, CV_BGR2RGB);;

*/

#include "board_cells_delimitation_auto.h"

namespace ttt {

	cellDelimitation::cellDelimitation() : image_transport(node_handle), window_name("Cell Delimitation")
	{
		image_subscriber = image_transport.subscribe("in", 1, &cellDelimitation::imageCallback, this);

		cv::namedWindow(cellDelimitation::window_name);
	}

	cellDelimitation::~cellDelimitation()
	{
		cv::destroyWindow(cellDelimitation::window_name);
	}

	int getIthIndex(std::vector<std::vector<cv::Point> > contours, Index ith){	
		
		if(ith != LARGEST && ith != NEXT_LARGEST){
			ROS_ERROR("[Cells_Delimitation_Auto] Index value is invalid. Valid inputs are limited to LARGEST = 1, NEXT_LARGEST = 2");
		};

		double largest_area = 0;
		int largest_area_index = 0;
		double next_largest_area = 0;
		int next_largest_area_index = 0;

		// iterate through contours and keeps track of contour w/ largest and 2nd-largest area
		for(int i = 0; i < contours.size(); i++){
			if(contourArea(contours[i], false) > largest_area){
				next_largest_area = largest_area;
				next_largest_area_index = largest_area_index;
				largest_area = contourArea(contours[i], false);
				largest_area_index = i;
			}
			else if(next_largest_area < contourArea(contours[i], false) && contourArea(contours[i], false) < largest_area){
				next_largest_area = contourArea(contours[i], false);
				next_largest_area_index = i;
			}
		}

		return ith==LARGEST?largest_area_index:next_largest_area_index;
	}

	cv::Point findCentroid(std::vector<cv::Point> contour){
		double x = cv::moments(contour, false).m10 / cv::moments(contour, false).m00;
		double y = cv::moments(contour, false).m01 / cv::moments(contour, false).m00;
		cv::Point point(x,y);
		return point;
	}

	void cellDelimitation::imageCallback(const sensor_msgs::ImageConstPtr& msg){

		board.resetState();

		ttt_board_sensor::ttt_board msg_board;
        msg_board.data.assign(undefined); // initially the state of each cell is undefined
        msg_board.header.stamp = msg->header.stamp;
        msg_board.header.frame_id = msg->header.frame_id;

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
   		cv::threshold(img_gray, img_binary, 150, 255, cv::THRESH_BINARY);

   		// a contour is an array of x-y coordinates describing the boundaries of an object
   		std::vector<std::vector<cv::Point> > contours;
   		// Vec4i = vectors w/ 4 ints
   		std::vector<cv::Vec4i> hierarchy;

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
   		for(int i = 0; i < contours.size(); i++){
   			if(i != largest_area_index){
   				drawContours(board_cells, contours, i, 
   							cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy);
   			}
   		}

   		// find and display cell centroids 
		std::vector<std::vector<cv::Point> > cells(9);
   		std::vector<cv::Point> cell_centroids(9);
   		for(int i = 0; i < contours.size(); i++){
   			int index = 0;
   			if(i != largest_area_index){
   				cells[index] = contours[i];
   				cell_centroids[index] = findCentroid(contours[i]); 
   				// draw centroids on screen
   				cv::line(board_cells, cell_centroids[index], cell_centroids[index], cv::Scalar(0,0,0), 3, 8);

   				++index;
			}
   		}

        cv::imshow(cellDelimitation::window_name, board_cells);
        // cv::imshow(cellDelimitation::window_name, outer_board_outline);
		cv::waitKey(30);
	}
}



int main(int argc, char ** argv){
	ros::init(argc, argv, "cell_delimitation_auto");
	ROS_DEBUG("in main");


	ttt::cellDelimitation cd;

	ros::spin();
	return 0;
}