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

	int cellDelimitation::getIthIndex(std::vector<std::vector<cv::Point> > contours, Index ith){	
		
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

	cv::Point cellDelimitation::findCentroid(std::vector<cv::Point> contour){
		double x = cv::moments(contour, false).m10 / cv::moments(contour, false).m00;
		double y = cv::moments(contour, false).m01 / cv::moments(contour, false).m00;
		cv::Point point(x,y);
		return point;
	}

	void cellDelimitation::imageCallback(const sensor_msgs::ImageConstPtr& msg){

    board.cells.clear();

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

 		// find and display cell centroids 
 		contours.erase(contours.begin() + largest_area_index);

    // aproximate cell contours to quadrilaterals
    std::vector<std::vector<cv::Point> > apx_contours;
    for(int i = 0; i < contours.size(); i++)
    {
      double epsilon = arcLength(contours[i], true);
      // printf("epsilon: %0.3f\n", epsilon);
      std::vector<cv::Point> apx_cell;
      approxPolyDP(contours[i], apx_cell, 0.1 * epsilon, true);
      apx_contours.push_back(apx_cell);
    }

    for(int i = 0; i < apx_contours.size(); i++)
    {
      drawContours(board_cells, apx_contours, i, cv::Scalar(255,255,255), CV_FILLED, 8);
    }

		std::vector<cv::Point> cell_centroids;

 		for(int i = 0; i < apx_contours.size(); i++){
  		cell_centroids.push_back(findCentroid(apx_contours[i])); 
		// printf("cell_centroids[%d]: x:%d y:%d\n", i, cell_centroids[i].x, cell_centroids[i].y);
		// cv::line(board_cells, cell_centroids[i], cell_centroids[i], cv::Scalar(0,0,0), 3, 8);

 		}

 		// sort cell_contours and cell_centroids in descending order of y-coordinate (not needed as empirical
 		// test shows that findContours apparently finds contours in ascending order of y-coordinate already)

   	// find leftmost and rightmost centroid
 		cv::Point leftmost_x(cell_centroids[0]);
 		cv::Point rightmost_x(cell_centroids[0]);
 		cv::Point highest_y(cell_centroids[cell_centroids.size() - 1]);

 		for(int i = 0; i < cell_centroids.size(); i++){
 			if(cell_centroids[i].x < leftmost_x.x) {
 				leftmost_x = cell_centroids[i];
 			}
 			if(cell_centroids[i].x > rightmost_x.x) {
 				rightmost_x = cell_centroids[i];
 			}
 		}

 		// cv::line(board_cells, leftmost_x, leftmost_x, cv::Scalar(0,0,0), 3, 8);
 		// cv::line(board_cells, rightmost_x, rightmost_x, cv::Scalar(0,0,0), 3, 8);
    // cv::line(board_cells, highest_y, highest_y, cv::Scalar(0,0,0), 3, 8);

		if( (rightmost_x.x - highest_y.x) > (highest_y.x - leftmost_x.x)){
			for(int i = apx_contours.size() - 1; i >= 0; i--){
				Cell cell(apx_contours[i]);
				board.cells.push_back(cell);
   			// cv::line(board_cells, cell_centroids[i], cell_centroids[i], cv::Scalar(0,0,0), i + 4, 8);
			}
		}
		else if( (rightmost_x.x - highest_y.x) <= (highest_y.x - leftmost_x.x)){
			int side_len = sqrt(apx_contours.size() + 1);
            // int thickness = contours.size();
			for(int i = side_len; i >= 1; i--){
				for(int j = side_len; j >= 1; j--){
					Cell cell(apx_contours[i * side_len - j]);
					board.cells.push_back(cell);
          // cv::line(board_cells, cell_centroids[i * side_len - j], cell_centroids[i * side_len - j], cv::Scalar(0,0,0), 3, 8);
				}		
			}
		}

    // board.save() SHOULD GO HERE
    cv::imshow(cellDelimitation::window_name, board_cells);
		// cv::waitKey(30);

    int c = cv::waitKey(30);
    if((char)c =='s'){
      printf("No. of cells in board: %lu\n", board.cells.size());
      board.save();
      ros::shutdown();
    }

	}
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "cell_delimitation_auto");
	ROS_DEBUG("in main");

	ttt::cellDelimitation cd;

	ros::spin();
	return 0;
}