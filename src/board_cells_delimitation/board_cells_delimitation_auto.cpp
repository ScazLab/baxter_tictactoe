/* outline for detection of cell areas

1. take in image from usb cam
2. convert ROS image to cv::Mat
3. convert matrix color model from BGR to grayscale
4. mask out board 
5. 
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

	void cellDelimitation::imageCallback(const sensor_msgs::ImageConstPtr& msg){

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

		// cv::Mat img_board = cv_ptr->image.clone();
		// cv::imshow(cellDelimitation::window_name, img);
		int c = cv::waitKey(3);
	}

}

int main(int argc, char ** argv){
	ros::init(argc, argv, "cells_delimitation_auto");
	ttt::cellDelimitation cd;
	ros::spin();
	return 0;
}