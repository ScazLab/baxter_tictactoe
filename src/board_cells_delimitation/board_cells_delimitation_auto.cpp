/* outline for detection of cell areas

1. take in image from usb cam
2. convert ROS image to cv::Mat
3. convert matrix color model from BGR to grayscale
4. mask out board 
5. 



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

   		cv::Mat img_mask;

   		// a contour is an array of x-y coordinates describing the boundaries of an object
   		std::vector<std::vector<cv::Point> > contours;
   		// Vec4i = vectors w/ 4 ints
   		std::vector<cv::Vec4i> hierarchy;

   		cv::findContours(img_binary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

   		double largest_area = 0;
   		int largest_area_index = 0;
   		for(int i = 0; i < contours.size(); i++){
   			largest_area = largest_area>contourArea(contours[i], false)?largest_area:contourArea(contours[i], false);
   			largest_area_index = largest_area>contourArea(contours[i], false)?largest_area_index:i;
   		}

   		cv::Mat board_outline = cv::Mat::zeros(img_binary.size(), CV_8UC3);
   		// for(int i = 0; i < contours.size(); i++){
   		// 	drawContours(board_outline, contours, -1, cv::Scalar(255,255,255), CV_FILLED);
   		// }

   		drawContours(board_outline, contours, largest_area_index, cv::Scalar(255,255,255), CV_FILLED);

        cv::imshow(cellDelimitation::window_name, board_outline);
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