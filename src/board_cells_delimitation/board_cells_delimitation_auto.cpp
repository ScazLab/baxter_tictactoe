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
   		cv::cvtColor(cv_ptr->image.clone(), img_gray, CV_BGR2GRAY);
   		cv::threshold(img_gray, img_binary, 50, 255, cv::THRESH_BINARY);

        cv::imshow(cellDelimitation::window_name, img_binary);
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