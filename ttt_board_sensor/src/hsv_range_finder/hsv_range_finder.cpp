#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ttt/ttt_definitions.h"


namespace ttt
{


class HsvRangeFinder
{
private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    int lowerH;
    int lowerS;
    int lowerV;
    int upperH;
    int upperS;
    int upperV;

    static const char WINDOW[];

    std::string board_config;

    t_Board board; // A vector of cells representing the board game


public:
    HsvRangeFinder()
        : it_(nh_)
    {
        image_sub_ = it_.subscribe("image_in", 1, &HsvRangeFinder::imageCb, this);

        cv::namedWindow(HsvRangeFinder::WINDOW);

        this->lowerH=0;
        this->lowerS=0;
        this->lowerV=0;
        this->upperH=180;
        this->upperS=256;
        this->upperV=256;

        cv::createTrackbar("LowerH", HsvRangeFinder::WINDOW, &this->lowerH, 180, NULL);
        cv::createTrackbar("UpperH", HsvRangeFinder::WINDOW, &this->upperH, 180, NULL);
        cv::createTrackbar("LowerS", HsvRangeFinder::WINDOW, &this->lowerS, 256, NULL);
        cv::createTrackbar("UpperS", HsvRangeFinder::WINDOW, &this->upperS, 256, NULL);
        cv::createTrackbar("LowerV", HsvRangeFinder::WINDOW, &this->lowerV, 256, NULL);
        cv::createTrackbar("UpperV", HsvRangeFinder::WINDOW, &this->upperV, 256, NULL);
    }

    ~HsvRangeFinder()
    {
        cv::destroyWindow(HsvRangeFinder::WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        //converting ROS image format to opencv image format
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

        cv::Mat img_hsv(cv_ptr->image.rows, cv_ptr->image.cols,CV_8UC3);
        cv::cvtColor(cv_ptr->image, img_hsv, CV_BGR2HSV); //Change the color format from BGR to HSV

        cv::Mat img_thresh = GetThresholdedImage(img_hsv, cv::Scalar(this->lowerH,this->lowerS,this->lowerV), cv::Scalar(this->upperH,this->upperS,this->upperV));

        cv::imshow(HsvRangeFinder::WINDOW, img_thresh);

        int c = cv::waitKey(3);
        if( (c & 255) == 27 ) // ESC key pressed
        {
            ROS_INFO_STREAM("H=[" << this->lowerH << ".." << this->upperH << "] S=[" << this->lowerS << ".." << this->upperS << "] V=[" << this->lowerV << ".." << this->upperV << "]");
            ros::shutdown();
        }        
    }

    //This function threshold the HSV image and create a binary image
    static cv::Mat GetThresholdedImage(const cv::Mat& img_hsv, cv::Scalar lower, cv::Scalar upper)
    {

     cv::Mat img_thresh(img_hsv.rows,img_hsv.cols,CV_8U);
     cv::inRange(img_hsv, lower, upper, img_thresh);

     return img_thresh;

    }
};

const char HsvRangeFinder::WINDOW[] = "HSV Range Finder";

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hsv_range_finder");
    ttt::HsvRangeFinder cd;
    ros::spin();
    return 0;
}
