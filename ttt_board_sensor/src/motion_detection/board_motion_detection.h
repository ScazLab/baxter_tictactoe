#ifndef BOARD_MOTION_DETECTION_H
#define BOARD_MOTION_DETECTION_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "src/ttt_definitions.h"

namespace ttt
{

class Board_Motion_Detection
{
private:

    typedef unsigned int point_img_type;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    static const char WINDOW[];

    cv::Mat _last_gray_img;

public:
    Board_Motion_Detection();
    ~Board_Motion_Detection();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

};

const char Board_Motion_Detection::WINDOW[] = "Board Motion Detection";

}

#endif //BOARD_MOTION_DETECTION_H
