#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <robot_perception/hsv_detection.h>

class HsvRangeFinder
{
private:
    ros::NodeHandle node_handle;

    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_subscriber;

    std::string window_name;

    hsvColorRange hsv;

public:
    explicit HsvRangeFinder(std::string _name) : node_handle(_name), image_transport(node_handle), window_name(_name),
                                                 hsv(colorRange(60, 130), colorRange(90,256), colorRange(10,256))
    {
        // left hand camera
        image_subscriber = image_transport.subscribe("/image_in", 1,
                            &HsvRangeFinder::imageCallback, this);

        cv::namedWindow(window_name);

        cv::createTrackbar("LowerH", window_name, &hsv.H.min, 180, NULL);
        cv::createTrackbar("UpperH", window_name, &hsv.H.max, 180, NULL);
        cv::createTrackbar("LowerS", window_name, &hsv.S.min, 256, NULL);
        cv::createTrackbar("UpperS", window_name, &hsv.S.max, 256, NULL);
        cv::createTrackbar("LowerV", window_name, &hsv.V.min, 256, NULL);
        cv::createTrackbar("UpperV", window_name, &hsv.V.max, 256, NULL);
    }

    ~HsvRangeFinder()
    {
        cv::destroyWindow(window_name);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        //convert ROS image format to opencv image format
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img_hsv(cv_ptr->image.rows, cv_ptr->image.cols,CV_8UC3);
        cv::cvtColor(cv_ptr->image, img_hsv, CV_BGR2HSV);

        cv::Mat img_thresh = hsvThreshold(img_hsv, hsv);

        cv::imshow(window_name, img_thresh);

        int c = cv::waitKey(3);
        if( (c & 255) == 27 ) // ESC key pressed
        {
            ROS_INFO("Finished. HSV: %s .", hsv.toString().c_str());
            ROS_INFO(" Copy this into the launch file:");
            printf("        H: [ %i, %i]\n", hsv.H.min, hsv.H.max);
            printf("        S: [ %i, %i]\n", hsv.S.min, hsv.S.max);
            printf("        V: [ %i, %i]\n", hsv.V.min, hsv.V.max);
            ros::shutdown();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hsv_range_finder");
    HsvRangeFinder hsv_range_finder("hsv_range_finder");
    ros::spin();
    return 0;
}
