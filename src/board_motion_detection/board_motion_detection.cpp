
#include "baxterTictactoe/tictactoe_utils.h"
#include "board_motion_detection.h"

namespace ttt
{

    Board_Motion_Detection::Board_Motion_Detection()
        : it_(nh_)
    {
        image_sub_ = it_.subscribe("in", 1, &Board_Motion_Detection::imageCb, this);

        cv::namedWindow(Board_Motion_Detection::WINDOW);

        _last_gray_img.dims=-1;
    }

    Board_Motion_Detection::~Board_Motion_Detection()
    {
        cv::destroyWindow(Board_Motion_Detection::WINDOW);
        cv::destroyAllWindows();
    }

    void Board_Motion_Detection::imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        //converting ROS image format to opencv image format
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Exception converting image from ROS to CV format: %s", e.what());
            return;
        }

        cv::Mat img_gray;
        cvtColor(cv_ptr->image.clone(),img_gray,CV_BGR2GRAY);
        if (_last_gray_img.dims!=-1) //This is not the first image received
        {
            if (_last_gray_img.rows!=img_gray.rows || _last_gray_img.cols!=img_gray.cols) {
                ROS_WARN_STREAM("Mismatch on size of images. Last one was " << _last_gray_img.rows << "x" << _last_gray_img.cols << " and new one is " << img_gray.rows << "x" << img_gray.cols);
                return;
            }
            for (int row = 0; row < _last_gray_img.rows; ++row) { //_last_img->rows==img_aux.rows && _last_img->cols==img_aux.cols
                for (int col = 0; col < _last_gray_img.cols; ++col) {
                    point_img_type p_last=_last_gray_img.at<point_img_type>(row,col);
                    point_img_type p_new=img_gray.at<point_img_type>(row,col);
                    ROS_INFO_STREAM(p_last << " - " << p_new << " |=| " << abs(p_last - p_new));
                }
            }
        }
        else {  // This is the first image received
            ROS_INFO("This is the first time an image is received. No previous image has been stored.");
        }
        img_gray.copyTo(_last_gray_img);

        //cv::imshow(Board_Motion_Detection::WINDOW, _last_gray_img);
        //cv::imshow(Board_Motion_Detection::WINDOW, img_gray);
        //cv::waitKey(300);
    }    

}

int main(int argc, char** argv)
{
    ROS_INFO("Detecting motion in the board");
    ros::init(argc, argv, "board_motion_detection");
    ttt::Board_Motion_Detection cd;
    ros::spin();
    return 0;
}
