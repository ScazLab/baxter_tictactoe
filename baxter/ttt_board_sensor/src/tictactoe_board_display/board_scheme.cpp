#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ttt_board_sensor/ttt_board.h"

namespace ttt
{

class BoardScheme
{
private:

    static const char WINDOW[];

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

    ros::Subscriber sub;


    cv::Mat draw_board(const ttt_board_sensor::ttt_board::ConstPtr& msg) const
    {
        ROS_DEBUG("@display_board");

        cv::Mat img(480,640,CV_8UC3,cv::Scalar(255,255,255));

        // 3x3 tic tac toe board
        const unsigned short int COLS = 3;
        const unsigned short int ROWS = 3;

        unsigned short int col, row;

        int cols_img = img.cols;
        int rows_img = img.rows;
        ROS_DEBUG_STREAM("cols_img=" << cols_img << " rows_img=" << rows_img);

        unsigned int cols_cell_img = cols_img/COLS;
        unsigned int rows_cell_img = rows_img/ROWS;
        ROS_DEBUG_STREAM("cols_cell_img=" << cols_cell_img << " rows_cell_img=" << rows_cell_img);

        this->draw_lines(img);

        for (size_t i = 0; i < msg->data.size(); ++i) {
            ROS_DEBUG_STREAM("Processing Cell " << i);
            if(msg->data[i]>0)
            {
                col=i%COLS;
                row=i/ROWS;
                cv::Point center_cell(cols_cell_img/2 + cols_cell_img*col, rows_cell_img/2 + rows_cell_img*row);
                ROS_DEBUG_STREAM("Center point in cell " << i << " (" << center_cell.x << "," << center_cell.y << ")");
                cv::putText(img,
                            msg->data[i]==1?"X":"O",
                            cv::Point(center_cell.x-25,center_cell.y+25), //we apply an offset of 25 to center the caracter in the cell
                            cv::FONT_HERSHEY_TRIPLEX,
                            3, msg->data[i]==1?cv::Scalar(255,0,0):cv::Scalar(0,0,255), 2);
            }
        }
        return img;
    }

    void draw_lines(cv::Mat& img) const
    {
        cv::line(img,cv::Point(img.cols/3,0),cv::Point(img.cols/3,480),cv::Scalar(0),2);
        cv::line(img,cv::Point(2*img.cols/3,0),cv::Point(2*img.cols/3,480),cv::Scalar(22),2);
        cv::line(img,cv::Point(0,img.rows/3),cv::Point(640,img.rows/3),cv::Scalar(0),2);
        cv::line(img,cv::Point(0,2*img.rows/3),cv::Point(640,2*img.rows/3),cv::Scalar(0),2);
    }

    void show_board_temporary(cv::Mat& img, int msec) const
    {
        cv::namedWindow(BoardScheme::WINDOW);
        cv::imshow(BoardScheme::WINDOW,img);
        cv::waitKey(msec);
        cv::destroyAllWindows();
    }

    void publish_draw_board(const ttt_board_sensor::ttt_board::ConstPtr& msg) const
    {
        cv::Mat img_board = this->draw_board(msg);
        //this->show_board_temporary(img_board,30000);

        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image    = img_board; // Your cv::Mat

        this->image_pub_.publish(out_msg.toImageMsg());
    }

public:

    BoardScheme() : it_(nh_)
    {
        image_pub_ = it_.advertise("new_board_scheme", 1);
        sub = nh_.subscribe("/new_board", 1, &BoardScheme::publish_draw_board, this);
    }



};

const char BoardScheme::WINDOW[] = "Tic Tac Toe Board Status";
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "board_display");
    ttt::BoardScheme bd;
    ros::spin();
    return 0;
}
