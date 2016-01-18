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

    int height;
    int width;

    int n_cols;
    int n_rows;

    cv::Scalar white;
    cv::Scalar   red;
    cv::Scalar  blue;

    cv::Mat draw_board(const ttt_board_sensor::ttt_board::ConstPtr& msg) const
    {
        ROS_DEBUG("@display_board");

        cv::Mat img(height,width,CV_8UC3,cv::Scalar(255,255,255));

        // 3x3 tic tac toe board
        unsigned short int col, row;

        int cols_img = img.cols;
        int rows_img = img.rows;
        ROS_DEBUG_STREAM("cols_img=" << cols_img << " rows_img=" << rows_img);

        unsigned int cols_cell_img = cols_img/n_cols;
        unsigned int rows_cell_img = rows_img/n_rows;
        ROS_DEBUG_STREAM("cols_cell_img=" << cols_cell_img << " rows_cell_img=" << rows_cell_img);

        this->draw_lines(img);

        for (size_t i = 0; i < msg->data.size(); ++i) {
            ROS_DEBUG_STREAM("Processing Cell " << i);
            if(msg->data[i]>0)
            {
                col=i%n_cols;
                row=i/n_rows;
                cv::Point center_cell(cols_cell_img/2 + cols_cell_img*col, rows_cell_img/2 + rows_cell_img*row);
                cv::Point bottom_left_corner(cols_cell_img*1/6 + cols_cell_img*col, rows_cell_img*1/6 + rows_cell_img*row);
                cv::Point   top_right_corner(cols_cell_img*5/6 + cols_cell_img*col, rows_cell_img*5/6 + rows_cell_img*row);
                ROS_DEBUG_STREAM("Center point in cell " << i << " (" << center_cell.x << "," << center_cell.y << ")");

                cv::rectangle(img,bottom_left_corner,top_right_corner,msg->data[i]==1?red:blue,CV_FILLED);
                if (msg->data[i]==1)
                {
                    /* Draw X */
                    cv::line(img,
                             cv::Point(cols_cell_img*3/12 + cols_cell_img*col, rows_cell_img*3/12 + rows_cell_img*row),
                             cv::Point(cols_cell_img*9/12 + cols_cell_img*col, rows_cell_img*9/12 + rows_cell_img*row),
                             white,16);
                    cv::line(img,
                             cv::Point(cols_cell_img*9/12 + cols_cell_img*col, rows_cell_img*3/12 + rows_cell_img*row),
                             cv::Point(cols_cell_img*3/12 + cols_cell_img*col, rows_cell_img*9/12 + rows_cell_img*row),
                             white,16);
                } else {
                    cv::circle(img,center_cell,rows_cell_img*3/12,white,16);
                }
            }
        }
        return img;
    }

    void draw_lines(cv::Mat& img) const
    {
        cv::line(img,cv::Point(img.cols/3,0),cv::Point(img.cols/3,height),cv::Scalar(0),8);
        cv::line(img,cv::Point(2*img.cols/3,0),cv::Point(2*img.cols/3,height),cv::Scalar(22),8);
        cv::line(img,cv::Point(0,img.rows/3),cv::Point(width,img.rows/3),cv::Scalar(0),8);
        cv::line(img,cv::Point(0,2*img.rows/3),cv::Point(width,2*img.rows/3),cv::Scalar(0),8);
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

    BoardScheme(std::string channel) : it_(nh_)
    {
        image_pub_ = it_.advertise(channel.c_str(), 1);
        sub = nh_.subscribe("/new_board", 1, &BoardScheme::publish_draw_board, this);

        height=600;
        width =1024;

        n_cols = 3;
        n_rows = 3;

        white = cv::Scalar(255,255,255);
        red   = cv::Scalar(180, 40, 40);
        blue  = cv::Scalar( 40, 40,150);

    }
};

const char BoardScheme::WINDOW[] = "Tic Tac Toe Board Status";
}


int main(int argc, char** argv)
{
    std::string channel = "/board_scheme";
    if (argc>1)
    {
        channel=std::string(argv[1]);
    }
    ros::init(argc, argv, "board_display");
    ttt::BoardScheme bd(channel);
    ros::spin();
    return 0;
}
