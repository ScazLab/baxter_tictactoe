#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "baxter_tictactoe/ttt_board.h"

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
    int minimum; // the minimum between height and width

    cv::Point board_bottom_left; // the bottom left corner of the board

    int n_cols;
    int n_rows;

    unsigned int cols_cell_img;
    unsigned int rows_cell_img;

    cv::Scalar white;
    cv::Scalar   red;
    cv::Scalar  blue;

    cv::Mat draw_board(const baxter_tictactoe::ttt_board::ConstPtr& msg) const
    {
        ROS_DEBUG("@display_board");

        cv::Mat img(height,width,CV_8UC3,white);

        // 3x3 tic tac toe board
        unsigned short int col, row;

        this->draw_lines(img);

        for (size_t i = 0; i < msg->data.size(); ++i) {
            ROS_DEBUG_STREAM("Processing Cell " << i);
            if(msg->data[i]>0)
            {
                col=i%n_cols;
                row=i/n_rows;
                cv::Point center_cell(cols_cell_img/2 + cols_cell_img*col, rows_cell_img/2 + rows_cell_img*row);
                cv::Point bottom_left_corner;
                cv::Point   top_right_corner(cols_cell_img*5/6 + cols_cell_img*col, rows_cell_img*5/6 + rows_cell_img*row);
                ROS_DEBUG_STREAM("Center point in cell " << i << " (" << center_cell.x << "," << center_cell.y << ")");

                draw_tile(img,i,msg->data[i]);
            }
        }
        return img;
    }

    void draw_tile(cv::Mat& img, size_t cell_number, unsigned int cell_data) const
    {
        cv::Point top_left =compute_cell_top_left(cell_number);
        cv::Point diag_incr(cols_cell_img*1/12,rows_cell_img*1/12);

        cv::rectangle(img,top_left+2*diag_incr,top_left+10*diag_incr,cell_data==baxter_tictactoe::ttt_board::RED?red:blue,CV_FILLED);

        if (cell_data==baxter_tictactoe::ttt_board::RED)
        {
            cv::Point center=top_left+6*diag_incr;
            cv::circle(img,center,rows_cell_img*3/12,white,16);
        }
        else
        {
            // Draw the white cross inside the rectangle
            cv::Point cross_top_left    =top_left+3*diag_incr;
            cv::Point cross_bottom_right=top_left+9*diag_incr;

            cv::line(img, cross_top_left, cross_bottom_right, white,20);
            cv::line(img,
                     cv::Point(cross_top_left.x,cross_bottom_right.y),
                     cv::Point(cross_bottom_right.x,cross_top_left.y),
                     white,20);
        }
    }

    cv::Point compute_cell_top_left(int cell_number) const
    {
        unsigned short int col=cell_number%n_cols;
        unsigned short int row=cell_number/n_rows;
        cv::Point result(cols_cell_img*col+board_bottom_left.x, rows_cell_img*row+board_bottom_left.y);

        ROS_DEBUG("Cell #%i bottom left corner: %i %i\n",cell_number,result.x,result.y);

        return result;
    }

    void draw_lines(cv::Mat& img) const
    {
        cv::line(img,compute_cell_top_left(3),compute_cell_top_left(5)+cv::Point(cols_cell_img,0),cv::Scalar(0),8);
        cv::line(img,compute_cell_top_left(6),compute_cell_top_left(8)+cv::Point(cols_cell_img,0),cv::Scalar(0),8);
        cv::line(img,compute_cell_top_left(1),compute_cell_top_left(7)+cv::Point(0,rows_cell_img),cv::Scalar(0),8);
        cv::line(img,compute_cell_top_left(2),compute_cell_top_left(8)+cv::Point(0,rows_cell_img),cv::Scalar(0),8);

    }

    void show_board_temporary(cv::Mat& img, int msec) const
    {
        cv::namedWindow(BoardScheme::WINDOW);
        cv::imshow(BoardScheme::WINDOW,img);
        cv::waitKey(msec);
        cv::destroyAllWindows();
    }

    void publish_draw_board(const baxter_tictactoe::ttt_board::ConstPtr& msg) const
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

        // Let's find the minimum between height and width
        int minimum = height>width?width:height;

        board_bottom_left.x = (width -minimum)/2;
        board_bottom_left.y = (height-minimum)/2;

        n_cols = 3;
        n_rows = 3;

        cols_cell_img = minimum/n_cols;
        rows_cell_img = minimum/n_rows;
        ROS_DEBUG_STREAM("cols_cell_img=" << cols_cell_img << " rows_cell_img=" << rows_cell_img);

        white = cv::Scalar(255,255,255);
        blue  = cv::Scalar(180, 40, 40);  // REMEMBER that this is in BGR color code!!
        red   = cv::Scalar( 40, 40,150);  // REMEMBER that this is in BGR color code!!

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
