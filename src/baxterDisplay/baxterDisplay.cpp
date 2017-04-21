#include <signal.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "baxter_tictactoe/tictactoe_utils.h"
#include "baxter_tictactoe/MsgBoard.h"

using namespace baxter_tictactoe;

class BaxterDisplay
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

    ros::Subscriber board_sub;

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

    std::string yale_logo_file;

    cv::Mat drawBoard(const MsgBoard& msg)
    {
        cv::Mat img(height,width,CV_8UC3,white);
        drawLines(img);

        for (size_t i = 0; i < msg.cells.size(); ++i)
        {
            if (msg.cells[i].state != MsgCell::EMPTY)
            {
                drawCell(img, i, msg.cells[i].state);
            }
        }

        return img;
    }

    void drawCell(cv::Mat& img, size_t cell_number, const std::string &cell_data)
    {
        cv::Point top_left =topLeftCorner(cell_number);
        cv::Point diag_incr(cols_cell_img*1/12,rows_cell_img*1/12);

        cv::rectangle(img,top_left+2*diag_incr,top_left+10*diag_incr,
                      cell_data==MsgCell::RED?red:blue,CV_FILLED);

        if (cell_data==MsgCell::RED)
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
            cv::line(img, cv::Point(cross_top_left.x,cross_bottom_right.y),
                          cv::Point(cross_bottom_right.x,cross_top_left.y), white, 20);
        }
    }

    cv::Point topLeftCorner(int cell_number)
    {
        unsigned short int col=cell_number%n_cols;
        unsigned short int row=cell_number/n_rows;
        cv::Point result(cols_cell_img*col+board_bottom_left.x,
                         rows_cell_img*row+board_bottom_left.y);

        ROS_DEBUG("Cell #%i bottom left corner: %i %i\n",cell_number,result.x,result.y);

        return result;
    }

    void drawLines(cv::Mat& img)
    {
        cv::line(img,topLeftCorner(3),topLeftCorner(5)+cv::Point(cols_cell_img,0),cv::Scalar(0),8);
        cv::line(img,topLeftCorner(6),topLeftCorner(8)+cv::Point(cols_cell_img,0),cv::Scalar(0),8);
        cv::line(img,topLeftCorner(1),topLeftCorner(7)+cv::Point(0,rows_cell_img),cv::Scalar(0),8);
        cv::line(img,topLeftCorner(2),topLeftCorner(8)+cv::Point(0,rows_cell_img),cv::Scalar(0),8);

        return;
    }

    void newBoardCb(const MsgBoard& msg)
    {
        cv::Mat img_board = drawBoard(msg);
        publishImage(img_board);

        return;
    }

    void publishImage(cv::Mat _img)
    {
        cv_bridge::CvImage out_msg;
        out_msg.encoding     = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image        = _img; // Your cv::Mat

        image_pub_.publish(out_msg.toImageMsg());

        return;
    }

public:

    void drawYaleLogo()
    {
        if (yale_logo_file != "")
        {
            ROS_INFO("Publishing Yale logo..");
            cv::Mat img;
            img = cv::imread(yale_logo_file, CV_LOAD_IMAGE_COLOR);   // Read the file

            if(not img.data )  // Check for invalid input
            {
                ROS_ERROR("Yale logo file not found: %s", yale_logo_file.c_str());
                return;
            }

            // cv::imshow("test", img);
            // cv::waitKey(39);

            publishImage(img);
        }

        return;
    }

    BaxterDisplay() : it_(nh_)
    {
        image_pub_ = it_.advertise("baxter_display", 3, true);
        board_sub  = nh_.subscribe("board_state", 3, &BaxterDisplay::newBoardCb, this);

        nh_.param<std::string>("baxter_tictactoe/yale_logo_file", yale_logo_file, "");

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
        ROS_DEBUG("Cols_cell_img = %u\t Rows_cell_img = %u", cols_cell_img, rows_cell_img);

        white = cv::Scalar(255,255,255);
        blue  = cv::Scalar(180, 40, 40);  // REMEMBER that this is in BGR color code!!
        red   = cv::Scalar( 40, 40,150);  // REMEMBER that this is in BGR color code!!

        // This delay is there to be able to publish the yale logo
        ros::Duration(0.1).sleep();
        drawYaleLogo();
    }

    ~BaxterDisplay()
    {

    }
};

sig_atomic_t sigflag = 0;

void mySigintHandler(int sig)
{
    // putting this flag to true will break the while loop in the main function,
    // and will publish one last message from the baxter display.
    sigflag = 1;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "baxter_display", ros::init_options::NoSigintHandler);
    BaxterDisplay bd;

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    ros::Rate r(50.0);

    while (true)
    {
        if (sigflag == 1)
        {
            bd.drawYaleLogo();
            break;
        }
        ros::spinOnce();
        r.sleep();
    }

    // All the default sigint handler does is call shutdown().
    // We call it here after the yale logo has been published.
    ros::shutdown();

    return 0;
}
