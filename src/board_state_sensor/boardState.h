#ifndef __BOARD_STATE_SENSING__
#define __BOARD_STATE_SENSING__

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <iostream>


#include "robot_interface/ros_thread_image.h"
#include "baxterTictactoe/tictactoe_utils.h"
#include "baxter_tictactoe/DefineCells.h"
#include "baxter_tictactoe/ScanState.h"

#define STATE_INIT      0
#define STATE_CALIB     1

#define LARGEST_IDX         1
#define NEXT_LARGEST_IDX    2

class BoardState : public ROSThreadImage
{
private:
    ros::Publisher  board_publisher;

    ros::ServiceClient cells_client;
    ros::ServiceServer service;

    std::string board_config;

    ttt::Board board;
    ttt::Cell cell;

    double area_threshold;

    ttt::hsvColorRange hsv_red;
    ttt::hsvColorRange hsv_blue;

    bool doShow;

    ros::Rate r;

    pthread_mutex_t mutex_b;

    // Last TTT board state message sent. Used to avoid the publication of the same board state messages.
    // It publishes the board only if its state changes.
    baxter_tictactoe::MsgBoard last_msg_board;

    int state;

    /**
     * @param      vector (i.e array) of contours, type indicating whether largest or
     *             next largest area is to be found where (LARGEST = largest area,
     *             NEXT_LARGEST = next largest area)
     *
     * @return     index of the contour with the largest area or the next largest area
     */
    int getIthIndex(std::vector<std::vector<cv::Point> > contours, int ith);

    std::string intToString( const int a );

    static bool ascendingY(std::vector<cv::Point> i, std::vector<cv::Point> j);

    static bool ascendingX(std::vector<cv::Point> i, std::vector<cv::Point> j);

    /**
     * service that provides data on the defined contours/boundaries of the board's cells
     *
     * @param      request variables and response variables
     *
     * @return     returns true when function is succesfully executed
     */
    bool serviceCb(baxter_tictactoe::DefineCells::Request  &req,
                   baxter_tictactoe::DefineCells::Response &res);

protected:
    void InternalThreadEntry();

public:
    BoardState(std::string _name, bool _show = "false");
    ~BoardState();
};

#endif //__BOARD_STATE_SENSING__
