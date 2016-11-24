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
#include "baxter_tictactoe/TTTBrainState.h"

#define STATE_INIT      0
#define STATE_CALIB     1
#define STATE_READY     2

#define LARGEST_IDX         1
#define NEXT_LARGEST_IDX    2

class BoardState : public ROSThreadImage
{
private:
    ros::Publisher          board_state_pub;
    ros::Subscriber         brain_state_sub;
    image_transport::Publisher      img_pub;

    ttt::Board board;
    ttt::Cell cell;

    double area_threshold;

    ttt::hsvColorRange hsv_red;
    ttt::hsvColorRange hsv_blue;

    bool doShow;

    ros::Rate r;

    int board_state; // State of the board
    int brain_state; // state of the demo

    cv::Scalar col_empty;
    cv::Scalar   col_red;
    cv::Scalar  col_blue;

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
     * Callback to get the state of the demo.
     **/
    void brainStateCb(const baxter_tictactoe::TTTBrainState & msg);

    /**
     * Checks if the detected board satisfies some empirical rule. For the time being, the only
     * thing checked is if the centroids of the cells are not within the contour of any other cell.
     *
     * @return true/false if the board is "sane"
     */
    bool isBoardSane();

protected:
    void InternalThreadEntry();

public:
    BoardState(std::string _name, bool _show = "false");
    ~BoardState();
};

#endif //__BOARD_STATE_SENSING__
