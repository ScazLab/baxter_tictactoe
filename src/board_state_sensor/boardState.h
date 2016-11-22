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

class BoardState : public ROSThreadImage
{
private:
    ros::Publisher  board_publisher;

    ros::ServiceClient cells_client;

    std::string board_config;

    ttt::Board board;
    ttt::Cell cell;

    double area_threshold;

    ttt::hsvColorRange hsv_red;
    ttt::hsvColorRange hsv_blue;

    bool doShow;

    ros::Rate r;

    std::string intToString( const int a );

    // Last TTT board state message sent. Used to avoid the publication of the same board state messages.
    // It publishes the board only if its state changes.
    baxter_tictactoe::MsgBoard last_msg_board;

protected:
    void InternalThreadEntry();

public:
    BoardState(std::string _name, bool _show = "false");
    ~BoardState();
};

#endif //__BOARD_STATE_SENSING__
