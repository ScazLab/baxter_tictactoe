#ifndef __BOARD_STATE_SENSING__
#define __BOARD_STATE_SENSING__

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <iostream>

#include "baxterTictactoe/tictactoe_utils.h"
#include "baxter_tictactoe/DefineCells.h"
#include "baxter_tictactoe/ScanState.h"

class BoardState
{
private:
    ros::NodeHandle node_handle;
    ros::Publisher  board_publisher;

    ros::ServiceClient cells_client;

    image_transport::ImageTransport image_transport;
    image_transport::Subscriber     image_subscriber;

    std::string board_config;

    ttt::Board board;
    ttt::Cell cell;

    double area_threshold;

    ttt::hsvColorRange hsv_red;
    ttt::hsvColorRange hsv_blue;

    bool doShow;

    // Last TTT board state message sent. Used to avoid the publication of the same board state messages.
    // It publishes the board only if its state changes.
    baxter_tictactoe::MsgBoard last_msg_board;

public:
    BoardState(bool _show = "false");
    ~BoardState();

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    std::string intToString( const int a );
};

#endif //__BOARD_STATE_SENSING__
