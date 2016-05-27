#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "baxterTictactoe/tictactoe_utils.h"
#include "baxter_tictactoe/DefineCells.h"
#include "ttt_board_sensor/ttt_board.h"

class BoardState
{

private:

    ros::NodeHandle node_handle;
    ros::Publisher  board_publisher;
    ros::ServiceClient client;
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
    ttt_board_sensor::ttt_board last_msg_board;

    void init();

public:

    BoardState();
    BoardState(bool _show);
    ~BoardState();

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};
