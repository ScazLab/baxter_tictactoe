
#include <ros/ros.h>

#include "ttt_board_sensor/ttt_board.h"

namespace ttt
{


class TTT_Brain
{
private:

    ros::NodeHandle _nh; //! ROS node handle
    ros::Subscriber _ttt_state_sub; //! subscriber to receive the messages comming from the board state sensor

    /**
     *
     * \param msg the message with the new TTT state, i.e. the states of each of the 9 cells
     **/
    void new_ttt_state(const ttt_board_sensor::ttt_boardConstPtr & msg);

public:
    TTT_Brain()
    {

        _ttt_state_sub = _nh.subscribe("/new_board", 1, &TTT_Brain::new_ttt_state, this); //receiving data about the TTT board state every time it changes

    }

    ~TTT_Brain()
    {
    }


};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ttt_brain");
    ttt::TTT_Brain brain;
    ros::spin();
    return 0;
}
