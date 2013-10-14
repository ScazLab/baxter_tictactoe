
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <QtGlobal> //required for qrand

#include "ttt_board_sensor/ttt_board.h"
#include "ttt_definitions.h"
#include "src/utils/T_ThreadSafe.h"
#include "tictactoe/PlaceTokenAction.h"

namespace ttt
{


class TTT_Brain
{
private:

    typedef actionlib::SimpleActionClient<tictactoe::PlaceTokenAction> Place_Token_Client_type;
    typedef boost::array<uint8_t, 9> TTT_State_type;

    ros::NodeHandle _nh; //! ROS node handle
    ros::Subscriber _ttt_state_sub; //! subscriber to receive the messages comming from the board state sensor

    ThreadSafeVariable<TTT_State_type> _ttt_state; //! it stores the state of the board. It is the same type than the data received when a new TTT board state is detected (ttt_board_sensor::ttt_board.data)

//    boost::array<uint8_t, 9> _ttt_state; //! it stores the state of the board. It is the same type than the data received when a new TTT board state is detected (ttt_board_sensor::ttt_board.data)

    /**
     * It handles the message published when the state of a cell has changed. The new TTT board state is stored in the thread-safe private attribute called _ttt_state.
     * \param msg the message with the new TTT state, i.e. the states of each of the 9 cells
     **/
    void new_ttt_state(const ttt_board_sensor::ttt_boardConstPtr & msg)
    {
        if (msg->data!=_ttt_state.get()) {
            ROS_INFO("New TTT board state detected.");
            _ttt_state.set(msg->data);
        }
    }

    Place_Token_Client_type move_commander; //! This is the incharge of sending the command to place a new token in a cell of the TTT board

    int (TTT_Brain::*_choose_next_move)();

    int random_move()
    {
        int r;
        do {
            r = qrand() % 9 + 1; //random number between 1 and 9
        }
        while(_ttt_state.get()[r]==empty || _ttt_state.get()[r]==undefined);
        return r;
    }

public:
    TTT_Brain(std::string strategy) : move_commander("place_token", true) // true causes the client to spin its own thread
    {
        TTT_State_type aux;
        aux.assign(undefined);
        _ttt_state.set(aux);    // initialy the state for all cells in the TTT board is undefined
//        _ttt_state.assign(undefined);
        _ttt_state_sub = _nh.subscribe("/new_board", 1, &TTT_Brain::new_ttt_state, this); //receiving data about the TTT board state every time it changes        

        if (strategy=="random") {
            _choose_next_move=&TTT_Brain::random_move;
        } else {
            ROS_ERROR_STREAM(strategy << " is not an available strategy");
        }
    }

    ~TTT_Brain()
    {
    }

    int get_next_move() {
        return (this->*_choose_next_move)();
    }


};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ttt_brain");
    ROS_INFO("Playing RANDOM TIC TAC TOE");
    ttt::TTT_Brain brain("random");
    bool victory=false;

    ros::Rate r(1); // 1 hz = 1 loop/sec
    while (!victory)
    {
        int cell_to_move = brain.get_next_move();

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
