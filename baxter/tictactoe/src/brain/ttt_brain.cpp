
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

    ThreadSafeVariable<unsigned short int> _number_of_tokens_on_board;
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

    Place_Token_Client_type _move_commander; //! This is the incharge of sending the command to place a new token in a cell of the TTT board

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
    TTT_Brain(std::string strategy) : _move_commander("place_token", true) // true causes the client to spin its own thread
    {
        _number_of_tokens_on_board.set(0);

        TTT_State_type aux;
        aux.assign(undefined);
        _ttt_state.set(aux);    // initialy the state for all cells in the TTT board is undefined

        _ttt_state_sub = _nh.subscribe("/new_board", 1, &TTT_Brain::new_ttt_state, this); //receiving data about the TTT board state every time it changes        

        if (strategy=="random") {
            _choose_next_move=&TTT_Brain::random_move;
        } else {
            ROS_ERROR_STREAM(strategy << " is not an available strategy");
        }

        ROS_INFO("Waiting for TTT Move Maker action server to start.");
        ROS_ASSERT_MSG(_move_commander.waitForServer(ros::Duration(10.0)),"TTT Move Maker action server doesn't found");
        ROS_INFO("TTT Move Maker action server is started. We are ready for sending goals.");
    }

    ~TTT_Brain()
    {
    }

    /**
     * Returns the cell where the next token is gonna be placed.
     * @return The return value is between 1 (first row, first column) and 9 (last raw, last column).
     **/
    int get_next_move() {
        return (this->*_choose_next_move)();
    }

    /**
     * This function sends the command to place a token in a particular cell.
     * @param cell_to_move value between 1 (first row, first column) and 9 (last raw, last column) that points where the next token will be placed
     * @return The state information for the goal selected. Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST. In case that the cell to move does not exist, the returned value will be LOST.
     **/
    actionlib::SimpleClientGoalState execute_move(int cell_to_move)
    {
        tictactoe::PlaceTokenGoal goal;
        switch(cell_to_move)
        {
        case 1:goal.cell="1x1"; break;
        case 2:goal.cell="1x2"; break;
        case 3:goal.cell="1x3"; break;
        case 4:goal.cell="2x1"; break;
        case 5:goal.cell="2x2"; break;
        case 6:goal.cell="2x3"; break;
        case 7:goal.cell="3x1"; break;
        case 8:goal.cell="3x2"; break;
        case 9:goal.cell="3x3"; break;
        default:
            ROS_ERROR("Unknown cell to move, %d",cell_to_move);
            return actionlib::SimpleClientGoalState::LOST;
        }
        _move_commander.sendGoal(goal);
        bool finished_before_timeout = _move_commander.waitForResult(ros::Duration(40.0)); //wait 40s for the action to return
        if (finished_before_timeout) ROS_INFO_STREAM("Action moving to " << goal.cell << " finished successfully!");
        else ROS_WARN_STREAM("Action moving to " << goal.cell << " did not finish before the time out.");
        return _move_commander.getState();
    }

    unsigned short int get_number_of_tokens_on_board()
    {
        unsigned short int counter=0;
        foreach (unsigned short int c, _ttt_state.get()) {
            if(c!=0) counter++;
        }
        return counter;
    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ttt_brain");
    ROS_INFO("Playing RANDOM TIC TAC TOE");
    ttt::TTT_Brain brain("random");
    bool victory=false;
    unsigned short int n_tokens=brain.get_number_of_tokens_on_board();
    unsigned short int aux_n_tokens;

    while (!victory)
    {
        int cell_to_move = brain.get_next_move();
        actionlib::SimpleClientGoalState goal_state = brain.execute_move(cell_to_move);
        if (goal_state==actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Last move successfully performed");
            //wait for my turn
            while(n_tokens<aux_n_tokens) //We sleep until the number of tokens on the board increases.
            {
                ros::Duration(1).sleep();
            }

        } else {
            ROS_ERROR_STREAM("Last move has not succeded. Goal state " << goal_state.toString().c_str());
            //What do we do now?
        }

        n_tokens=brain.get_number_of_tokens_on_board();

        ros::spinOnce();
        ros::Duration(1).sleep();// 1sec
    }
    return 0;
}
