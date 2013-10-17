
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
    typedef boost::array<uint8_t, NUMBER_OF_CELLS> TTT_State_type;

    ros::NodeHandle _nh; //! ROS node handle
    ros::Subscriber _ttt_state_sub; //! subscriber to receive the messages comming from the board state sensor

    ThreadSafeVariable<TTT_State_type> _ttt_state; //! it stores the state of the board. It is the same type than the data received when a new TTT board state is detected (ttt_board_sensor::ttt_board.data)

    ThreadSafeVariable<unsigned short int> _number_of_tokens_on_board; //! It stores the total number of cells on the board. This is used to detect the end of the oponent's turn

    t_Cell_State _robot_color;   //! It represents the color of the tokens the robot is playing with.
    t_Cell_State _oponent_color; //! It represents the color of the tokens the oponent is playing with.

    /**
     * It handles the message published when the state of a cell has changed. The new TTT board state is stored in the thread-safe private attribute called _ttt_state.
     * \param msg the message with the new TTT state, i.e. the states of each of the cells
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
            r = qrand() % NUMBER_OF_CELLS + 1; //random number between 1 and NUMBER_OF_CELLS
            ROS_INFO_STREAM("randon number = " << r);
        }
//        while(_ttt_state.get()[r-1]==empty || _ttt_state.get()[r-1]==undefined);
        while(_ttt_state.get()[r-1]==ttt_board_sensor::ttt_board::EMPTY || _ttt_state.get()[r-1]==ttt_board_sensor::ttt_board::UNDEFINED);

        ROS_INFO_STREAM("Random cell with number " << r);
        return r;
    }

public:
//    TTT_Brain(std::string strategy, t_Cell_State robot_color=blue) : _robot_color(robot_color), _move_commander("place_token", true) // true causes the client to spin its own thread
    TTT_Brain(std::string strategy, t_Cell_State robot_color=ttt_board_sensor::ttt_board::BLUE) : _robot_color(robot_color), _move_commander("place_token", true) // true causes the client to spin its own thread
    {
        _number_of_tokens_on_board.set(0);

//        ROS_ASSERT_MSG(_robot_color==blue || _robot_color==red, "Wrong color for robot's tokens");
//        _oponent_color=_robot_color==blue?red:blue;
        ROS_ASSERT_MSG(_robot_color==ttt_board_sensor::ttt_board::BLUE || _robot_color==ttt_board_sensor::ttt_board::RED, "Wrong color for robot's tokens");
        _oponent_color=(_robot_color==ttt_board_sensor::ttt_board::BLUE? ttt_board_sensor::ttt_board::RED : ttt_board_sensor::ttt_board::BLUE);

        TTT_State_type aux;
//        aux.assign(undefined);
        aux.assign(ttt_board_sensor::ttt_board::UNDEFINED);
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
     * @return The return value is between 1 (first row, first column) and NUMBER_OF_CELLS (last raw, last column).
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

    /**
     * This function checks if there are 3 cell_color tokens in a row, which means that the game is over.
     * In a 3x3 board there are 8 possible convinations to get 3 tokens in a row. We explore all of them.
     * @param cell_color It represents the color of the tokens in the row we are searching for.
     * @return True in case of a 3 token row is found, false otherwise.
     **/
    bool three_in_a_row(const t_Cell_State& cell_color)
    {
        ROS_DEBUG("@three in a row");
//        if(cell_color!=blue && cell_color!=red) return false; // There are only red and blue tokens
        if(cell_color!=ttt_board_sensor::ttt_board::BLUE && cell_color!=ttt_board_sensor::ttt_board::RED) return false; // There are only red and blue tokens

        if(_ttt_state.get()[1]==cell_color && _ttt_state.get()[2]==cell_color && _ttt_state.get()[3]==cell_color) return true; // 3 in a row in the first row
        if(_ttt_state.get()[4]==cell_color && _ttt_state.get()[5]==cell_color && _ttt_state.get()[6]==cell_color) return true; // 3 in a row in the second row
        if(_ttt_state.get()[7]==cell_color && _ttt_state.get()[8]==cell_color && _ttt_state.get()[9]==cell_color) return true; // 3 in a row in the third row
        if(_ttt_state.get()[1]==cell_color && _ttt_state.get()[4]==cell_color && _ttt_state.get()[7]==cell_color) return true; // 3 in a row in the first column
        if(_ttt_state.get()[2]==cell_color && _ttt_state.get()[5]==cell_color && _ttt_state.get()[8]==cell_color) return true; // 3 in a row in the second column
        if(_ttt_state.get()[3]==cell_color && _ttt_state.get()[6]==cell_color && _ttt_state.get()[9]==cell_color) return true; // 3 in a row in the third column
        if(_ttt_state.get()[1]==cell_color && _ttt_state.get()[5]==cell_color && _ttt_state.get()[9]==cell_color) return true; // 3 in a row in the first diagonal
        if(_ttt_state.get()[3]==cell_color && _ttt_state.get()[5]==cell_color && _ttt_state.get()[7]==cell_color) return true; // 3 in a row in the second diagonal

        return false;
    }

    inline t_Cell_State get_robot_color()
    {
        return _robot_color;
    }

    inline t_Cell_State get_oponent_color()
    {
        return _oponent_color;
    }

    inline std::string get_robot_color_str()
    {
        return cell_state_to_str(_robot_color);
    }

    inline std::string get_oponent_color_str()
    {
        return cell_state_to_str(_oponent_color);
    }

    /**
     * This function returns the winner of the game.
     * @return 0 if there is not winner, 1 if the winner is the robot, or 2 if the winner is the oponent.
     **/
    inline unsigned short int get_winner()
    {
        if (this->three_in_a_row(_robot_color)) return 1;
        if (this->three_in_a_row(_oponent_color)) return 2;
        return 0;
    }

    /**
     * This function blocks until the oponent has done his move.
     * This is detected considering the number of tokens on the board. The function waits until the number of tokens in the board increases.
     **/
    void wait_for_oponent_turn()
    {
        unsigned short int n_tokens=this->get_number_of_tokens_on_board();
        unsigned short int aux_n_tokens=n_tokens;
        while(n_tokens<=aux_n_tokens) // Waiting for my turn: the participant has to place one token, so we wait until the number of tokens on the board increases.
        {
            ros::Duration(1).sleep();
            n_tokens=this->get_number_of_tokens_on_board();
        }
    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ttt_brain");
    ROS_INFO("Playing RANDOM TIC TAC TOE");
    ttt::TTT_Brain brain("random");
    ROS_INFO_STREAM("Robot plays with " << brain.get_robot_color_str() << " and the oponent with " << brain.get_oponent_color_str());

    //TODO: Move the arms to a neutral position

    short int winner=0; // no winner
    while ((winner=brain.get_winner())==0)
    {        
        int cell_to_move = brain.get_next_move();
        actionlib::SimpleClientGoalState goal_state = brain.execute_move(cell_to_move);
        if (goal_state==actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Last move successfully performed. Waiting for the participant's move");
            brain.wait_for_oponent_turn(); // Waiting for my turn: the participant has to place one token, so we wait until the number of tokens on the board increases.
        } else {
            ROS_ERROR_STREAM("Last move has not succeded. Goal state " << goal_state.toString().c_str());
            //What do we do now?
        }        

        //ros::spinOnce();
        //ros::Duration(1).sleep();// 1sec
    }

    switch(winner)
    {
    case 1:
        ROS_INFO("ROBOT's VICTORY!");
        break;
    case 2:
        ROS_INFO("OPONENT's VICTORY!");
        break;
    default:
        ROS_ERROR("Who wins?????");
    }

    //TODO: Move the arms to a neutral position

    return 0;
}
