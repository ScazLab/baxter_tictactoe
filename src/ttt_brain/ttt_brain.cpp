#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sound_play/sound_play.h>
#include <QtGlobal> //required for qrand
#include <QDateTime>
#include <QFile>
#include <QTextStream>

#include <baxter_tictactoe/MsgCell.h>
#include <baxter_tictactoe/MsgBoard.h>
#include "baxterTictactoe/tictactoe_utils.h"

#include "T_ThreadSafe.h"
#include <tictactoe/PlaceTokenAction.h>
#include "vacuum_gripper/vacuum_gripper.h"

#include <tictactoe/SetTrajectoryType.h>

namespace ttt
{

bool operator==(boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells1, boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells2)
{
    for(int i = 0; i < cells1.size(); i++){
        if(cells1[i].state != cells2[i].state){
            return false;
        }
    }
    return true;
}

bool operator!=(boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells1, boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells2)
{
    return !(cells1==cells2);
}

class TTT_Brain
{
private:
    typedef actionlib::SimpleActionClient<tictactoe::PlaceTokenAction> Place_Token_Client_type;
    typedef boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> TTT_State_type;
    // typedef std::vector<baxter_tictactoe::MsgCell> TTT_State_type;

    ros::NodeHandle _nh; // ROS node handle
    ros::Subscriber _ttt_state_sub; // subscriber to receive the messages 
                                    // coming from the board state sensor

    ThreadSafeVariable<TTT_State_type> _ttt_state; // it stores the state of the board. It is 
                                                   // the same type than the data received when a 
                                                   // new TTT board state is detected 
                                                   // (baxter_tictactoe::MsgBoard::cells)

    ThreadSafeVariable<unsigned short int> _number_of_tokens_on_board; // It stores the total number
                                                                       // of cells on the board. This 
                                                                       // is used to detect the end of 
                                                                       // the opponent's turn

    cellState _robot_color;   // It represents the color of the tokens the robot is playing with.
    cellState _opponent_color; // It represents the color of the tokens the opponent is playing with.

    sound_play::SoundClient _voice_synthesizer; //! This is used for generating voice utterances.

    /**
     * It handles the message published when the state of a cell has changed. The new TTT board state
     * is stored in the thread-safe private attribute called _ttt_state.
     * \param msg the message with the new TTT state, i.e. the states of each of the cells
     **/
    void new_ttt_state(const baxter_tictactoe::MsgBoardConstPtr & msg)
    {
        if ((msg->cells)!=_ttt_state.get()) {
            ROS_DEBUG_STREAM("[TTT_Brain] New TTT board state detected at ." << msg->header.stamp);
            _ttt_state.set(msg->cells);
        }
        else{
            ROS_DEBUG("[TTT_Brain] Same board received. Do not update.");
        }
    }

    ros::ServiceClient _clnt_movement_type;

    std::string _voice_type; //! It determines the type of voice.

    bool _smooth; //! It determines the type of movements: smooth and natural or more mechanistic and robotic.

    bool _can_cheat; //! It determines if the robot can cheat or not.

    Place_Token_Client_type _move_commander; /* This is the incharge of sending the command to 
                                                place a new token in a cell of the TTT board */

    int (TTT_Brain::*_choose_next_move)(bool& cheating); /* This a pointer to the function that 
                                                            decides the next move. We use a pointer 
                                                            because we could have different strategies. */

    bool has_cheated;
    /**
     * It determines randomly the next empty cell to place a token.
     * \param cheating It indicates if cheating has happened.
     * \return an integer representing the cell where to place the next token
     **/
    int random_move(bool& cheating)
    {
        cheating=false;
        int r;
        TTT_State_type aux = _ttt_state.get();
        do {
            r = qrand() % NUMBER_OF_CELLS + 1; //random number between 1 and NUMBER_OF_CELLS
            ROS_DEBUG("[TTT_Brain] Cell %d is in state %d ==? %d || %d", r, aux[r-1].state, 
                      baxter_tictactoe::MsgCell::EMPTY, baxter_tictactoe::MsgCell::UNDEFINED);
        }
        while(aux[r-1].state!=empty && aux[r-1].state!=undefined);

        ROS_INFO_STREAM("[TTT_Brain] Random move to cell with number " << r);
        return r;
    }

    /**
     * It determines the next cell to place a token. It will try to win in this turn, even if it has to cheat 
     * placing a token on top of an opponent's token. If the robot can win without breaking the rules, it will
     * do it. Otherwise, if it can win cheating, it will do it. It will block opponent's victory. If it cannot 
     * win anyway, it will randomly choose a cell.
     * \param cheating It indicates if cheating has happened.
     * \return an integer representing the cell where to place the next token
     **/
    int cheating_to_win_random_move(bool& cheating)
    {
        int next_cell_id=-1;
        if ((next_cell_id = victory_move()) != -1) return next_cell_id;
        if ((next_cell_id = cheating_move()) != -1)
        {
            cheating=true;
            return next_cell_id;
        }
        if ((next_cell_id = defensive_move()) != -1) return next_cell_id;
        return random_move(cheating);
    }

    /**
     * It determines the next cell to place a token. It tries to win in its turn but it does no cheat.
     * If the robot can win, it will do it. Otherwise, if it cannot win in this turn anyway, it will 
     * try to block the opponent's victory. If this is not needed, it will randomly choose a cell.
     * \param cheating It indicates if cheating has happened.
     * \return an integer representing the cell where to place the next token
     **/
    int winning_defensive_random_move(bool& cheating)
    {
        cheating=false;
        int next_cell_id=-1;
        if ((next_cell_id = victory_move()) != -1) return next_cell_id;
        if ((next_cell_id = defensive_move()) != -1) return next_cell_id;
        return random_move(cheating);
    }

    /*
     * It determines the next cell to place a token. It always will try to win in this turn, even if 
     * there is an opponent's token already in that cell. If the robot can win without breaking the 
     * rules, it will do it. Otherwise, if it can win cheating, it will do it. If it cannot win in 
     * this turn anyway, it will try to block the opponent's victory. If this is not needed it will 
     * randomly choose a cell.
     * @param cheating It indicates if cheating has happened.
     * @return an integer representing the cell where to place the next token
     **/
    int smart_cheating_random_move(bool& cheating)
    {
        int next_cell_id=-1;
        if ((next_cell_id = victory_move()) != -1) return next_cell_id;
        if ((next_cell_id = cheating_move()) != -1)
        {
            cheating=true;
            return next_cell_id;
        }
        if ((next_cell_id = defensive_move()) != -1) return next_cell_id;
        return random_move(cheating);
    }

    /*
     * It determines if the robot can win in this turn cheating, i.e. placing a token in a cell 
     * occupied with an opponent's token. 
     * @return -1 if the robot cannot win in the next move, 
     * or an integer corresponding to the cell where to place the next token to win, even if there 
     * is an opponent's token in that cell. The cell ids are between 1 (first row, first column) 
     * and NUMBER_OF_CELLS (last raw, last column).
     */
    int cheating_move()
    {
        TTT_State_type aux = _ttt_state.get();
        uint8_t cell_state = undefined;
        for (size_t i = 0; i < NUMBER_OF_CELLS; ++i) {
            if (aux[i].state==_opponent_color)
            {
                cell_state=aux[i].state;
                aux[i].state=_robot_color;
                if (TTT_Brain::three_in_a_row(_robot_color, aux))
                {
                    ROS_INFO_STREAM("[TTT_Brain] Cheating move to cell with number " << i+1);
                    has_cheated=true;
                    return i+1;
                }
                aux[i].state=cell_state;
            }
        }
        ROS_INFO("[TTT_Brain] No chance to win in this turn");
        return -1;
    }

    /**
     * It determines if the opponent can win in the next move.
     * \return -1 if the opponent cannot win in the next move, or an integer corresponding 
     * to the first found cell where an opponent's token can be placed to win the game. 
     * The cell ids are between 1 (first row, first column) and NUMBER_OF_CELLS (last raw, last column).
     **/
    int defensive_move()
    {
        TTT_State_type aux = _ttt_state.get();
        uint8_t cell_state = undefined;
        for (size_t i = 0; i < NUMBER_OF_CELLS; ++i) {
            if (aux[i].state==empty || aux[i].state==undefined)
            {
                cell_state = aux[i].state;
                aux[i].state = _opponent_color;
                if (TTT_Brain::three_in_a_row(_opponent_color, aux))
                {
                    ROS_INFO_STREAM("[TTT_Brain] Defensive move to cell with number " << i+1);
                    return i+1;
                }
                aux[i].state=cell_state;
            }
        }
        ROS_INFO("[TTT_Brain] No chance to lose in this turn");
        return -1;
    }

    /**
     * It determines if the robot can win in the next move.
     * \return -1 if the robot cannot win in the next move, or an integer corresponding 
     * to the first found cell where a robot's token can be placed to win the game. The 
     * cell ids are between 1 (first row, first column) and NUMBER_OF_CELLS (last raw, last column).
     **/
    int victory_move()
    {
        TTT_State_type aux = _ttt_state.get();
        uint8_t cell_state = undefined;
        for (size_t i = 0; i < NUMBER_OF_CELLS; ++i) {
            if (aux[i].state==empty || aux[i].state==undefined)
            {
                cell_state = aux[i].state;
                aux[i].state = _robot_color;
                if (TTT_Brain::three_in_a_row(_robot_color, aux))
                {
                    ROS_INFO_STREAM("[TTT_Brain] Victory move to cell with number " << i+1);
                    return i+1;
                }
                aux[i].state = cell_state;
            }
        }
        ROS_INFO("[TTT_Brain] No chance to win in this turn");
        return -1;
    }

    /**
     * It sets the kind of movements: smooth or mechanistic.
     * \return true if there is no error, or false otherwise.
     **/
    bool set_smooth_movements(bool b)
    {
        _smooth=b;
        tictactoe::SetTrajectoryType srv;
        srv.request.smooth=(_smooth?true:false);

        if(_clnt_movement_type.call(srv))
        {
            if (!srv.response.error)
            {
                ROS_INFO_STREAM("[TTT_Brain] Movements set to " << (srv.request.smooth?"smooth":"mechanistic"));
                return true;
            }
            ROS_ERROR_STREAM("[TTT_Brain] Error setting movements to " << (srv.request.smooth?"smooth":"mechanistic"));
            return false;
        }
        ROS_ERROR("[TTT_Brain] Failed to call service set_movement_type");
        return false;
    }

    /**
     * It sets the cheating field
     * \return true if there is can cheats, or false otherwise.
     **/
    inline void set_can_cheat(bool b)
    {
        _can_cheat=b;
    }


public:

    TTT_Brain(cellState robot_color=blue, std::string strategy="random") : _robot_color(robot_color), _move_commander("place_token", true) // true causes the client to spin its own thread
    {
        _number_of_tokens_on_board.set(0);

        ROS_ASSERT_MSG(_nh.hasParam("voice"),"No voice found in the parameter server!");
        ROS_ASSERT_MSG(_nh.getParam("voice",_voice_type), "The voice parameter not retreive from the parameter server");
        ROS_INFO_STREAM("[TTT_Brain] Using voice " << _voice_type);

        ROS_ASSERT_MSG(_nh.hasParam("cheating"),"No cheating parameter found in the parameter server!");
        ROS_ASSERT_MSG(_nh.getParam("cheating",_can_cheat), "The cheating possibility has not been retreived from the parameter server");
        ROS_INFO_STREAM("[TTT_Brain] Robot " << (_can_cheat?"can":"cannot") << " cheat");

        ROS_ASSERT_MSG(_nh.hasParam("smooth"),"No sort of movements found in the parameter server!");
        ROS_ASSERT_MSG(_nh.getParam("smooth",_smooth), "The sort of movements has not been retreived from the parameter server");
        ROS_INFO_STREAM("[TTT_Brain] Using " << _smooth << " movements");
        _clnt_movement_type = _nh.serviceClient<tictactoe::SetTrajectoryType>("set_movement_type");
        this->set_smooth_movements(_smooth);

        ROS_ASSERT_MSG(_robot_color==blue || _robot_color==red, "Wrong color for robot's tokens");
        _opponent_color=_robot_color==blue?red:blue;

        TTT_State_type aux; // aux is an array of 9 MsgCells 
        for(int i = 0; i < aux.size(); i++){
            aux[i].state = baxter_tictactoe::MsgCell::UNDEFINED;
        }

        // aux.assign(undefined);
        _ttt_state.set(aux);    // initialy the state for all cells in the TTT board is undefined

        _ttt_state_sub = _nh.subscribe("/new_board", 1, &TTT_Brain::new_ttt_state, this); //receiving data about the TTT board state every time it changes        

        qsrand(ros::Time::now().nsec);
        this->set_strategy(strategy);

        has_cheated=false;

        ROS_INFO("[TTT_Brain] Waiting for TTT Move Maker action server to start.");
        ROS_ASSERT_MSG(_move_commander.waitForServer(ros::Duration(10.0)),"TTT Move Maker action server doesn't found");
        ROS_INFO("[TTT_Brain] TTT Move Maker action server is started. We are ready for sending goals.");
    }

    ~TTT_Brain()
    {
    }

    /**
     * Returns the cell where the next token is gonna be placed.
     * @param cheating It indicates if cheating happens
     * @return The return value is between 1 (first row, first column) 
     * and NUMBER_OF_CELLS (last raw, last column).
     **/
    int get_next_move(bool& cheating) {
        return (this->*_choose_next_move)(cheating);
    }

    /**
     * It indicates if the robot can cheats.
     * \return true if robot can cheats, or false if it does not.
     **/
    inline bool can_cheat()
    {
        return _can_cheat;
    }

    /**
     * It indicates if the robot moves smoothly. If not, it moves mechanisticaly.
     * \return true if robot uses smooth movements, or false if it uses mechanistic movements.
     **/
    inline bool is_smooth_movements()
    {
        return _smooth;
    }


    /**
     * This function sends the command to place a token in a particular cell.
     * @param cell_to_move value between 1 (first row, first column) and 
     * 9 (last raw, last column) that points where the next token will be placed
     * @return The state information for the goal selected. 
     * Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, 
     * SUCCEEDED, LOST. In case that the cell to move does not exist, the returned value will be LOST.
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
            ROS_ERROR("[TTT_Brain] Unknown cell to move, %d",cell_to_move);
            return actionlib::SimpleClientGoalState::LOST;
        }
        _move_commander.sendGoal(goal);
        _move_commander.waitForResult(ros::Duration(40.0)); //wait 40s for the action to return
        bool _success = (_move_commander.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
        //bool finished_before_timeout = _move_commander.waitForResult(ros::Duration(40.0)); //wait 40s for the action to return
        //if (finished_before_timeout) ROS_INFO_STREAM("[TTT_Brain] Action moving to " << goal.cell << " finished successfully!");
        if (_success) ROS_INFO_STREAM("[TTT_Brain] Action moving to " << goal.cell << " finished successfully!");
        else ROS_WARN_STREAM("[TTT_Brain] Action moving to " << goal.cell << " did not finish before the time out.");
        return _move_commander.getState();
    }

    /**
     * This function counts the total number of tokens on the board. 
     * That is, the number of cells that are not empty or undefined.
     * @return The number of cells where there is a red or blue token.
     **/
    unsigned short int get_number_of_tokens_on_board()
    {
        unsigned short int counter=0;
        TTT_State_type aux = _ttt_state.get();
        for(int i = 0; i < aux.size(); i++)
        {
            if(aux[i].state!=empty && aux[i].state!=undefined) counter++;
        }
        return counter;
    }

    /**
     * This function counts the number of a particular type of tokens 
     * on the board. That is, the number of cells that occupied by these kind of tikens.
     * @param token_type The kind of tokens we are counting
     * @return The number of cells where there is a token_type token.
     **/
    unsigned short int get_number_of_tokens_on_board(cellState token_type)
    {
        unsigned short int counter=0;
        TTT_State_type aux = _ttt_state.get();
        for(int i = 0; i < aux.size(); i++)
        {
           if(aux[i].state==token_type) counter++;
        }
        return counter;
    }

    /**
     * This function checks if there are 3 cell_color tokens in a row, which means that the game is over.
     * In a 3x3 board there are 8 possible convinations to get 3 tokens in a row. We explore all of them.
     * @param cell_color It represents the color of the tokens in the row we are searching for.
     * @param tttboard TTT board where searching for three tokens of the same color in a row.
     * @return True in case of a 3 token row is found, false otherwise.
     **/
    bool three_in_a_row(const cellState& cell_color, const TTT_State_type& tttboard)
    {
        ROS_DEBUG("[TTT_Brain] @three in a row");
        if(cell_color!=blue && cell_color!=red) return false; // There are only red and blue tokens

        if(tttboard[0].state==cell_color && tttboard[1].state==cell_color && tttboard[2].state==cell_color) return true; // 3 in a row in the first row
        if(tttboard[3].state==cell_color && tttboard[4].state==cell_color && tttboard[5].state==cell_color) return true; // 3 in a row in the second row
        if(tttboard[6].state==cell_color && tttboard[7].state==cell_color && tttboard[8].state==cell_color) return true; // 3 in a row in the third row
        if(tttboard[0].state==cell_color && tttboard[3].state==cell_color && tttboard[6].state==cell_color) return true; // 3 in a row in the first column
        if(tttboard[1].state==cell_color && tttboard[4].state==cell_color && tttboard[7].state==cell_color) return true; // 3 in a row in the second column
        if(tttboard[2].state==cell_color && tttboard[5].state==cell_color && tttboard[8].state==cell_color) return true; // 3 in a row in the third column
        if(tttboard[0].state==cell_color && tttboard[4].state==cell_color && tttboard[8].state==cell_color) return true; // 3 in a row in the first diagonal
        if(tttboard[2].state==cell_color && tttboard[4].state==cell_color && tttboard[6].state==cell_color) return true; // 3 in a row in the second diagonal

        return false;
    }

    /**
     * This function checks if there are 3 cell_color tokens in a row in the current TTT board, 
     * which means that the game is over. In a 3x3 board there are 8 possible convinations to 
     * get 3 tokens in a row. We explore all of them.
     * @param cell_color It represents the color of the tokens in the row we are searching for.
     * @return True in case of a 3 token row is found, false otherwise.
     **/
    bool three_in_a_row(const cellState& cell_color)
    {
        return this->three_in_a_row(cell_color,_ttt_state.get());
    }

    inline cellState get_robot_color()
    {
        return _robot_color;
    }

    inline cellState get_opponent_color()
    {
        return _opponent_color;
    }

    inline std::string get_robot_color_str()
    {
        return cell_state_to_str(_robot_color);
    }

    inline std::string get_opponent_color_str()
    {
        return cell_state_to_str(_opponent_color);
    }

    /**
     * This function returns the winner of the game.
     * @return 0 if there is not winner, 1 if the winner is the robot, or 2 if the winner is the opponent.
     **/
    inline unsigned short int get_winner()
    {
        if (TTT_Brain::three_in_a_row(_robot_color,_ttt_state.get())) return 1;
        if (TTT_Brain::three_in_a_row(_opponent_color, _ttt_state.get())) return 2;
        return 0;
    }

    /**
     * This function blocks until the opponent has done his move.
     * This is detected considering the number of the opponent's 
     * tokens on the board. The function waits until the number 
     * of opponent's tokens in the board increases.
     * @param number of opponent's token at the beginning
     **/
    void wait_for_opponent_turn(const uint8_t& n_opponent_tokens)
    {        
        uint8_t aux_n_opponent_tokens=n_opponent_tokens;
        while(aux_n_opponent_tokens<=n_opponent_tokens) /* Waiting for my turn: the participant 
                                                           has to place one token, so we wait until
                                                           the number opponent's tokens increase. */
        {
//            ros::Duration(1).sleep();
            ROS_WARN("[TTT_Brain] Press ENTER when the opponent's turn is done");
            std::cin.get();
            aux_n_opponent_tokens=this->get_number_of_tokens_on_board(_opponent_color);
        }
    }

    /**
     * This function indicates if all cells are occupied by tokens.
     * @return True if all cells are occupied, false otherwise.
     **/
    bool full_board()
    {
        TTT_State_type aux = _ttt_state.get();
        for(int i = 0; i < aux.size(); i++){
            if(aux[i].state==empty || aux[i].state==undefined) /* we consider the case where cells are 
                                                                  undefined because is needed in the first
                                                                  loop when all cells are undefined */
            return false;
        }
        return true;
    }

    /**
     * This function synthetizes sentence and waits t seconds.
     * @param sentence string corresponding with the sentence to synthetize.
     * @param t number of seconds to block.
     **/
    void say_sentence(std::string sentence, double t)
    {
        _voice_synthesizer.say(sentence, _voice_type);
        if(_nh.ok()) ros::Duration(t).sleep();
        else ROS_WARN("[TTT_Brain] Nodel handle is not ok. Don't sleeping during the synthetitation.");
    }

    /**
     * This function sets an strategy.
     * @param strategy string corresponding with a particular strategy.
     **/
    void set_strategy(std::string strategy)
    {

        if (strategy=="random")
        {
            _choose_next_move=&TTT_Brain::random_move;
            ROS_INFO("[TTT_Brain] New strategy: Baxter randomly places tokens");
        } else if (strategy=="smart")
        {
            _choose_next_move=&TTT_Brain::winning_defensive_random_move;
            ROS_INFO("[TTT_Brain] New strategy: Baxter randomly places tokens but it wins, if there is a chance, or blocks opponent's victory");
        }
        else if (strategy=="cheating")
        {
            _choose_next_move=&TTT_Brain::cheating_to_win_random_move;
            ROS_INFO("[TTT_Brain] New strategy: Baxter randomly places tokens but it wins, if there is a chance even if cheating is required");
        }
        else if (strategy=="smart-cheating")
        {
            _choose_next_move=&TTT_Brain::smart_cheating_random_move;
            ROS_INFO("[TTT_Brain] New strategy: Baxter randomly places tokens but it wins, if there is a chance even if cheating is required, or blocks opponent's victory");
        }
        else {
            ROS_ERROR_STREAM(strategy << " is not an available strategy");
        }
    }

    unsigned short int play_one_game(bool& cheating)
    {
        bool robot_turn=true;
        unsigned short int winner=0; // no winner
        has_cheated=false;

        // this->say_sentence("Please place the blue tokens in the blue box on the right side of the board",6);
        // this->say_sentence(" and the red tokens in the red square on the left side of the board",6);
        this->say_sentence("I start the game.",2);
        this->set_smooth_movements(_smooth);

        ROS_WARN("[TTT_Brain] PRESS ENTER TO START THE GAME");
        std::cin.get();        
        uint8_t n_opponent_tokens=0;
        // uint8_t n_robot_tokens=0;
        while ((winner=this->get_winner())==0 && !this->full_board())
        {
            if (robot_turn) // Robot's turn
            {
                // n_robot_tokens=this->get_number_of_tokens_on_board(_robot_color); //number of robot's tokens befor the robot's turn
                n_opponent_tokens=this->get_number_of_tokens_on_board(_opponent_color); //number of opponent's tokens befor the robot's turn
                this->say_sentence("It is my turn",0.3);
                int cell_to_move = this->get_next_move(cheating);
                ROS_DEBUG_STREAM("[TTT_Brain] Robot's token to " << cell_to_move);
                actionlib::SimpleClientGoalState goal_state = this->execute_move(cell_to_move);
                if (goal_state==actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("[TTT_Brain] Last move successfully performed.");
                } else {
                    ROS_ERROR_STREAM("[TTT_Brain] Last move has not succeded. Goal state " << goal_state.toString().c_str() << " is not guaranteed.");
                    //What do we do now?
                }
                //the number of robot tokens haven't increased so we try again.
                // if(n_robot_tokens==this->get_number_of_tokens_on_board(_robot_color)) 
                // {
                //     ROS_ERROR("[TTT_Brain] Somehow there is still %ud robot tokens.",n_robot_tokens);
                //     continue;
                // }
            }
            else // Participant's turn
            {
                ROS_INFO("[TTT_Brain] Waiting for the participant's move.");
                this->say_sentence("It is your turn",0.1);
                this->wait_for_opponent_turn(n_opponent_tokens); /* Waiting for my turn: the participant 
                                                                    has to place one token, so we wait 
                                                                    until the number of the opponent's 
                                                                    tokens increases. */
            }
            robot_turn=!robot_turn;
        }

        switch(winner)
        {
        case 1:
            ROS_INFO("[TTT_Brain] ROBOT's VICTORY!");
            if (has_cheated)
            {
                this->say_sentence("You humans are so easy to beat!",5);
            }
            this->say_sentence("I win", 3);
            break;
        case 2:
            ROS_INFO("[TTT_Brain] OPPONENT's VICTORY!");
            this->say_sentence("You won this time",4);
            break;
        default:
            ROS_INFO("[TTT_Brain] TIE!");
            this->say_sentence("That's a tie. I will win next time.",8);
            winner=3;
        }
        return winner;
    }

    static uint NUM_GAMES;
    static uint CHEATING_GAME_A;
    static uint CHEATING_GAME_B;
};

uint TTT_Brain::NUM_GAMES=3;
uint TTT_Brain::CHEATING_GAME_A=2;
uint TTT_Brain::CHEATING_GAME_B=3;

}

int main(int argc, char** argv)
{    
    ROS_INFO("[TTT_Brain] Playing TIC TAC TOE");
    ros::init(argc, argv, "ttt_brain");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ttt::TTT_Brain brain; //random strategy by default
    brain.set_strategy("smart");
    ros::Duration(1).sleep(); //this second is needed in order to use the voice at the beggining
    ROS_INFO_STREAM("[TTT_Brain] Robot plays with " << brain.get_robot_color_str() << " and the opponent with " << brain.get_opponent_color_str());

    ROS_WARN("[TTT_Brain] PRESS ENTER TO START WITH A NEW PARTICIPANT");
    std::cin.get();
    brain.say_sentence("  Welcome!  Let's play Tic Tac Toe.",4);
    brain.say_sentence("Do not grasp your token before I say that it is your turn",5);

    uint robot_victories=0, participant_victories=0, ties=0;
    uint i=1;
    bool cheating=false;
    ROS_INFO_STREAM("[TTT_Brain] Let's play " << ttt::TTT_Brain::NUM_GAMES << " times Tic Tac Toe");
    unsigned short game_result=0;

    while(i<=ttt::TTT_Brain::NUM_GAMES)
    {
        ROS_INFO_STREAM("[TTT_Brain] Game " << i);
        if ((i==ttt::TTT_Brain::CHEATING_GAME_A || i==ttt::TTT_Brain::CHEATING_GAME_B) && brain.can_cheat()) //In the fourth game, Baxter cheats
        {
            brain.set_strategy("smart-cheating");
        }
        game_result=brain.play_one_game(cheating);
        switch(game_result)
        {
        case 1: robot_victories++;
            break;
        case 2: participant_victories++;
            break;
        case 3: ties++;
            break;
        default: ROS_ERROR_STREAM("[TTT_Brain] Unexpected return value for the game: " << game_result << " ???");
        }
        if ((i==ttt::TTT_Brain::CHEATING_GAME_A || i==ttt::TTT_Brain::CHEATING_GAME_B) && brain.can_cheat())
        {
            if (!cheating) {
                ROS_INFO("[TTT_Brain] Game ended but no cheating. Game counter does not increase.");
                continue;
            } else {
                ROS_INFO("[TTT_Brain] Game ended cheating. Back to random strategy");
                brain.set_strategy("smart");
            }
        }
        i++;
    }

    brain.say_sentence("Game over. It was my pleasure to win over you. Thanks for being so human.",10);

    ROS_INFO_STREAM("[TTT_Brain] Baxter " << robot_victories << " - Human " << participant_victories << " - Ties " << ties);    
    std::string filepath = "/home/alvaro/ttt " + QDateTime::currentDateTime().toString().toStdString() + ".txt";
    QFile file(filepath.c_str());
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    out << (brain.can_cheat()? "cheats" : "does not cheat") << "\n";
    out << (brain.is_smooth_movements()? "smooth" : "mechanistic") << "\n";
    out << "Baxter " << robot_victories << " - Human " << participant_victories << " - Ties " << ties;
    file.close();

    return 0;
}
