#include "tictactoeBrain.h"

using namespace ttt;
using namespace std;
using namespace baxter_tictactoe;

bool ttt::operator==(boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells1,
                     boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells2)
{
    for(int i = 0; i < cells1.size(); i++)
    {
        if(cells1[i].state != cells2[i].state)
        {
            return false;
        }
    }
    return true;
}

bool ttt::operator!=(boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells1,
                     boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells2)
{
    return !(cells1==cells2);
}

tictactoeBrain::tictactoeBrain(cellState robot_color, std::string strategy) : r(100),
                               _robot_color(robot_color), _setup(false), _move_commander("place_token", true),
                               leftArmCtrl("tictactoe", "left"), rightArmCtrl("tictactoe", "right")
{
    _ttt_state_sub = _nh.subscribe("baxter_tictactoe/new_board", 1, &tictactoeBrain::tttStateCb, this);
    _scan_server   = _nh.advertiseService("baxter_tictactoe/ready_scan", &tictactoeBrain::scanState, this);

    _number_of_tokens_on_board.set(0);

    ROS_ASSERT_MSG(_nh.hasParam("voice"),"No voice found in the parameter server!");
    ROS_ASSERT_MSG(_nh.getParam("voice",_voice_type), "The voice parameter not retrieve from the parameter server");
    ROS_INFO("Using voice %s", _voice_type.c_str());

    ROS_ASSERT_MSG(_nh.hasParam("cheating"),"No cheating parameter found in the parameter server!");
    ROS_ASSERT_MSG(_nh.getParam("cheating",cheating), "The cheating possibility has not been retrieved from the parameter server");
    ROS_INFO("Robot %s cheat", cheating?"does":"does not");

    ROS_ASSERT_MSG(_robot_color==blue || _robot_color==red, "Wrong color for robot's tokens");
    _opponent_color=_robot_color==blue?red:blue;

    leftArmCtrl.scanBoard();
    while(ros::ok() && leftArmCtrl.getState() != SCANNED)
    {
        r.sleep();
    }

    TTT_State_type aux; // aux is an array of 9 MsgCells
    for(int i = 0; i < aux.size(); i++)
    {
        aux[i].state = baxter_tictactoe::MsgCell::UNDEFINED;
    }

    _ttt_state.set(aux);    // initially the state for all cells in the TTT board is undefined

    qsrand(ros::Time::now().nsec);
    set_strategy(strategy);

    has_cheated=false;

    _setup = true; // indicates whether board has been scanned by arm camera
                   // (signal to boardStateSensing that the board's position
                   // is locked and ready to scanned by head camera)

    while(ros::ok())
    {
        TTT_State_type arr = _ttt_state.get();
        if(arr != aux)
        {
            break;
        }

        ROS_WARN("Board was not detected. Make sure board is within the head camera's view and is not obscured. Press ENTER afterwards.");
        r.sleep();
    }
}

tictactoeBrain::~tictactoeBrain()
{

}

bool tictactoeBrain::scanState(ScanState::Request &req, ScanState::Response &res)
{
    res.state = _setup == true ? true : false;
    return true;
}

void tictactoeBrain::tttStateCb(const baxter_tictactoe::MsgBoardConstPtr &msg)
{
    if (msg->cells != _ttt_state.get())
    {
        ROS_DEBUG("New TTT board state received");
        _ttt_state.set(msg->cells);
    }
}

int tictactoeBrain::random_move(bool& cheating)
{
    cheating=false;
    int r;
    TTT_State_type aux = _ttt_state.get();
    do {
        r = qrand() % NUMBER_OF_CELLS + 1; //random number between 1 and NUMBER_OF_CELLS
        ROS_DEBUG("Cell %d is in state %d ==? %d || %d", r, aux[r-1].state,
                  baxter_tictactoe::MsgCell::EMPTY, baxter_tictactoe::MsgCell::UNDEFINED);
    }
    while(aux[r-1].state!=empty && aux[r-1].state!=undefined);

    ROS_INFO("Random move to cell with number %i", r);
    return r;
}

int tictactoeBrain::cheating_to_win_random_move(bool& cheating)
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

int tictactoeBrain::winning_defensive_random_move(bool& cheating)
{
    cheating=false;
    int next_cell_id=-1;
    if ((next_cell_id = victory_move()) != -1) return next_cell_id;
    if ((next_cell_id = defensive_move()) != -1) return next_cell_id;
    return random_move(cheating);
}

int tictactoeBrain::smart_cheating_random_move(bool& cheating)
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

int tictactoeBrain::cheating_move()
{
    TTT_State_type aux = _ttt_state.get();
    uint8_t cell_state = undefined;
    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i) {
        if (aux[i].state==_opponent_color)
        {
            cell_state=aux[i].state;
            aux[i].state=_robot_color;
            if (three_in_a_row(_robot_color, aux))
            {
                ROS_INFO("Cheating move to cell with number %lu", i+1);
                has_cheated=true;
                return i+1;
            }
            aux[i].state=cell_state;
        }
    }
    ROS_INFO("No chance to win in this turn");
    return -1;
}

int tictactoeBrain::defensive_move()
{
    TTT_State_type aux = _ttt_state.get();
    uint8_t cell_state = undefined;
    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i) {
        if (aux[i].state==empty || aux[i].state==undefined)
        {
            cell_state = aux[i].state;
            aux[i].state = _opponent_color;
            if (three_in_a_row(_opponent_color, aux))
            {
                ROS_INFO("Defensive move to cell with number %lu", i+1);
                return i+1;
            }
            aux[i].state=cell_state;
        }
    }
    ROS_INFO("No chance to lose in this turn");
    return -1;
}

int tictactoeBrain::victory_move()
{
    TTT_State_type aux = _ttt_state.get();
    uint8_t cell_state = undefined;

    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i)
    {
        if (aux[i].state==empty || aux[i].state==undefined)
        {
            cell_state = aux[i].state;
            aux[i].state = _robot_color;
            if (three_in_a_row(_robot_color, aux))
            {
                ROS_INFO("Victory move to cell with number %lu", i+1);
                return i+1;
            }
            aux[i].state = cell_state;
        }
    }

    ROS_INFO("No chance to win in this turn");
    return -1;
}

int tictactoeBrain::get_next_move(bool& cheating)
{
    return (this->*_choose_next_move)(cheating);
}

actionlib::SimpleClientGoalState tictactoeBrain::execute_move(int cell_to_move)
{
    baxter_tictactoe::PlaceTokenGoal goal;
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
            ROS_ERROR("Unknown cell %d to move to",cell_to_move);
            return actionlib::SimpleClientGoalState::LOST;
    }

    _move_commander.sendGoal(goal);
    _move_commander.waitForResult(ros::Duration(40.0)); //wait 40s for the action to return
    bool _success = (_move_commander.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);

    if (_success) ROS_INFO("Action moving to %s finished successfully", goal.cell.c_str());
    else ROS_WARN("Action moving to %s did not finish before the time out.", goal.cell.c_str());

    return _move_commander.getState();
}

unsigned short int tictactoeBrain::get_number_of_tokens_on_board()
{
    unsigned short int counter=0;
    TTT_State_type aux = _ttt_state.get();
    for(int i = 0; i < aux.size(); i++)
    {
        if(aux[i].state!=empty && aux[i].state!=undefined) counter++;
    }
    return counter;
}

unsigned short int tictactoeBrain::get_number_of_tokens_on_board(cellState token_type)
{
    unsigned short int counter=0;
    TTT_State_type aux = _ttt_state.get();
    for(int i = 0; i < aux.size(); i++)
    {
       if(aux[i].state==token_type) counter++;
    }
    return counter;
}

bool tictactoeBrain::three_in_a_row(const cellState& color, const TTT_State_type& b)
{
    if(color!=blue && color!=red) return false; // There are only red and blue tokens

    if(b[0].state==color && b[1].state==color && b[2].state==color) return true; // first row
    if(b[3].state==color && b[4].state==color && b[5].state==color) return true; // second row
    if(b[6].state==color && b[7].state==color && b[8].state==color) return true; // third row
    if(b[0].state==color && b[3].state==color && b[6].state==color) return true; // first column
    if(b[1].state==color && b[4].state==color && b[7].state==color) return true; // second column
    if(b[2].state==color && b[5].state==color && b[8].state==color) return true; // third column
    if(b[0].state==color && b[4].state==color && b[8].state==color) return true; // first diagonal
    if(b[2].state==color && b[4].state==color && b[6].state==color) return true; // second diagonal

    return false;
}

unsigned short int tictactoeBrain::get_winner()
{
    if (three_in_a_row(_robot_color,_ttt_state.get()))      return 1;
    if (three_in_a_row(_opponent_color, _ttt_state.get()))  return 2;
    return 0;
}

void tictactoeBrain::wait_for_opponent_turn(const uint8_t& n_opponent_tokens)
{
    uint8_t aux_n_opponent_tokens=n_opponent_tokens;
    while(aux_n_opponent_tokens<=n_opponent_tokens) /* Waiting for my turn: the participant
                                                       has to place one token, so we wait until
                                                       the number opponent's tokens increase. */
    {
        ROS_WARN("Press ENTER when the opponent's turn is done");
        std::cin.get();
        aux_n_opponent_tokens=get_number_of_tokens_on_board(_opponent_color);
    }
}

bool tictactoeBrain::is_board_full()
{
    TTT_State_type aux = _ttt_state.get();
    for(int i = 0; i < aux.size(); i++)
    {
        if(aux[i].state==empty || aux[i].state==undefined) /* we consider the case where cells are
                                                              undefined because is needed in the first
                                                              loop when all cells are undefined */
        return false;
    }
    return true;
}

void tictactoeBrain::say_sentence(std::string sentence, double t)
{
    _voice_synthesizer.say(sentence, _voice_type);
    ros::Duration(t).sleep();
}

void tictactoeBrain::set_strategy(std::string strategy)
{
    if (strategy=="random")
    {
        _choose_next_move=&tictactoeBrain::random_move;
        ROS_INFO("[strategy] Baxter randomly places tokens");
    }
    else if (strategy=="smart")
    {
        _choose_next_move=&tictactoeBrain::winning_defensive_random_move;
        ROS_INFO("[strategy] Baxter randomly places tokens but it wins if there is a chance, or blocks opponent's victory");
    }
    else if (strategy=="cheating")
    {
        _choose_next_move=&tictactoeBrain::cheating_to_win_random_move;
        ROS_INFO("[strategy] Baxter randomly places tokens but it wins if there is a chance even if cheating is required");
    }
    else if (strategy=="smart-cheating")
    {
        _choose_next_move=&tictactoeBrain::smart_cheating_random_move;
        ROS_INFO("[strategy] Baxter randomly places tokens but it wins if there is a chance "
                            "even if cheating is required, or blocks opponent's victory");
    }
    else
    {
        ROS_ERROR("%s is not an available strategy", strategy.c_str());
    }
}

unsigned short int tictactoeBrain::play_one_game(bool& cheating)
{
    bool robot_turn=true;
    unsigned short int winner=0; // no winner
    has_cheated=false;

    say_sentence("I start the game.",2);

    ROS_WARN("PRESS ENTER TO START THE GAME");
    std::cin.get();
    uint8_t n_opponent_tokens=0;
    // uint8_t n_robot_tokens=0;
    while ((winner=get_winner())==0 && !is_board_full())
    {
        if (robot_turn) // Robot's turn
        {
            ROS_INFO("robot turn");

            // n_robot_tokens=get_number_of_tokens_on_board(_robot_color); //number of robot's tokens befor the robot's turn
            n_opponent_tokens=get_number_of_tokens_on_board(_opponent_color); //number of opponent's tokens befor the robot's turn
            say_sentence("It is my turn", 0.3);
            int cell_to_move = get_next_move(cheating);
            ROS_DEBUG("Robot's token to %i", cell_to_move);
            ROS_INFO("get next move");

            leftArmCtrl.pickUpToken();
            while(ros::ok() && leftArmCtrl.getState() != PICK_UP)
            {
                r.sleep();
            }

            leftArmCtrl.putDownToken(cell_to_move);
            while(ros::ok() && leftArmCtrl.getState() != PUT_DOWN)
            {
                r.sleep();
            }
        }
        else // Participant's turn
        {
            ROS_INFO("Waiting for the participant's move.");
            say_sentence("It is your turn",0.1);
            wait_for_opponent_turn(n_opponent_tokens); // Waiting for my turn: the participant has to place one token,
                                                       // so we wait until the number of the opponent's tokens increases.
            ROS_INFO("after participant move");
        }
        robot_turn=!robot_turn;
    }

    switch(winner)
    {
    case 1:
        ROS_INFO("ROBOT's VICTORY!");
        if (has_cheated)
        {
            say_sentence("You humans are so easy to beat!",5);
        }
        say_sentence("I won", 3);
        break;
    case 2:
        ROS_INFO("OPPONENT's VICTORY!");
        say_sentence("You won this time",4);
        break;
    default:
        ROS_INFO("TIE!");
        say_sentence("That's a tie. I will win next time.",8);
        winner=3;
    }
    return winner;
}

