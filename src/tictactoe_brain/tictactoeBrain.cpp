#include "tictactoeBrain.h"

#include <QFile> // this is for qrand (but we should use something else)

using namespace ttt;
using namespace std;
using namespace baxter_tictactoe;

bool ttt::operator==(boost::array<MsgCell, NUMBER_OF_CELLS> cells1,
                     boost::array<MsgCell, NUMBER_OF_CELLS> cells2)
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

bool ttt::operator!=(boost::array<MsgCell, NUMBER_OF_CELLS> cells1,
                     boost::array<MsgCell, NUMBER_OF_CELLS> cells2)
{
    return !(cells1==cells2);
}

tictactoeBrain::tictactoeBrain(std::string _name, std::string _strategy) :
                               r(100), _setup(false), _nh(name), spinner(4),
                               leftArmCtrl("ttt_controller", "left"),
                               rightArmCtrl("ttt_controller", "right")
{
    pthread_mutexattr_t _mutex_attr;
    pthread_mutexattr_init(&_mutex_attr);
    pthread_mutexattr_settype(&_mutex_attr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(&_mutex_brainstate, &_mutex_attr);

    printf("\n");
    boardState_sub = _nh.subscribe("baxter_tictactoe/board_state", SUBSCRIBER_BUFFER,
                                    &tictactoeBrain::boardStateCb, this);
    tttBrain_pub   = _nh.advertise<TTTBrainState>("baxter_tictactoe/ttt_brain_state", 1);

    set_brain_state(TTTBrainState::INIT);
    brainstate_timer = _nh.createTimer(ros::Duration(0.1), &tictactoeBrain::publishTTTBrainStateCb, this, false);

    _nh.param<string>("ttt_brain/voice", _voice_type, VOICE);
    ROS_INFO("Using voice %s", _voice_type.c_str());

    _nh.param<bool>("ttt_brain/cheating", cheating, true);
    ROS_INFO("Robot %s cheat", cheating?"does":"does not");

    // string robot_color;
    // _nh.param<string>("ttt_brain/robot_color",  robot_color, "blue");
    // ROS_ASSERT_MSG(robot_color!="blue", "robot_color should be set to blue in the parameter server. "
    //                            "If you want to use red, be willing to spend some time in coding it!");
    _robot_color=blue;
    _opponent_color=_robot_color==blue?red:blue;

    set_brain_state(TTTBrainState::CALIB);
    leftArmCtrl.startAction(ACTION_SCAN);
    while(ros::ok() && leftArmCtrl.getState() != DONE)
    {
        r.sleep();
    }
    set_brain_state(TTTBrainState::READY);

    TTT_Board_State aux; // aux is an array of 9 MsgCells
    for(int i = 0; i < aux.size(); i++)
    {
        aux[i].state = MsgCell::UNDEFINED;
    }

    boardState.set(aux);    // initially the state for all cells in the TTT board is undefined

    qsrand(ros::Time::now().nsec);
    set_strategy(_strategy);

    has_cheated=false;

    _setup = true; // indicates whether board has been scanned by arm camera
                   // (signal to boardStateSensing that the board's position
                   // is locked and ready to be scanned by head camera)

    while(ros::ok())
    {
        TTT_Board_State arr = boardState.get();
        if(arr != aux)
        {
            break;
        }

        ROS_WARN_THROTTLE(1, "Board was not detected. Make sure it is within view and not obscured.");
        r.sleep();
    }
}

void tictactoeBrain::publishTTTBrainStateCb(const ros::TimerEvent&)
{
    pthread_mutex_lock(&_mutex_brainstate);
    tttBrain_pub.publish(s);
    pthread_mutex_unlock(&_mutex_brainstate);
}

void tictactoeBrain::set_brain_state(int state)
{
    pthread_mutex_lock(&_mutex_brainstate);
    s.state = state;
    pthread_mutex_unlock(&_mutex_brainstate);
}

void tictactoeBrain::boardStateCb(const MsgBoardConstPtr &msg)
{
    if (msg->cells != boardState.get())
    {
        ROS_DEBUG("New TTT board state received");
        boardState.set(msg->cells);
    }
}

int tictactoeBrain::random_move(bool& cheating)
{
    cheating=false;
    int r;
    TTT_Board_State aux = boardState.get();
    do {
        r = qrand() % NUMBER_OF_CELLS + 1; //random number between 1 and NUMBER_OF_CELLS
        ROS_DEBUG("Cell %d is in state %d ==? %d || %d", r, aux[r-1].state,
                  MsgCell::EMPTY, MsgCell::UNDEFINED);
    }
    while(aux[r-1].state!=empty && aux[r-1].state!=undefined);

    ROS_INFO("Random move to cell # %i", r);
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

int tictactoeBrain::cheating_move()
{
    TTT_Board_State aux = boardState.get();
    uint8_t cell_state = undefined;
    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i)
    {
        if (aux[i].state==_opponent_color)
        {
            cell_state=aux[i].state;
            aux[i].state=_robot_color;
            if (three_in_a_row(_robot_color, aux))
            {
                ROS_INFO("Cheating move to cell # %lu", i+1);
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
    TTT_Board_State aux = boardState.get();
    uint8_t cell_state = undefined;
    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i)
    {
        if (aux[i].state==empty || aux[i].state==undefined)
        {
            cell_state = aux[i].state;
            aux[i].state = _opponent_color;
            if (three_in_a_row(_opponent_color, aux))
            {
                ROS_INFO("Defensive move to cell # %lu", i+1);
                return i+1;
            }
            aux[i].state=cell_state;
        }
    }
    // ROS_INFO("No chance to lose in this turn");
    return -1;
}

int tictactoeBrain::victory_move()
{
    TTT_Board_State aux = boardState.get();
    uint8_t cell_state = undefined;

    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i)
    {
        if (aux[i].state==empty || aux[i].state==undefined)
        {
            cell_state = aux[i].state;
            aux[i].state = _robot_color;
            if (three_in_a_row(_robot_color, aux))
            {
                ROS_INFO("Victory move to cell # %lu", i+1);
                return i+1;
            }
            aux[i].state = cell_state;
        }
    }

    // ROS_INFO("No chance to win in this turn");
    return -1;
}

int tictactoeBrain::get_next_move(bool& cheating)
{
    return (this->*_choose_next_move)(cheating);
}

unsigned short int tictactoeBrain::get_num_tokens()
{
    unsigned short int counter=0;
    TTT_Board_State aux = boardState.get();
    for(int i = 0; i < aux.size(); i++)
    {
        if(aux[i].state!=empty && aux[i].state!=undefined) counter++;
    }
    return counter;
}

unsigned short int tictactoeBrain::get_num_tokens(cellState token_type)
{
    unsigned short int counter=0;
    TTT_Board_State aux = boardState.get();
    for(int i = 0; i < aux.size(); i++)
    {
       if(aux[i].state==token_type) counter++;
    }
    return counter;
}

bool tictactoeBrain::three_in_a_row(const cellState& color, const TTT_Board_State& b)
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
    if (three_in_a_row(_robot_color,boardState.get()))      return 1;
    if (three_in_a_row(_opponent_color, boardState.get()))  return 2;
    return 0;
}

void tictactoeBrain::wait_for_opponent_turn(const uint8_t& num_tok_opp)
{
    uint8_t cur_tok_opp = num_tok_opp;

    // We wait until the number of opponent's tokens increases
    while(ros::ok())
    {
        ROS_WARN("Press ENTER when the opponent's turn is done");
        std::cin.get();

        {
            cur_tok_opp = get_num_tokens(_opponent_color);

            if (cur_tok_opp > num_tok_opp) return;
        }

        r.sleep();
    }
}

bool tictactoeBrain::is_board_empty()
{
    TTT_Board_State aux = boardState.get();
    for(int i = 0; i < aux.size(); i++)
    {
        if(aux[i].state==red || aux[i].state==blue) return false;
    }
    return true;
}

bool tictactoeBrain::is_board_full()
{
    TTT_Board_State aux = boardState.get();
    for(int i = 0; i < aux.size(); i++)
    {
        if(aux[i].state==empty || aux[i].state==undefined) return false;
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
        ROS_INFO("[strategy] Randomly place tokens");
    }
    else if (strategy=="smart")
    {
        _choose_next_move=&tictactoeBrain::winning_defensive_random_move;
        ROS_INFO("[strategy] Randomly place tokens but win if possible, or block opponent's victory");
    }
    else if (strategy=="cheating")
    {
        _choose_next_move=&tictactoeBrain::cheating_to_win_random_move;
        ROS_INFO("[strategy] Randomly place tokens but win if possible "
                            "even if cheating is required, or block opponent's victory");
    }
    else
    {
        ROS_ERROR("%s is not an available strategy", strategy.c_str());
    }
}

int tictactoeBrain::play_one_game(bool &cheating)
{
    set_brain_state(TTTBrainState::READY);
    while(ros::ok())
    {
        if (is_board_empty())
        {
            set_brain_state(TTTBrainState::GAME_STARTED);
            break;
        }
    }

    bool robot_turn=true;
    int winner=0; // no winner
    has_cheated=false;

    say_sentence("I start the game.",2);

    ROS_WARN("PRESS ENTER TO START THE GAME");
    std::cin.get();
    uint8_t num_tok_opp=0;
    // uint8_t n_robot_tokens=0;
    while ((winner=get_winner())==0 && !is_board_full() && !ros::isShuttingDown())
    {
        if (robot_turn) // Robot's turn
        {
            // n_robot_tokens=get_num_tokens(_robot_color);
            num_tok_opp=get_num_tokens(_opponent_color);
            say_sentence("It is my turn", 0.3);
            int cell_to_move = get_next_move(cheating);
            ROS_INFO("Moving to cell %i", cell_to_move);

            leftArmCtrl.startAction(ACTION_PICKUP);
            leftArmCtrl.startAction(ACTION_PUTDOWN, cell_to_move);
        }
        else // Participant's turn
        {
            ROS_INFO("Waiting for the participant's move.");
            say_sentence("It is your turn", 0.1);
            wait_for_opponent_turn(num_tok_opp);
        }
        robot_turn=!robot_turn;
    }

    set_brain_state(TTTBrainState::GAME_FINISHED);

    switch(winner)
    {
    case 1:
        ROS_INFO("ROBOT's VICTORY");
        if (has_cheated)
        {
            say_sentence("You humans are so easy to beat!", 5);
        }
        say_sentence("I won", 3);
        break;
    case 2:
        ROS_INFO("OPPONENT's VICTORY");
        say_sentence("You won this time", 4);
        break;
    default:
        ROS_INFO("TIE");
        say_sentence("That's a tie. I will win next time.", 8);
        winner=3;
    }

    return winner;
}

tictactoeBrain::~tictactoeBrain()
{
    pthread_mutex_destroy(&_mutex_brainstate);
    brainstate_timer.stop();
}
