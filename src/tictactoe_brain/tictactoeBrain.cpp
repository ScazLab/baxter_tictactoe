#include "tictactoeBrain.h"

#include <QFile> // this is for qrand (but we should use something else)

using namespace ttt;
using namespace std;
using namespace baxter_tictactoe;

tictactoeBrain::tictactoeBrain(std::string _name, std::string _strategy) :
                               r(100), _nh(name), spinner(4),
                               leftArmCtrl("ttt_controller", "left"),
                               rightArmCtrl("ttt_controller", "right")
{
    pthread_mutexattr_t _mutex_attr;
    pthread_mutexattr_init(&_mutex_attr);
    pthread_mutexattr_settype(&_mutex_attr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(&_mutex_brainstate, &_mutex_attr);
    pthread_mutex_init(&_mutex_board, &_mutex_attr);

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

    // string robot_col;
    // _nh.param<string>("ttt_brain/robot_col",  robot_col, "blue");
    // ROS_ASSERT_MSG(robot_col!="blue", "robot_col should be set to blue in the parameter server. "
    //                            "If you want to use red, be willing to spend some time in coding it!");
    _robot_col=COL_BLUE;
    _opponent_col=_robot_col==COL_BLUE?COL_RED:COL_BLUE;

    set_brain_state(TTTBrainState::CALIB);
    leftArmCtrl.startAction(ACTION_SCAN);
    while(ros::ok() && leftArmCtrl.getState() != DONE)
    {
        r.sleep();
    }
    set_brain_state(TTTBrainState::READY);

    qsrand(ros::Time::now().nsec);
    set_strategy(_strategy);

    has_cheated=false;
}

ttt::Board tictactoeBrain::getBoard()
{
    ttt::Board res;

    pthread_mutex_lock(&_mutex_board);
    res = board;
    pthread_mutex_unlock(&_mutex_board);

    return res;
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

void tictactoeBrain::boardStateCb(const baxter_tictactoe::MsgBoard &msg)
{
    ROS_DEBUG("New TTT board state received");
    pthread_mutex_lock(&_mutex_board);
    board.fromMsgBoard(msg);
    pthread_mutex_unlock(&_mutex_board);
}

int tictactoeBrain::randomMove(bool& cheating)
{
    cheating=false;
    int r;
    ttt::Board aux = getBoard();
    do {
        r = qrand() % NUMBER_OF_CELLS + 1; //random number between 1 and NUMBER_OF_CELLS
        ROS_DEBUG("Cell %d is in state %s ==? %s", r, aux.getCellState(r-1).c_str(),
                  MsgCell::EMPTY.c_str());
    }
    while(aux.getCellState(r-1)!=COL_EMPTY);

    ROS_INFO("Random move to cell # %i", r);
    return r;
}

int tictactoeBrain::cheating_to_win_randomMove(bool& cheating)
{
    int next_cell_id=-1;
    if ((next_cell_id = victoryMove()) != -1) return next_cell_id;
    if ((next_cell_id = cheatingMove()) != -1)
    {
        cheating=true;
        return next_cell_id;
    }
    if ((next_cell_id = defensiveMove()) != -1) return next_cell_id;
    return randomMove(cheating);
}

int tictactoeBrain::winning_defensive_randomMove(bool& cheating)
{
    cheating=false;
    int next_cell_id=-1;
    if ((next_cell_id = victoryMove()) != -1) return next_cell_id;
    if ((next_cell_id = defensiveMove()) != -1) return next_cell_id;
    return randomMove(cheating);
}

int tictactoeBrain::cheatingMove()
{
    ttt::Board aux = getBoard();
    string cell_state = COL_EMPTY;
    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i)
    {
        if (aux.getCellState(i)==_opponent_col)
        {
            cell_state=aux.getCellState(i);
            aux.getCellState(i)=_robot_col;
            if (three_in_a_row(_robot_col, aux))
            {
                ROS_INFO("Cheating move to cell # %lu", i+1);
                has_cheated=true;
                return i+1;
            }
            aux.getCellState(i)=cell_state;
        }
    }
    ROS_INFO("No chance to win in this turn");
    return -1;
}

int tictactoeBrain::defensiveMove()
{
    ttt::Board aux = getBoard();
    string cell_state = COL_EMPTY;
    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i)
    {
        if (aux.getCellState(i)==COL_EMPTY)
        {
            cell_state = aux.getCellState(i);
            aux.getCellState(i) = _opponent_col;
            if (three_in_a_row(_opponent_col, aux))
            {
                ROS_INFO("Defensive move to cell # %lu", i+1);
                return i+1;
            }
            aux.getCellState(i)=cell_state;
        }
    }
    // ROS_INFO("No chance to lose in this turn");
    return -1;
}

int tictactoeBrain::victoryMove()
{
    ttt::Board aux = getBoard();
    string cell_state = COL_EMPTY;

    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i)
    {
        if (aux.getCellState(i)==COL_EMPTY)
        {
            cell_state = aux.getCellState(i);
            aux.getCellState(i) = _robot_col;
            if (three_in_a_row(_robot_col, aux))
            {
                ROS_INFO("Victory move to cell # %lu", i+1);
                return i+1;
            }
            aux.getCellState(i) = cell_state;
        }
    }

    // ROS_INFO("No chance to win in this turn");
    return -1;
}

int tictactoeBrain::get_nextMove(bool& cheating)
{
    return (this->*_choose_nextMove)(cheating);
}

unsigned short int tictactoeBrain::get_num_tokens()
{
    unsigned short int counter=0;
    ttt::Board aux = getBoard();
    for(int i = 0; i < aux.getNumCells(); i++)
    {
        if(aux.getCellState(i)!=COL_EMPTY) counter++;
    }
    return counter;
}

unsigned short int tictactoeBrain::get_num_tokens(std::string token_type)
{
    unsigned short int counter=0;
    ttt::Board aux = getBoard();
    for(int i = 0; i < aux.getNumCells(); i++)
    {
       if(aux.getCellState(i)==token_type) counter++;
    }
    return counter;
}

bool tictactoeBrain::three_in_a_row(const std::string& color, ttt::Board& b)
{
    if(color!=COL_BLUE && color!=COL_RED) return false; // There are only red and blue tokens

    if(b.getCellState(0)==color && b.getCellState(1)==color && b.getCellState(2)==color) return true; // first row
    if(b.getCellState(3)==color && b.getCellState(4)==color && b.getCellState(5)==color) return true; // second row
    if(b.getCellState(6)==color && b.getCellState(7)==color && b.getCellState(8)==color) return true; // third row
    if(b.getCellState(0)==color && b.getCellState(3)==color && b.getCellState(6)==color) return true; // first column
    if(b.getCellState(1)==color && b.getCellState(4)==color && b.getCellState(7)==color) return true; // second column
    if(b.getCellState(2)==color && b.getCellState(5)==color && b.getCellState(8)==color) return true; // third column
    if(b.getCellState(0)==color && b.getCellState(4)==color && b.getCellState(8)==color) return true; // first diagonal
    if(b.getCellState(2)==color && b.getCellState(4)==color && b.getCellState(6)==color) return true; // second diagonal

    return false;
}

unsigned short int tictactoeBrain::get_winner()
{
    ttt::Board aux = getBoard();
    if (three_in_a_row(_robot_col, aux))      return 1;
    if (three_in_a_row(_opponent_col, aux))   return 2;
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
            cur_tok_opp = get_num_tokens(_opponent_col);

            if (cur_tok_opp > num_tok_opp) return;
        }

        r.sleep();
    }
}

bool tictactoeBrain::is_board_empty()
{
    ttt::Board aux = getBoard();
    for(int i = 0; i < aux.getNumCells(); i++)
    {
        if(aux.getCellState(i)==COL_RED || aux.getCellState(i)==COL_BLUE) return false;
    }
    return true;
}

bool tictactoeBrain::is_board_full()
{
    ttt::Board aux = getBoard();
    for(int i = 0; i < aux.getNumCells(); i++)
    {
        if(aux.getCellState(i)==COL_EMPTY) return false;
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
        _choose_nextMove=&tictactoeBrain::randomMove;
        ROS_INFO("[strategy] Randomly place tokens");
    }
    else if (strategy=="smart")
    {
        _choose_nextMove=&tictactoeBrain::winning_defensive_randomMove;
        ROS_INFO("[strategy] Randomly place tokens but win if possible, or block opponent's victory");
    }
    else if (strategy=="cheating")
    {
        _choose_nextMove=&tictactoeBrain::cheating_to_win_randomMove;
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
            // n_robot_tokens=get_num_tokens(_robot_col);
            num_tok_opp=get_num_tokens(_opponent_col);
            say_sentence("It is my turn", 0.3);
            int cell_toMove = get_nextMove(cheating);
            ROS_INFO("Moving to cell %i", cell_toMove);

            leftArmCtrl.startAction(ACTION_PICKUP);
            leftArmCtrl.startAction(ACTION_PUTDOWN, cell_toMove);
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
    pthread_mutex_destroy(&_mutex_board);
    brainstate_timer.stop();
}
