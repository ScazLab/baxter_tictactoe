#include "tictactoeBrain.h"

#include <stdlib.h> // srand, rand

using namespace std;
using namespace baxter_tictactoe;

tictactoeBrain::tictactoeBrain(std::string _name, std::string _strategy, bool _legacy_code) : brain_thread_close_flag(false),
                               nh(_name), spinner(4), r(100), legacy_code(_legacy_code), num_games(NUM_GAMES), curr_game(0),
                               wins(3,0), curr_board(9), internal_board(9), is_board_detected(false),
                               left_ttt_ctrl(_name, "left", _legacy_code), right_ttt_ctrl(_name, "right", _legacy_code),
                               n_robot_tokens(0), n_human_tokens(0)
{

    printf("\n");
    ROS_INFO("Legacy code %s enabled.", _legacy_code?"is":"is not");
    setBrainState(TTTBrainState::INIT);

    srand(ros::Time::now().nsec);
    setStrategy(_strategy);

    boardState_sub = nh.subscribe("/baxter_tictactoe/board_state", SUBSCRIBER_BUFFER,
                                    &tictactoeBrain::boardStateCb, this);
    tttBrain_pub   = nh.advertise<TTTBrainState>("/baxter_tictactoe/ttt_brain_state", 1);

    brainstate_timer = nh.createTimer(ros::Duration(0.1), &tictactoeBrain::publishTTTBrainState, this, false);

    nh.param<string>("voice", voice_type, VOICE);
    ROS_INFO("Using voice %s", voice_type.c_str());

    nh.param<int>("num_games", num_games, NUM_GAMES);

    if (nh.hasParam("cheating_games"))
    {
        nh.getParam("cheating_games", cheating_games);
    }
    else
    {
        cheating_games.push_back(CHEATING_GAME_A);
        cheating_games.push_back(CHEATING_GAME_B);
    }

    ROS_INFO("Number of games: %i; Cheating games: %s",
              num_games, toString(cheating_games).c_str());

    nh.param<string>("robot_color", robot_color, "blue");
    robot_color    = robot_color==COL_BLUE?COL_BLUE:COL_RED;
    opponent_color = robot_color==COL_BLUE?COL_RED:COL_BLUE;

    ROS_INFO("Robot plays with %s tokens and the opponent with %s tokens.",
              getRobotColor().c_str(), getOpponentColor().c_str());

    startInternalThread();
}

bool tictactoeBrain::startInternalThread()
{
    brain_thread = std::thread(&tictactoeBrain::InternalThreadEntry, this);
    return brain_thread.joinable();
}

void tictactoeBrain::InternalThreadEntry()
{
    bool wipe_out_board_message = true;
    while (ros::ok() && not getBrainThreadCloseFlag())
    {
        if      (getBrainState() == TTTBrainState::INIT)
        {
            left_ttt_ctrl.startAction(ACTION_SCAN);
            setBrainState(TTTBrainState::CALIB);
        }
        else if (getBrainState() == TTTBrainState::CALIB)
        {
            if (left_ttt_ctrl.getState() == DONE) { setBrainState(TTTBrainState::READY); }
        }
        else if (getBrainState() == TTTBrainState::READY)
        {
            if (is_board_detected)        { setBrainState(TTTBrainState::MATCH_STARTED); }
        }
        else if (getBrainState() == TTTBrainState::MATCH_STARTED)
        {
            saySentence("Welcome! Let's play Tic Tac Toe.", 3);
            saySentence("Do not grasp your token before I say that it is your turn", 4);
            curr_game = 1;
            setBrainState(TTTBrainState::GAME_STARTED);
        }
        else if (getBrainState() == TTTBrainState::GAME_STARTED)
        {
            if (getCurrBoard().isEmpty())
            {
                setBrainState(TTTBrainState::GAME_RUNNING);
                wipe_out_board_message = true;
            }
            else if (wipe_out_board_message == true)
            {
                wipe_out_board_message = false;
                saySentence("Please clean the board so that we can start a new game.", 6);
            }
        }
        else if (getBrainState() == TTTBrainState::GAME_RUNNING)
        {
            ROS_WARN("GAME #%i", curr_game);

            playOneGame();

            if (curr_game > num_games) { setBrainState(baxter_tictactoe::TTTBrainState::MATCH_FINISHED); }
            else                       { setBrainState(baxter_tictactoe::TTTBrainState::GAME_STARTED);   }
        }
        else if (getBrainState() == TTTBrainState::MATCH_FINISHED)
        {
            saySentence("Game over. It was my pleasure to win over you. Thanks for being so human.", 10);
            ROS_INFO("Baxter Wins: %i\tHuman Wins: %i\tTies: %i", wins[0], wins[1], wins[2]);
            break;
        }

        r.sleep();
    }
}

bool tictactoeBrain::getBrainThreadCloseFlag()
{
    std::lock_guard<std::mutex> lck(mutex_brain_thread_close_flag);
    return brain_thread_close_flag;
}

void tictactoeBrain::setBrainThreadCloseFlag(bool arg)
{
    std::lock_guard<std::mutex> lck(mutex_brain_thread_close_flag);
    brain_thread_close_flag = arg;
}

void tictactoeBrain::playOneGame()
{
    bool robot_turn = true;
    int winner  = WIN_NONE;

    bool has_to_cheat=false;

    for (size_t j = 0; j < cheating_games.size(); ++j)
    {
        if (cheating_games[j] == curr_game)
        {
            has_to_cheat=true;
        }
    }

    if (has_to_cheat)   { setStrategy("cheating"); }
    else                { setStrategy(   "smart"); }

    saySentence("I start the game.",2);

    n_robot_tokens=0;
    n_human_tokens=0;

    while (winner == WIN_NONE && not internal_board.isFull() && not ros::isShuttingDown())
    {
        if (robot_turn) // Robot's turn
        {
            if (n_robot_tokens != 0) { saySentence("It is my turn", 0.3); }

            int cell_toMove = getNextMove();    // This should be from 1 to 9
            ROS_INFO("Moving to cell %i", cell_toMove);

            left_ttt_ctrl.startAction(ACTION_PICKUP);
            left_ttt_ctrl.startAction(ACTION_PUTDOWN, cell_toMove);
            internal_board.setCellState(cell_toMove-1, getRobotColor());
            n_robot_tokens = internal_board.getNumTokens(getRobotColor());
        }
        else // Participant's turn
        {
            waitForOpponentTurn();
        }

        robot_turn = not robot_turn;
        winner = getWinner();
    }

    setBrainState(TTTBrainState::GAME_FINISHED);

    switch(winner)
    {
        case WIN_ROBOT:
            ROS_INFO("ROBOT's VICTORY");
            if (has_cheated)
            {
                saySentence("You humans are so easy to beat!", 3);
            }
            saySentence("I won", 1);
            break;
        case WIN_OPP:
            ROS_INFO("OPPONENT's VICTORY");
            saySentence("You won this time", 2);
            break;
        default:
            ROS_INFO("TIE");
            saySentence("That's a tie. I will win next time.", 3);
            winner = WIN_TIE;
    }

    // Let's increment the winners' count
    wins[winner-1] = wins[winner-1] + 1;

    if (has_to_cheat && not has_cheated)
    {
        ROS_WARN("Cheating game ended without cheating. Game counter does not increase.");
        --curr_game;
    }

    ++curr_game;
    internal_board.resetCellStates();

    return;
}

Board tictactoeBrain::getCurrBoard()
{
    Board res;

    std::lock_guard<std::mutex> lck(mutex_curr_board);
    res = curr_board;

    return res;
}

void tictactoeBrain::publishTTTBrainState(const ros::TimerEvent&)
{
    std::lock_guard<std::mutex> lck(mutex_brain);
    tttBrain_pub.publish(s);
}

int tictactoeBrain::getBrainState()
{
    int state;
    std::lock_guard<std::mutex> lck(mutex_brain);
    state = s.state;

    return state;
}

void tictactoeBrain::setBrainState(int _state)
{
    if (_state != getBrainState())
    {
        std::lock_guard<std::mutex> lck(mutex_brain);
        s.state = _state;
        ROS_WARN("New state [%i]", _state);
    }

    if (_state == TTTBrainState::GAME_FINISHED)
    {
        is_board_detected = false;
    }
}

void tictactoeBrain::boardStateCb(const baxter_tictactoe::MsgBoard &_msg)
{
    ROS_DEBUG("New TTT board state received");
    std::lock_guard<std::mutex> lck(mutex_curr_board);
    curr_board.fromMsgBoard(_msg);
    is_board_detected = true;
}

int tictactoeBrain::randomStrategyMove()
{
    int rnd;
    Board aux = internal_board;
    do {
        rnd = rand() % NUMBER_OF_CELLS + 1; //random number between 1 and NUMBER_OF_CELLS
        ROS_DEBUG("Cell %d is in state %s ==? %s", rnd, aux.getCellState(rnd-1).c_str(),
                  MsgCell::EMPTY.c_str());
    }
    while(aux.getCellState(rnd-1)!=COL_EMPTY);

    ROS_WARN("Random move to cell # %i", rnd);
    return rnd;
}

int tictactoeBrain::cheatingStrategyMove()
{
    int next_cell_id=-1;
    if (  victoryMove(next_cell_id))    { return next_cell_id; }
    if ( cheatingMove(next_cell_id))    { return next_cell_id; }
    if (defensiveMove(next_cell_id))    { return next_cell_id; }
    return randomStrategyMove();
}

int tictactoeBrain::smartStrategyMove()
{
    int next_cell_id=-1;
    if (  victoryMove(next_cell_id))    { return next_cell_id; }
    if (defensiveMove(next_cell_id))    { return next_cell_id; }
    return randomStrategyMove();
}

bool tictactoeBrain::cheatingMove(int &_id)
{
    _id = -1;
    Board aux = internal_board;
    string cell_state = COL_EMPTY;

    for (size_t i = 0; i < aux.getNumCells(); ++i)
    {
        if (aux.getCellState(i)==getOpponentColor())
        {
            cell_state=aux.getCellState(i);
            aux.setCellState(i, getRobotColor());
            if (aux.threeInARow(getRobotColor()))
            {
                ROS_WARN("Cheating move to cell # %lu", i+1);
                has_cheated=true;
                _id = i+1;
                // If the robot cheats, the number of tokens expected from the humans decreases.
                --n_human_tokens;
                return true;
            }
            aux.setCellState(i, cell_state);
        }
    }

    // ROS_WARN("Cheating move not successful!");
    return false;
}

bool tictactoeBrain::defensiveMove(int &_id)
{
    _id = -1;
    Board aux = internal_board;
    string cell_state = COL_EMPTY;

    for (size_t i = 0; i < aux.getNumCells(); ++i)
    {
        if (aux.getCellState(i)==COL_EMPTY)
        {
            cell_state = aux.getCellState(i);
            aux.setCellState(i, getOpponentColor());
            if (aux.threeInARow(getOpponentColor()))
            {
                ROS_WARN("Defensive move to cell # %lu", i+1);
                _id = i+1;
                return true;
            }
            aux.setCellState(i, cell_state);
        }
    }

    // ROS_INFO("DefensiveMove not successful!");
    return false;
}

bool tictactoeBrain::victoryMove(int &_id)
{
    _id = -1;
    Board aux = internal_board;
    string cell_state = COL_EMPTY;

    for (size_t i = 0; i < aux.getNumCells(); ++i)
    {
        if (aux.getCellState(i)==COL_EMPTY)
        {
            cell_state = aux.getCellState(i);
            aux.setCellState(i, getRobotColor());
            if (aux.threeInARow(getRobotColor()))
            {
                ROS_WARN("Victory move to cell # %lu", i+1);
                _id = i+1;
                return true;
            }
            aux.setCellState(i, cell_state);
        }
    }

    // ROS_INFO("VictoryMove not successful!");
    return false;
}

int tictactoeBrain::getNextMove()
{
    return (this->*choose_next_move)();
}

unsigned short int tictactoeBrain::getWinner()
{
    if (internal_board.threeInARow(getRobotColor()))      { return WIN_ROBOT; }
    if (internal_board.threeInARow(getOpponentColor()))   { return   WIN_OPP; }

    return WIN_NONE;
}

void tictactoeBrain::waitForOpponentTurn()
{
    ROS_INFO("Waiting for the participant's move. "
             "I am expecting %lu token%s from myself and %lu token%s from my opponent",
             n_robot_tokens, n_robot_tokens==1?"":"s",
             n_human_tokens+1, n_human_tokens+1==1?"":"s");

    int cnt = 0;
    bool say_it_is_your_turn = true;

    // We wait until the number of opponent's tokens equals the robots'
    while(ros::ok())
    {
        Board new_board = getCurrBoard();

        if (internal_board.isOneTokenAdded(new_board, getOpponentColor()))
        {
            ++cnt;
        }
        else
        {
            if (say_it_is_your_turn == true)
            {
                saySentence("It is your turn", 0.1);
                say_it_is_your_turn = false;
            }

            cnt = 0;
        }

        if (cnt == 100)
        {
            internal_board = new_board;
            n_human_tokens = internal_board.getNumTokens(getOpponentColor());
            return;
        } // 100 means 1 second

        r.sleep();
    }
}

void tictactoeBrain::saySentence(std::string _sentence, double _t)
{
    ROS_INFO("saySentence: %s", _sentence.c_str());
    voice_synthesizer.say(_sentence, voice_type);
    ros::Duration(_t).sleep();
}

void tictactoeBrain::setStrategy(std::string _strategy)
{
    if (_strategy=="random")
    {
        choose_next_move=&tictactoeBrain::randomStrategyMove;
        ROS_INFO("[strategy] Randomly place tokens");
    }
    else if (_strategy=="smart")
    {
        choose_next_move=&tictactoeBrain::smartStrategyMove;
        ROS_INFO("[strategy] Try to win without cheating");
    }
    else if (_strategy=="cheating")
    {
        choose_next_move=&tictactoeBrain::cheatingStrategyMove;
        ROS_INFO("[strategy] Try to win by cheating");
    }
    else
    {
        ROS_ERROR("%s is not an available strategy.", _strategy.c_str());
    }
}

tictactoeBrain::~tictactoeBrain()
{
    setBrainThreadCloseFlag(true);
    if (brain_thread.joinable())
    {
        brain_thread.join();
    }
    brainstate_timer.stop();
}
