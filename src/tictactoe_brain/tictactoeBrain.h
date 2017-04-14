#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <robot_utils/ros_thread.h>

#include <baxter_tictactoe/MsgBoard.h>
#include <baxter_tictactoe/TTTBrainState.h>

#include "baxter_tictactoe/ttt_controller.h"
#include "baxter_tictactoe/tictactoe_utils.h"

#include <pthread.h>

#define WIN_NONE    0
#define WIN_ROBOT   1
#define WIN_OPP     2
#define WIN_TIE     3

namespace ttt
{

class tictactoeBrain : public ROSThread
{
private:
    std::string name;

    int    num_games;
    int    curr_game;

    std::vector<int> cheating_games; // vector that stores which of the games will be a cheating one.
    std::vector<int>           wins; // vector of three elements to count the wins (wins[0]->robot, wins[1]->opponent, wins[2]->ties)

    ros::Rate r;

    ros::NodeHandle _nh;        // ROS node handle
    ros::AsyncSpinner spinner;  // AsyncSpinner to handle callbacks

    /* STATE OF THE BOARD */
    ttt::Board               board;
    ros::Subscriber boardState_sub; // subscriber to receive the state of the board
    pthread_mutex_t   _mutex_board;
    bool        _is_board_detected;

    /* STATE OF THE TTT DEMO */
    baxter_tictactoe::TTTBrainState    s; // state of the system
    ros::Timer          brainstate_timer; // timer to publish the state of the system at a specific rate

    ros::Publisher  tttBrain_pub;   // publisher to publish state of the system
    pthread_mutex_t _mutex_brain;   // mutex to protect the state of the system

    /* MISC */
    std::string    _robot_col;  // Color of the tokens the robot    is playing with.
    std::string _opponent_col;  // Color of the tokens the opponent is playing with.

    sound_play::SoundClient _voice_synthesizer;
    std::string                    _voice_type; // Type of voice.

    // Pointer to the function that chooses the next move
    int (tictactoeBrain::*_choose_next_move)();

    TTTController  leftArmCtrl;
    TTTController rightArmCtrl;

    bool has_cheated;

    /**
     * Timer callback to publish the state of the demo.
     */
    void publishTTTBrainState(const ros::TimerEvent&);

    /**
     * ROS callback to handle the message published when the state of a cell has changed.
     *
     * \param msg the message with the new the state of each of the cells
     **/
    void boardStateCb(const baxter_tictactoe::MsgBoard &msg);

    /**
     * It determines randomly the next empty cell to place a token.
     *
     * @return                the cell where to place the next token
     **/
    int randomStrategyMove();

    /**
     * It determines the next cell to win by cheating (by placing a token on top of an opponent's token).
     * If the robot cannot win by explicitly cheating, it will try to win without breaking the rules.
     * Otherwise, it will try block opponent's victory. If this is not needed, it will randomly choose a cell.
     *
     * @return                the cell where to place the next token
     **/
    int cheatingStrategyMove();

    /**
     * It determines the next cell to win without cheating. If the robot cannot win, it will
     * try to block the opponent's victory. If this is not needed, it will randomly choose a cell.
     *
     * @return                the cell where to place the next token
     **/
    int smartStrategyMove();

    /*
     * It determines if the robot can win in this turn cheating, i.e. placing a token in a cell
     * occupied with an opponent's token.
     *
     * @param       id of the cell to move to if the action was successful (-1 if not)
     * @return      true/false if success/failure
     */
    bool cheatingMove(int &cell_id);

    /**
     * It determines if the opponent can win in the next move.
     *
     * @param       id of the cell to move to if the action was successful (-1 if not)
     * @return      true/false if success/failure
     **/
    bool defensiveMove(int &cell_id);

    /**
     * It determines if the robot can win in the next move.
     *
     * @param       id of the cell to move to if the action was successful (-1 if not)
     * @return      true/false if success/failure
     **/
    bool victoryMove(int &cell_id);

protected:

    void InternalThreadEntry();

public:

    tictactoeBrain(std::string _name="ttt_brain", std::string _strategy="random");

    ~tictactoeBrain();

    /**
     * Returns the cell where the next token is gonna be placed.
     * @param cheating It indicates if cheating happens
     * @return The return value is between 1 (first row, first column)
     * and NUMBER_OF_CELLS (last row, last column).
     **/
    int getNextMove();

    /**
     * This function counts the total number of tokens on the board.
     * That is, the number of cells that are not empty or undefined.
     * @return The number of cells where there is a red or blue token.
     **/
    unsigned short int getNumTokens();

    /**
     * This function counts the number of a particular type of tokens
     * on the board. That is, the number of cells that occupied by these kind of tokens.
     * @param token_type The kind of tokens we are counting
     * @return The number of cells where there is a token_type token.
     **/
    unsigned short int getNumTokens(std::string token_type);

    /**
     * This function checks if there are 3 cell_color tokens in a row, which means that the game is over.
     * In a 3x3 board there are 8 possible combinations to get 3 tokens in a row. We explore all of them.
     *
     * @param color It represents the color of the tokens in the row we are searching for.
     * @param b   TTT board where searching for three tokens of the same color in a row.
     *
     * @return True in case of a 3 token row is found, false otherwise.
     **/
    bool threeInARow(const std::string& col, ttt::Board &b);

    /**
     * This function returns the winner of the game.
     * @return 0 if there is not winner, 1 if the winner is the robot, or 2 if the winner is the opponent.
     **/
    unsigned short int getWinner();

    /**
     * This function blocks until the opponent has done his move.
     * This is detected considering the number of the opponent's
     * tokens on the board. The function waits until the number
     * of opponent's tokens in the board increases.
     * @param number of opponent's token at the beginning
     **/
    void waitForOpponentTurn(const uint8_t &num_tok_opp);

    /**
     * Indicates if the board is full.
     * @return  true/false if full or not
     **/
    bool isBoardFull();

    /**
     * Indicates if the board is empty.
     * @return  true/false if empty or not
     **/
    bool isBoardEmpty();

    /**
     * This function synthesizes sentence and waits t seconds.
     * @param sentence string corresponding with the sentence to synthesize.
     * @param t number of seconds to block.
     **/
    void saySentence(std::string sentence, double t);

    /**
     * Plays one game
     */
    void playOneGame();

    /* GETTERS */
    ttt::Board  getBoard();
    std::string getRobotColor()        { return    _robot_col; };
    std::string getOpponentColor()     { return _opponent_col; };
    int         getBrainState();

    /* SETTERS */
    void setStrategy(std::string strategy);
    void setBrainState(int state);
};

}
