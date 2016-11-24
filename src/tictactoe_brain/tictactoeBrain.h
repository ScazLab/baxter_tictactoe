#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <baxter_tictactoe/MsgBoard.h>
#include <baxter_tictactoe/TTTBrainState.h>

#include "baxter_tictactoe/tictactoe_utils.h"
#include "baxter_tictactoe/ttt_controller.h"

#include <pthread.h>

namespace ttt
{

class tictactoeBrain
{
private:
    std::string name;

    ros::NodeHandle _nh;        // ROS node handle
    ros::AsyncSpinner spinner;  // AsyncSpinner to handle callbacks

    /* STATE OF THE BOARD */
    ttt::Board board;

    ros::Subscriber boardState_sub; // subscriber to receive the state of the board

    pthread_mutex_t _mutex_board;

    /* STATE OF THE TTT DEMO */
    baxter_tictactoe::TTTBrainState    s; // state of the system
    ros::Timer          brainstate_timer; // timer to publish the state of the system at a specific rate

    ros::Publisher  tttBrain_pub;   // publisher to publish state of the system

    pthread_mutex_t _mutex_brainstate;

    void publishTTTBrainStateCb(const ros::TimerEvent&);  // callback to publish the state of the demo

    /* MISC */
    std::string    _robot_col;  // Color of the tokens the robot    is playing with.
    std::string _opponent_col;  // Color of the tokens the opponent is playing with.

    sound_play::SoundClient _voice_synthesizer;
    std::string                    _voice_type; // Type of voice.

    bool cheating;         // It determines if the robot can cheat or not.

    // Pointer to the function that chooses the next move
    int (tictactoeBrain::*_choose_next_move)(bool& cheating);

    TTTController  leftArmCtrl;
    TTTController rightArmCtrl;

    ros::Rate r;

    bool has_cheated;

    /**
     * It handles the message published when the state of a cell has changed. The new TTT board state
     * is stored in the thread-safe private attribute called boardState.
     * \param msg the message with the new TTT state, i.e. the states of each of the cells
     **/
    void boardStateCb(const baxter_tictactoe::MsgBoard &msg);

    /**
     * It determines randomly the next empty cell to place a token.
     * \param cheating It indicates if cheating has happened.
     * @return an integer representing the cell where to place the next token
     **/
    int randomMove(bool& cheating);

    /**
     * It determines the next cell to place a token. It will try to win in this turn, even if it has to cheat
     * placing a token on top of an opponent's token. If the robot can win without breaking the rules, it will
     * do it. Otherwise, if it can win cheating, it will do it. It will block opponent's victory. If it cannot
     * win anyway, it will randomly choose a cell.
     * \param cheating It indicates if cheating has happened.
     * @return an integer representing the cell where to place the next token
     **/
    int cheatingToWinMove(bool& cheating);

    /**
     * It determines the next cell to place a token. It tries to win in its turn but it does no cheat.
     * If the robot can win, it will do it. Otherwise, if it cannot win in this turn anyway, it will
     * try to block the opponent's victory. If this is not needed, it will randomly choose a cell.
     * \param cheating It indicates if cheating has happened.
     * @return an integer representing the cell where to place the next token
     **/
    int winningDefensiveMove(bool& cheating);

    /*
     * It determines the next cell to place a token. It always will try to win in this turn, even if
     * there is an opponent's token already in that cell. If the robot can win without breaking the
     * rules, it will do it. Otherwise, if it can win cheating, it will do it. If it cannot win in
     * this turn anyway, it will try to block the opponent's victory. If this is not needed it will
     * randomly choose a cell.
     * @param cheating It indicates if cheating has happened.
     * @return an integer representing the cell where to place the next token
     **/
    int smartCheatingMove(bool& cheating);

    /*
     * It determines if the robot can win in this turn cheating, i.e. placing a token in a cell
     * occupied with an opponent's token.
     * @return -1 if the robot cannot win in the next move,
     * or an integer corresponding to the cell where to place the next token to win, even if there
     * is an opponent's token in that cell. The cell ids are between 1 (first row, first column)
     * and NUMBER_OF_CELLS (last row, last column).
     */
    int cheatingMove();

    /**
     * It determines if the opponent can win in the next move.
     * @return -1 if the opponent cannot win in the next move, or an integer corresponding
     * to the first found cell where an opponent's token can be placed to win the game.
     * The cell ids are between 1 (first row, first column) and NUMBER_OF_CELLS (last row, last column).
     **/
    int defensiveMove();

    /**
     * It determines if the robot can win in the next move.
     * @return -1 if the robot cannot win in the next move, or an integer corresponding
     * to the first found cell where a robot's token can be placed to win the game. The
     * cell ids are between 1 (first row, first column) and NUMBER_OF_CELLS (last row, last column).
     **/
    int victoryMove();

public:

    tictactoeBrain(std::string _name="ttt_brain", std::string _strategy="random");

    ~tictactoeBrain();

    /**
     * Returns the cell where the next token is gonna be placed.
     * @param cheating It indicates if cheating happens
     * @return The return value is between 1 (first row, first column)
     * and NUMBER_OF_CELLS (last row, last column).
     **/
    int getNextMove(bool& cheating);

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
    bool threeInARow(const std::string& color, ttt::Board &b);

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
     * @param  cheating if to cheat or not.
     * @return          The game results (i.e. the winner, if any)
     */
    int playOneGame(bool &cheating);

    /* GETTERS */
    ttt::Board  getBoard();
    std::string getRobotColor()        { return    _robot_col; };
    std::string getOpponentColor()     { return _opponent_col; };

    bool getCheating() { return cheating; };

    /* SETTERS */
    void setCheating(bool _c) { cheating=_c; };
    void setStrategy(std::string strategy);
    void setBrainState(int state);
};

}
