#ifndef __TICTACTOE_UTILS_H__
#define __TICTACTOE_UTILS_H__

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

#include "baxter_tictactoe/MsgBoard.h"

namespace enc = sensor_msgs::image_encodings;

namespace ttt
{

#define ACTION_SCAN         "scan"
#define ACTION_PICKUP       "pick_up"
#define ACTION_PUTDOWN      "put_down"

#undef  ARM_SPEED
#define ARM_SPEED  0.080    // [m/s]

#define NUMBER_OF_CELLS 9

#define NUM_GAMES           3
#define CHEATING_GAME_A     2
#define CHEATING_GAME_B     3

#define COL_RED       "red"
#define COL_BLUE      "blue"
#define COL_EMPTY     "empty"

#define VOICE       "voice_kal_diphone"

typedef std::vector<cv::Point>  Contour;
typedef std::vector<Contour>    Contours;

class Cell
{
private:
    Contour     contour;
    std::string   state;
    int        area_red;
    int       area_blue;

public:
    /* CONSTRUCTORS */
    Cell(            std::string _s = COL_EMPTY, int _ar = 0, int _ab = 0);
    Cell(Contour _c, std::string _s = COL_EMPTY, int _ar = 0, int _ab = 0);
    Cell(const Cell &_c);

    /* DESTRUCTOR */
    ~Cell() {};

    /**
     * Assignment operator
     */
    Cell& operator=(const Cell& _c);

    /**
     * Comparison operator (isEqual). It compares only the state of the cell,
     * which is the only value that matters.
     *
     * @return true/false if equal/different
     */
    bool operator==(const Cell &_c) const;

    /**
     * Comparison operator (isDifferent). It compares only the state of the cell,
     * which is the only value that matters.
     *
     * @return true/false if different/equal
     */
    bool operator!=(const Cell &_c) const;

    /**
     * Resets only the cell state to empty
     */
    bool resetState();

    /**
     * Resets the cell to an empty, pristine state.
     */
    bool resetCell();

    /**
     * Computes and updates its state according to the amount of red and blue area.
     *
     * @return true/false if the cell is colored or still empty.
     */
    bool computeState();

    /**
     * Returns a mask for a cell, i.e. a new image that keeps only the portion
     * delimited by the cell (the rest is set to black)
     *
     * @param      the original image, where the mask will be extracted from.
     * @return     the mask. It has the same size than the original image.
     */
    cv::Mat maskImage(const cv::Mat &);

    /**
     * Prints some useful information from the cell
     *
     * @return the string with the information
     */
    std::string toString();

    /* Self-explaining "getters" */
    std::string getState()    { return state;     };
    Contour     getContour()  { return contour;   };
    cv::Point   getCentroid();
    int         getContourArea();
    int         getRedArea()  { return area_red;  };
    int         getBlueArea() { return area_blue; };

    /* Self-explaining "setters" */
    bool setState(const std::string& _s);
    void setRedArea (size_t _a) { area_red  = _a; };
    void setBlueArea(size_t _a) { area_blue = _a; };
};

class Board
{
private:
    std::vector<Cell> cells;

public:
    /* CONSTRUCTOR */
    Board()  {};

    /* DESTRUCTOR */
    ~Board() {};

    /**
     * Assignment operator. Does not care about boards with different sizes.
     */
    Board& operator=(const Board& _c);

    /**
     * Adds a cell to the board.
     *
     * @param c The cell to be added.
     */
    void addCell(Cell _c) { cells.push_back(_c); };

    /**
     * Masks the board onto an image
     *
     * @param      the original image.
     * @return     the masked image.
     */
    cv::Mat maskImage(const cv::Mat &);

    /**
     * Resets only the cells' states to empty
     */
    bool resetCellStates();

    /**
     * Resets the cells to an empty, pristine state.
     */
    bool resetCells();

    /**
     * Resets the board to an empty, pristine state. It deletes the cells.
     */
    bool resetBoard() { cells.clear(); return true; };

    /**
     * Computes and updates the cells' states according to the amount of red and blue area.
     *
     * @return true/false if there are cells on the board.
     */
    bool computeState();

    /**
     * Indicates if the board is full.
     * @return  true/false if full or not
     **/
    bool isFull();

    /**
     * Indicates if the board is empty.
     * @return  true/false if empty or not
     **/
    bool isEmpty();

    /**
     * Converts a MsgBoard object to the board.
     */
    void fromMsgBoard(const baxter_tictactoe::MsgBoard &msgb);

    /**
     * Converts the board to a MsgBoard (ready to be sent)
     *
     * @return the MsgBoard
     */
    baxter_tictactoe::MsgBoard toMsgBoard();

    /**
     * Print function.
     *
     * @return A text description of the board
     */
    std::string toString();

    /* Self-explaining "getters" */
    Contours    getContours();
    size_t      getNumCells()             { return cells.size();              };

    Cell&       getCell(size_t i)         { return cells[i];                  };
    int         getCellArea(size_t i)     { return cells[i].getContourArea(); };
    int         getCellAreaRed(size_t i)  { return cells[i].getRedArea();     };
    int         getCellAreaBlue(size_t i) { return cells[i].getBlueArea();    };
    std::string getCellState(size_t i)    { return cells[i].getState();       };
    Contour     getCellContour(size_t i)  { return cells[i].getContour();     };
    cv::Point   getCellCentroid(size_t i) { return cells[i].getCentroid();    };

    /* Self-explaining "setters" */
    void setCellState(size_t i, const std::string& s) { cells[i].setState(s); };
};

}

#endif // __TICTACTOE_UTILS_H__
