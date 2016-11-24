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

#undef  PICK_UP_SPEED
#define PICK_UP_SPEED  0.080    // [m/s]

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

struct colorRange
{
    int min;
    int max;

    colorRange(): min(0), max(0) {};
    colorRange(int _min, int _max) : min(_min), max(_max) {};
    colorRange(const colorRange &_cr): min(_cr.min), max(_cr.max) {};

    /**
    * Copy Operator
    **/
    colorRange &operator=(const colorRange &);
};

struct hsvColorRange
{
    colorRange H;
    colorRange S;
    colorRange V;

    hsvColorRange(): H(0,180), S(0,256), V(0,256) {};
    hsvColorRange(const colorRange &_H, const colorRange &_S,
                 const colorRange &_V) : H(_H), S(_S), V(_V) {};
    hsvColorRange(XmlRpc::XmlRpcValue);

    cv::Scalar get_hsv_min() { return cv::Scalar(H.min, S.min, V.min); };
    cv::Scalar get_hsv_max() { return cv::Scalar(H.max, S.max, V.max); };

    /**
     * Print function.
     *
     * @return A text description of the cell
     */
    std::string toString();

    /**
    * Copy Operator
    **/
    hsvColorRange &operator=(const hsvColorRange &);
};

/**
 * Thresholds the HSV image and create a binary image. It can automatically
 * take care of the fact that the red is at the extremes of the HSV spectrum,
 * so two thresholds may be needed in order to robustly detect the red color.
 *
 * @param  _src [description]
 * @param  _hsv [description]
 * @return      [description]
 */
cv::Mat hsvThreshold(const cv::Mat& _src, hsvColorRange _hsv);

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
     * Resets the cell to an empty, pristine state.
     */
    void resetCell();

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
    void setState(std::string s) { state = s;      };
    void setRedArea (int _a)   { area_red  = _a; };
    void setBlueArea(int _a)   { area_blue = _a; };
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
     * Clears the board, by deleting any cell.
     */
    void clear() { cells.clear(); };

    bool resetState();

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
    Contours getContours();
    int      getNumCells()             { return cells.size();              };

    Cell&       getCell(int i)         { return cells[i];                  };
    int         getCellArea(int i)     { return cells[i].getContourArea(); };
    int         getCellAreaRed(int i)  { return cells[i].getRedArea();     };
    int         getCellAreaBlue(int i) { return cells[i].getBlueArea();    };
    std::string getCellState(int i)    { return cells[i].getState();       };
    Contour     getCellContour(int i)  { return cells[i].getContour();     };
    cv::Point   getCellCentroid(int i) { return cells[i].getCentroid();    };

    /* Self-explaining "setters" */
    void setCellState(int i, std::string s) { cells[i].setState(s); };
};

}

#endif // __TICTACTOE_UTILS_H__
