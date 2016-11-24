#ifndef __TICTACTOE_UTILS_H__
#define __TICTACTOE_UTILS_H__

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

#include "baxter_tictactoe/MsgCell.h"

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

#define COLOR_RED       "red"
#define COLOR_BLUE      "blue"
#define COLOR_EMPTY     "empty"
#define COLOR_UNDEFINED "undef"

#define VOICE       "voice_kal_diphone"

// Used to determine the three possible states of a cell.
// Undefined is just used when it is created and non state has been assigned.
typedef enum {empty=baxter_tictactoe::MsgCell::EMPTY,
              blue=baxter_tictactoe::MsgCell::BLUE,
              red=baxter_tictactoe::MsgCell::RED,
              undefined=baxter_tictactoe::MsgCell::UNDEFINED} cellState;

typedef std::vector<cv::Point>  Contour;
typedef std::vector<Contour>    Contours;

std::string cell_state_to_str(cellState c_s);

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
 * Thresholds the HSV image and create a binary image
 * @param  _src [description]
 * @param  _hsv [description]
 * @return      [description]
 */
cv::Mat hsv_threshold(const cv::Mat& _src, hsvColorRange _hsv);

struct Cell
{
private:
    Contour  contour;
    cellState  state;
    int     area_red;
    int    area_blue;

public:
    Cell(cellState _s = empty, int _ar = 0, int _ab = 0);
    Cell(Contour _c, cellState _s = empty, int _ar = 0, int _ab = 0);

    ~Cell() {};

    void reset();

    /**
     * Returns a mask for a cell, i.e. a new image that keeps only the portion
     * delimited by the cell (the rest is set to black)
     * @param      the original image, where the mask will be extracted from.
     * @return     the mask. It has the same size than the original image.
     */
    cv::Mat maskImage(const cv::Mat &);

    /**
     * Computes the centroid of a cell.
     * @return  the centroid as a cv::Point (defaults to [0,0] if ther is no contour)
     */
    cv::Point getCentroid();

    /**
     * Computes the area of the contour
     * @return the area of the contour (defaults to 0 if there is no contour)
     */
    int getContourArea();

    /**
     * Prints some useful information from the cell
     * @return the string with the information
     */
    std::string toString();

    /* Self-explaining "getters" */
    std::string getStateStr();
    int         getRedArea()  { return area_red;  };
    int         getBlueArea() { return area_blue; };
    cellState   getState()    { return state;     };
    Contour     getContour()  { return contour;   };

    /* Self-explaining "setters" */
    void setState(cellState s) { state = s;      };
    void setRedArea (int _a)   { area_red  = _a; };
    void setBlueArea(int _a)   { area_blue = _a; };
};

struct Board
{
private:
    std::vector<Cell> cells;

public:
    Board()  {};
    ~Board() {};

    /**
     * Adds a cell to the board.
     *
     * @param c The cell to be added.
     */
    void addCell(Cell _c) { cells.push_back(_c); };

    /**
     * Masks the board onto an image
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
    cellState   getCellState(int i)    { return cells[i].getState();       };
    Contour     getCellContour(int i)  { return cells[i].getContour();     };
    cv::Point   getCellCentroid(int i) { return cells[i].getCentroid();    };

    /* Self-explaining "setters" */
    void setCellState(int i, cellState s) { cells[i].setState(s); };
};

}

#endif // __TICTACTOE_UTILS_H__
