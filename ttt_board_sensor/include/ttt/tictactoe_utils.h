#ifndef TTT_DEFINITIONS_H
#define TTT_DEFINITIONS_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QFile>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFileDialog>
#include <QApplication>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

#include "ttt_board_sensor/ttt_board.h"

namespace enc = sensor_msgs::image_encodings;

namespace ttt
{

// Used to determine the three possible states of a cell.
// Undefined is just used when it is created and non state has been assigned.
typedef enum {empty=ttt_board_sensor::ttt_board::EMPTY,
              blue=ttt_board_sensor::ttt_board::BLUE,
              red=ttt_board_sensor::ttt_board::RED,
              undefined=ttt_board_sensor::ttt_board::UNDEFINED} cellState;

std::string cell_state_to_str(cellState c_s)
{
    switch(c_s)
    {
        case empty: return std::string("empty");
        case blue: return std::string("blue");
        case red: return std::string("red");
        case undefined: return std::string("undefined");
        default: return std::string("[value unknown]");
    }
}

const std::size_t NUMBER_OF_CELLS=9;
const std::string CELLS_DATA_PARAM_NAME="/baxter_tictactoe/board_file";

struct color_range {
    int min;
    int max;

    color_range(): min(0), max(0) {};
    color_range(int _min, int _max) : min(_min), max(_max) {};
    color_range(const color_range &_cr): min(_cr.min), max(_cr.max) {};
};

struct hsv_color {
    color_range H;
    color_range S;
    color_range V;

    hsv_color(): H(0,180), S(0,256), V(0,256) {};
    hsv_color(color_range &_H, color_range &_S, color_range &_V) :
              H(_H), S(_S), V(_V) {};

    cv::Scalar get_hsv_min() { return cv::Scalar(H.min, S.min, V.min); };
    cv::Scalar get_hsv_max() { return cv::Scalar(H.max, S.max, V.max); };
};

struct Cell
{
public:
    std::vector<cv::Point> contours;
    cellState state;

public:
    Cell() {};
    Cell(std::vector<cv::Point> _vec) : contours(_vec) {};
    ~Cell() {};

    std::vector<cv::Point> get_contours() { return contours; };
    
    /**
    Returns a mask for a cell, i.e. a new image that preserves the size of the original one, but 
    keeps only the portion delimited by the cell (the rest is set to black)
    @param img  the original image, where the mask will be extracted from.
    @param cell the cell defining the mask.
    @return     the mask for a cell. It has the same size than the original image.
     */
    cv::Mat mask_image(const cv::Mat img);

    /**
    Computes the centroid of a cell.
    @param cell the cell whose centroid is going to be computed
    @param centroid the central point.
    @return true/false if success/failure.
    */
    bool get_cell_centroid(cv::Point& centroid);
};

struct Board
{
public: 
    std::vector<ttt::Cell> cells;

public:
    Board()  {};
    ~Board() {};

    std::vector<std::vector<cv::Point> > as_vector_of_vectors();

    /**
    Saves the board to file (whose name will be specified with a dialog window).
    The format of the file is as follows:
        <board>
            <cell id="0">
                <vertex x="195" y="50"/>
                [...]
            </cell>
            [...]
        </board>
    @param board where all the Cells are stored
    @return true/false if success/failure.
     */
    bool save();

    /**
    Reads the contours of the cells from the parameter server, in a param named cells_param.
    @param board where the Cells are going to be stored
    @param cells_param the name of the parameter where the raw data from the cells is stored. This data is formatted as a xml file.
    The format of the data is as follows:
            <board>
                <cell id="0">
                    <vertex x="195" y="50"/>
                    [...]
                </cell>
                [...]
            </board>
    @return true/false if success/failure.
     */
    bool load(std::string cells_param);
};

}

#endif // TTT_DEFINITIONS_H
