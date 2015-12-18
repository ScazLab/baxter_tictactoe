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
    hsv_color(const color_range &_H, const color_range &_S, const color_range &_V) :
              H(_H), S(_S), V(_V) {};

    cv::Scalar get_hsv_min() { return cv::Scalar(H.min, S.min, V.min); };
    cv::Scalar get_hsv_max() { return cv::Scalar(H.max, S.max, V.max); };
};

struct Cell
{
public:
    std::vector<cv::Point> contours;
    cellState state;
    double cell_area_red;
    double cell_area_blue;

public:
    Cell() { state=empty; cell_area_red=0; cell_area_blue=0; };
    Cell(std::vector<cv::Point> _vec) : contours(_vec) {};
    ~Cell() {};

    std::vector<cv::Point> get_contours() { return contours; };
    
    /**
     * Returns a mask for a cell, i.e. a new image that keeps only the portion
     * delimited by the cell (the rest is set to black)
     * @param      the original image, where the mask will be extracted from.
     * @return     the mask. It has the same size than the original image.
     */
    cv::Mat mask_image(const cv::Mat &);

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

    bool resetState();

    std::string printState();

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

    /**
     * Returns a mask for a board, i.e. a new image that keeps only the portion
     * delimited by the board (the rest is set to black)
     * @param      the original image, where the mask will be extracted from.
     * @return     the mask. It has the same size than the original image.
     */
    cv::Mat mask_image(const cv::Mat &);
};

/**
 * This function thresholds the HSV image and create a binary image
 * @param  img_hsv [description]
 * @param  lower   [description]
 * @param  upper   [description]
 * @return         [description]
 */
cv::Mat hsv_threshold(const cv::Mat& _src, hsv_color _hsv)
{
    cv::Mat res = _src.clone();

    // If H.lower is higher than H.upper it means that we would like to
    // detect something in the range [0-upper] & [lower-180] (i.e. the red)
    // So the thresholded image will be made with two opencv calls to inRange
    // and then the two will be merged into one
    if (_hsv.H.min > _hsv.H.max)
    {
        cv::Mat resA = _src.clone();
        cv::Mat resB = _src.clone();

        cv::inRange(_src, cv::Scalar(         0, _hsv.S.min, _hsv.V.min),
                          cv::Scalar(_hsv.H.max, _hsv.S.max, _hsv.V.max), resA);
        cv::inRange(_src, cv::Scalar(_hsv.H.min, _hsv.S.min, _hsv.V.min),
                          cv::Scalar(       180, _hsv.S.max, _hsv.V.max), resB);

        cv::bitwise_or(resA,resB,res);
    }
    else
    {
        cv::inRange(_src, cv::Scalar(_hsv.get_hsv_min()),
                          cv::Scalar(_hsv.get_hsv_max()), res);
    }
   

    return res;
}

}

#endif // TTT_DEFINITIONS_H
