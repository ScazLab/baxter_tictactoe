#ifndef TTT_DEFINITIONS_H
#define TTT_DEFINITIONS_H

#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

#include "ttt_board_sensor/ttt_board.h"

namespace enc = sensor_msgs::image_encodings;

namespace ttt
{

typedef std::vector<cv::Point> t_Cell;    // vector of points delimiting a cell
typedef std::vector<t_Cell> t_Board;   // vector of cells, i.e. a vector of vectors of points

//typedef enum {empty=0, blue, red, undefined} t_Cell_State; // used to determine the three possible states of a cell. Undefined is just used when it is created and non state has been asigned.
typedef uint8_t t_Cell_State; // used to determine the three possible states of a cell. Undefined is just used when it is created and non state has been asigned.

std::string cell_state_to_str(t_Cell_State c_s)
{
    switch(c_s)
    {
//        case empty: return std::string("empty");
//        case blue: return std::string("blue");
//        case red: return std::string("red");
//        case undefined:
        case ttt_board_sensor::ttt_board::EMPTY: return std::string("empty");
        case ttt_board_sensor::ttt_board::BLUE: return std::string("blue");
        case ttt_board_sensor::ttt_board::RED: return std::string("red");
        case ttt_board_sensor::ttt_board::UNDEFINED:
        default: return std::string("undefined");
    }
}

const std::size_t NUMBER_OF_CELLS = 9;

}

#endif // TTT_DEFINITIONS_H
