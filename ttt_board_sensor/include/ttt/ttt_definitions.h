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

// Used to determine the three possible states of a cell.
// Undefined is just used when it is created and non state has been assigned.
typedef enum {empty=ttt_board_sensor::ttt_board::EMPTY,
              blue=ttt_board_sensor::ttt_board::BLUE,
              red=ttt_board_sensor::ttt_board::RED,
              undefined=ttt_board_sensor::ttt_board::UNDEFINED} t_Cell_State;

std::string cell_state_to_str(t_Cell_State c_s)
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

const std::size_t NUMBER_OF_CELLS = 9;

}

#endif // TTT_DEFINITIONS_H
