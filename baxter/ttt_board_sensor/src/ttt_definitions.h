#ifndef TTT_DEFINITIONS_H
#define TTT_DEFINITIONS_H

namespace enc = sensor_msgs::image_encodings;

namespace ttt
{

typedef std::vector<cv::Point> t_Cell;    // vector of points delimiting a cell
typedef std::vector<t_Cell> t_Board;   // vector of cells, i.e. a vector of vectors of points

typedef enum {empty=0, blue, red} t_Cell_State; // used to determine the three possible states of a cell

}

#endif // TTT_DEFINITIONS_H
