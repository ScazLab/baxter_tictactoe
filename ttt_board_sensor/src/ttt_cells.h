#ifndef TTT_CELLS_H
#define TTT_CELLS_H

/** Class used to manipulate general funtions on cells
  *
  *
  */
#include <ros/ros.h>
//#include <ros/console.h>
//#include <ros/assert.h>
//#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QFile>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFileDialog>
#include <QApplication>

#include "ttt_definitions.h"

namespace ttt {

class Cells
{
public:
    static const std::string CELLS_DATA_PARAM_NAME;

    static bool save_to_file(const t_Board& board);

    static bool read_from_parameter_server(t_Board& board, std::string cells_param);

    static bool get_centroid_of_cell(const t_Cell& cell, cv::Point& p);

    static cv::Mat masked_cell_image(const cv::Mat img, const t_Cell cell);
};

}

#endif // TTT_CELLS_H
