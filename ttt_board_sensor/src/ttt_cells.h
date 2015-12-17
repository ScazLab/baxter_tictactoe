#ifndef TTT_CELLS_H
#define TTT_CELLS_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QFile>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFileDialog>
#include <QApplication>

#include "ttt/tictactoe_utils.h"

namespace ttt {

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
    bool load(t_Board& board, std::string cells_param);

    /**
    Computes the centroid of a cell.
    @param cell the cell whose centroid is going to be computed
    @param centroid the central point.
    @return true/false if success/failure.
    */
    bool get_cell_centroid(const t_Cell& cell, cv::Point& p);

    /**
    Returns a mask for a cell, i.e. a new image that preserves the size of the original one, but 
    keeps only the portion delimited by the cell (the rest is set to black)
    @param img  the original image, where the mask will be extracted from.
    @param cell the cell defining the mask.
    @return     the mask for a cell. It has the same size than the original image.
     */
    cv::Mat mask_image(const cv::Mat img, const t_Cell cell);

}

#endif // TTT_CELLS_H
