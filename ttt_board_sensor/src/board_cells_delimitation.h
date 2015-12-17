#ifndef BOARD_CELLS_DELIMITATION_H
#define BOARD_CELLS_DELIMITATION_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QFile>
#include <QXmlStreamWriter>
#include <QFileDialog>
#include <QApplication>

#include "ttt/tictactoe_utils.h"
#include "ttt_cells.h"

namespace ttt
{

class cellDelimitation
{
private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    const short POINT_RADIUS;

    static const char WINDOW[];

    ttt::Cell points; // A vector of points delimiting a cell
    ttt::Board board; // A vector of cells representing the board game

    bool remove_point(const cv::Point & p);
    bool point_is_inside_cell(const cv::Point & p);
    void show_how_to(cv::Mat& img);
    static void crop_cells(cv_bridge::CvImageConstPtr& , const std::vector<std::vector<cv::Point> > );

public:
    cellDelimitation();
    ~cellDelimitation();

    /* mouse event handler function */
    static void onMouseClick( int event, int x, int y, int, void* param);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};

const char cellDelimitation::WINDOW[] = "Cell delimitation";

}

#endif //BOARD_CELLS_DELIMITATION_H
