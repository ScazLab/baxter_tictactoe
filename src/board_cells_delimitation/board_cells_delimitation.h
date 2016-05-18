#ifndef BOARD_CELLS_DELIMITATION_H
#define BOARD_CELLS_DELIMITATION_H

#include <string>

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

#include "baxterTictactoe/tictactoe_utils.h"

namespace ttt
{

class cellDelimitation
{
private:

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_subscriber;

    short point_radius;
    std::string window_name;

    ttt::Cell   cell; // A single cell being instantiated temporarily
    ttt::Board board; // A vector of cells representing the board game

    bool remove_point(const cv::Point & p);
    bool point_is_inside_cell(const cv::Point & p);
    void show_tutorial(cv::Mat& img);
    void crop_cells(cv_bridge::CvImageConstPtr& , const std::vector<std::vector<cv::Point> > );

public:
    cellDelimitation();
    ~cellDelimitation();

    /* mouse event handler function */
    static void onMouseClick( int event, int x, int y, int, void* param);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};

}

#endif //BOARD_CELLS_DELIMITATION_H
