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

class CellDelimitation
{
private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    const short POINT_RADIUS;

    static const char WINDOW[];

    t_Cell points; // A vector of points delimiting a cell
    t_Board board; // A vector of cells representing the board game

    bool remove_point(const cv::Point & p);
    bool remove_cell(const cv::Point & p);
    void show_how_to(cv::Mat& img);
    static void cropping_cells(cv_bridge::CvImageConstPtr& cv_cp_img,const t_Board& a_board);    

public:
    CellDelimitation();
    ~CellDelimitation();

    /* mouse event handler function */
    static void onMouse( int event, int x, int y, int, void* param);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

};

const char CellDelimitation::WINDOW[] = "Cell delimitation";

}

#endif //BOARD_CELLS_DELIMITATION_H
