/*
 * cellsDefinition
 * ------------------------------
 *
 * cellsDefinition identifies and then advertises the boundaries (contours)
 * of the tictactoe board and its cells
 *
 */

#ifndef __CELLS_DEFINITION__
#define __CELLS_DEFINITION__

#include <pthread.h>
#include <iostream>
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

#include "robot_interface/ros_thread_image.h"
#include "baxterTictactoe/tictactoe_utils.h"

#include "baxter_tictactoe/DefineCells.h"
#include "baxter_tictactoe/MsgCell.h"
#include "baxter_tictactoe/MsgBoard.h"
#include "baxter_tictactoe/Point.h"

enum Index
{
    LARGEST         = 1,
    NEXT_LARGEST    = 2
};

class cellsDefinition : public ROSThreadImage
{

private:

    ros::ServiceServer service;

    ttt::Cell cell;
    ttt::Board board;

    bool img_loaded;

    pthread_mutex_t mutex_b;

    ros::Rate r;

    /**
     * @param      vector (i.e array) of contours, type indicating whether largest or
     *             next largest area is to be found where (LARGEST = largest area,
     *             NEXT_LARGEST = next largest area)
     *
     * @return     index of the contour with the largest area or the next largest area
     */
    int getIthIndex(std::vector<std::vector<cv::Point> > contours, Index ith);

    std::string intToString( const int a );

    static bool ascendingY(std::vector<cv::Point> i, std::vector<cv::Point> j);

    static bool ascendingX(std::vector<cv::Point> i, std::vector<cv::Point> j);

protected:
    void InternalThreadEntry();

public:

    cellsDefinition(std::string name);
    ~cellsDefinition();

    /**
     * callback function executed whenever a new message (raw image) from the usb_cam node is received
     * and identifies the boundaries of the tictactoe board and its cells
     *
     * @param      raw image from camera
     *
     * @return     N/A
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    /**
     * service that provides data on the defined contours/boundaries of the board's cells
     *
     * @param      request variables and response variables
     *
     * @return     returns true when function is succesfully executed
     */
    bool serviceCb(baxter_tictactoe::DefineCells::Request  &req,
                   baxter_tictactoe::DefineCells::Response &res);
};

#endif //__CELLS_DEFINITION__
