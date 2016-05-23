#ifndef BOARD_CELLS_DELIMITATION_AUTO_H
#define BOARD_CELLS_DELIMITATION_AUTO_H

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

enum Index {
	LARGEST 		= 1,
	NEXT_LARGEST    = 2
};

class cellDelimitation
{

private:

	ros::NodeHandle node_handle;
	image_transport::ImageTransport image_transport;
	image_transport::Subscriber image_subscriber;

	std::string window_name;
	ttt::Cell cell;
	ttt::Board board;


    /**
     * @param      vector (i.e array) of contours, type indicating whether largest or  
     * 			   next largest area is to be found where (LARGEST = largest area, 
     * 			   NEXT_LARGEST = next largest area)
     * @return     index of the contour with the largest area or the next largest area
     */
	// int getIthIndex(std::vector<std::vector<cv::Point> > contours, Index ith);
    
    /**
     * @param      contour 
     * @return     centroid of the contour
     */
	// cv::Point findCentroid(std::vector<cv::Point> contour);
	
public:

	cellDelimitation();
	~cellDelimitation();

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
}; 

}

#endif //BOARD_CELLS_DELIMITATION_AUTO_H
