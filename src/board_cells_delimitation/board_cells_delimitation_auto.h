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

class cellDelimitation
{

private:

	ros::NodeHandle node_handle;
	image_transport::ImageTransport image_transport;
	image_transport::Subscriber image_subscriber;

	std::string window_name;
	ttt::Cell cell;
	ttt::Board board;

public:

	cellDelimitation();
	~cellDelimitation();

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
}; 

}

#endif //BOARD_CELLS_DELIMITATION_AUTO_H
