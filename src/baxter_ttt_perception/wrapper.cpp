#include <stdio.h>

#include <ros/ros.h>

#include "robot_perception/cartesian_estimator_hsv.h"
#include <baxter_ttt_perception/tile_detection.h>
#include "baxter_ttt_perception/ttt_executable.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ttt_executable");
    ros::NodeHandle _n("ttt_executable");

    CartesianEstimatorHSV ce_ttt("ttt_executable");
    ROS_INFO("READY!\n");

    ros::spin();
    return 0;
}

