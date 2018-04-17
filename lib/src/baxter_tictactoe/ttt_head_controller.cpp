#include <baxter_core_msgs/HeadPanCommand.h>

#include "baxter_tictactoe/ttt_head_controller.h"


TTTHeadController::TTTHeadController(ros::NodeHandle nh, float pan) : _pan(pan)
{
    cmd_pub = nh.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan", 1);
}

TTTHeadController::TTTHeadController(ros::NodeHandle nh)
  : TTTHeadController::TTTHeadController(nh, 0.)
{
};

void TTTHeadController::set_pan(float angle)
{
    _pan = angle;
}

void TTTHeadController::pan_to(float angle)
{
    baxter_core_msgs::HeadPanCommand msg;
    msg.target = angle;
    cmd_pub.publish(msg);
}

void TTTHeadController::pan()
{
    TTTHeadController::pan_to(_pan);
}

void TTTHeadController::center()
{
    TTTHeadController::pan_to(0.);
}
