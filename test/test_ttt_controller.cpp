#include <ros/ros.h>
#include <ros/console.h>
#include "baxter_tictactoe/ttt_controller.h"

using namespace baxter_tictactoe;
using namespace std;

int main(int argc, char * argv[])
{
    string name = "ttt_controller";
    bool legacy_code = false;

    ros::init(argc, argv, name);

    TTTController  *left_ac = new TTTController(name,  "left", legacy_code);
    TTTController *right_ac = new TTTController(name, "right", legacy_code);

    for(int i = 0; i < 9; i++)
    {
        left_ac->startAction(ACTION_PICKUP);
        left_ac->startAction(ACTION_PUTDOWN, i+1);
    }

    delete left_ac;
    left_ac = NULL;

    delete right_ac;
    right_ac = NULL;

    ros::shutdown();
    return 0;
}

