#include "tictactoeBrain.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tictactoe_brain");

    ttt::tictactoeBrain brain("ttt_controller", "smart");

    ros::spin();
    return 0;
}
