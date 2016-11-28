#include "tictactoeBrain.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ttt_brain");

    ttt::tictactoeBrain brain("ttt_brain", "smart");

    ros::spin();
    return 0;
}
