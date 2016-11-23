#include "boardState.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "board_state_sensor");

    // Very dirty way to process command line arguments. It seems that
    // there is not a straightforward standard ROS way, unfortunately.
    // (by alecive, all the fault goes to him)

    bool show=false;
    if (argc>1)
    {
        if (std::string(argv[1])=="--show")
        {
            show=std::string(argv[2])=="true"?true:false;
        }
    }

    BoardState bs("/baxter_tictactoe", show);

    ros::spin();

    return 0;
}
