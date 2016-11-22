#include "boardState.h"
#include "cellsDefinition.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_cells");
    ros::AsyncSpinner spinner(4);  // AsyncSpinner to handle callbacks

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

    BoardState bs(show);
    cellsDefinition cd;
    spinner.start();
    ros::spin();

    return 0;
}
