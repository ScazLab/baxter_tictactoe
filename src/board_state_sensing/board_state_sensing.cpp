#include "board_state_sensing.h"

using namespace ttt;
using namespace baxter_tictactoe;
using namespace std;

bool operator==(ttt_board_sensor::ttt_board msg1,ttt_board_sensor::ttt_board msg2)
{
    if (msg1.data.size()!=msg2.data.size()) return false;

    for (size_t i = 0; i < msg1.data.size(); ++i) {
        if (msg1.data[i]!=msg2.data[i]) {
            return false;
        }
    }
    return true;
}

bool operator!=(ttt_board_sensor::ttt_board msg1,ttt_board_sensor::ttt_board msg2)
{
    return !(msg1==msg2);
}

void BoardState::init()
{
    image_subscriber = image_transport.subscribe("image_in", 1, &BoardState::imageCallback, this);
    
    client = node_handle.serviceClient<DefineCells>("define_cells");
    ROS_ASSERT_MSG(client, "Empty client");

    board_publisher  = node_handle.advertise<ttt_board_sensor::ttt_board>("/new_board", 1);
    ROS_ASSERT_MSG(board_publisher,"Empty publisher");

    DefineCells srv;

    int cells_num = srv.response.board.cells.size();
    for(int i = 0; i < cells_num; i++)
    {
        int edges_num = srv.response.board.cells[i].contours.size();
        for(int j = 0; j < edges_num; i++){
            cell.contours[j].x = srv.response.board.cells[i].contours[j].x;
            cell.contours[j].y = srv.response.board.cells[i].contours[j].y;
        }

        switch(srv.response.board.cells[i].state)
        {
            case BoardCell::EMPTY:
                cell.state = empty;
                break;
            case BoardCell::RED:
                cell.state = red;
                break;
            case BoardCell::BLUE:
                cell.state = blue;
                break;
            case BoardCell::UNDEFINED:
                cell.state = undefined;
                break;
        }
        // cell.state = srv.response.board.cells[i].state;
        board.cells.push_back(cell);
    }
    
    // ROS_INFO("Red  tokens in\t%s", hsv_red.toString().c_str());
    // ROS_INFO("Blue tokens in\t%s", hsv_blue.toString().c_str());
    // ROS_INFO("Area threshold: %g", area_threshold);
    // ROS_INFO("Show param set to %i", doShow);

    if (doShow)
    {
        cv::namedWindow("red  masked image of the board");
        cv::namedWindow("blue masked image of the board");
    }
}

BoardState::BoardState()          : image_transport(node_handle), doShow(false) { init(); };
BoardState::BoardState(bool _show): image_transport(node_handle), doShow(_show) { init(); };

BoardState::~BoardState()
{
    if (doShow)
    {
        cv::destroyWindow("red  masked image of the board");
        cv::destroyWindow("blue masked image of the board");
    }
}

void BoardState::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    board.resetState();

    //converting ROS image format to opencv image format
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ttt_board_sensor::ttt_board msg_board;
    msg_board.data.assign(undefined); // initially the state of each cell is undefined
    msg_board.header.stamp = msg->header.stamp;
    msg_board.header.frame_id = msg->header.frame_id;

    // convert the original image to hsv color space
    cv::Mat img_hsv;
    cv::cvtColor(cv_ptr->image,img_hsv,CV_BGR2HSV);

    // mask the original image to the board
    cv::Mat img_hsv_mask = board.mask_image(img_hsv);

    for (int i = 0; i < 2; ++i)
    {
        cv::Mat hsv_filt_mask = hsv_threshold(img_hsv_mask, i==0?hsv_red:hsv_blue);
        if (doShow)
        {
            if (i==0) cv::imshow("red  masked image of the board",hsv_filt_mask);
            if (i==1) cv::imshow("blue masked image of the board",hsv_filt_mask);
        }

        for (int j = 0; j < board.cells.size(); ++j)
        {
            Cell *cell = &(board.cells[j]);
            cv::Mat crop = cell->mask_image(hsv_filt_mask);

            /* we smooth the image to reduce the noise */
            cv::GaussianBlur(crop.clone(),crop,cv::Size(3,3),0,0);

            // the area formed by the remaining pixels is computed based on the moments
            double cell_area=cv::moments(crop,true).m00;

            if (cell_area > area_threshold)
            {
                if (i==0)  cell->cell_area_red =cell_area;
                else       cell->cell_area_blue=cell_area;
            }
        }
    }

    for (int j = 0; j < board.cells.size(); ++j)
    {
        Cell *cell = &(board.cells[j]);
        if (cell->cell_area_red || cell->cell_area_blue)
        {
            cell->cell_area_red>cell->cell_area_blue?cell->state=red:cell->state=blue;
        }

        msg_board.data[j]=cell->state;
    }

    ROS_DEBUG("Board state is %s", board.stateToString().c_str());

    if(last_msg_board!=msg_board)
    {
        board_publisher.publish(msg_board);
        last_msg_board=msg_board;
        ROS_DEBUG("NEW TTT BOARD STATE PUBLISHED");
    }

    if (doShow) cv::waitKey(50);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_cells");
    ros::NodeHandle n;

    // Very dirty way to process command line arguments. It seems that
    // there is not a straightforward standard ROS way, unfortunately.
    // (by alecive, all the fault goes to him)

    bool show=false;
    if (argc>1)
    {
        if (string(argv[1])=="--show")
        {
            show=string(argv[2])=="true"?true:false;
        }
    }

    BoardState cd(show);
    ros::spin();

    return 0;
}

    // board_publisher  = node_handle.advertise<ttt_board_sensor::ttt_board>("/new_board", 1);
    // ROS_ASSERT_MSG(board_publisher,"Empty publisher");

    // // Reading cells definition data from the parameter server
    // ROS_ASSERT_MSG(board.load("/baxter_tictactoe/board_file"),"No cell data to display!");

    // XmlRpc::XmlRpcValue hsv_red_symbols;
    // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/hsv_red",hsv_red_symbols),
    //                                                       "No HSV params for RED!");
    // hsv_red=hsvColorRange(hsv_red_symbols);

    // XmlRpc::XmlRpcValue hsv_blue_symbols;
    // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/hsv_blue",hsv_blue_symbols),
    //                                                        "No HSV params for BLUE!");
    // hsv_blue=hsvColorRange(hsv_blue_symbols);

    // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/area_threshold",area_threshold),
    //                                                                 "No area threshold!");