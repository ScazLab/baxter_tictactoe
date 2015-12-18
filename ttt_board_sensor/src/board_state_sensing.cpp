
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/lexical_cast.hpp>

#include "ttt/tictactoe_utils.h"
#include "ttt_board_sensor/ttt_board.h"

namespace ttt
{

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

class BoardState
{
private:

    ros::NodeHandle node_handle;
    ros::Publisher  board_publisher;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber     image_subscriber;

    std::string board_config;

    ttt::Board board;

    double area_threshold;

    hsvColorRange hsv_red;
    hsvColorRange hsv_blue;

    // Last TTT board state message sent. Used to avoid the publication of the same board state messages.
    ttt_board_sensor::ttt_board last_msg_board; 

public:
    BoardState() : image_transport(node_handle)
    {
        image_subscriber = image_transport.subscribe("image_in", 1, &BoardState::image_callback, this);
        board_publisher  = node_handle.advertise<ttt_board_sensor::ttt_board>("/new_board", 1);
        ROS_ASSERT_MSG(board_publisher,"Empty publisher");

        // Reading cells definition data from the parameter server
        ROS_ASSERT_MSG(board.load("/baxter_tictactoe/board_file"),"No cell data to display!");

        XmlRpc::XmlRpcValue hsv_red_symbols;
        ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/hsv_red",hsv_red_symbols),
                                                              "No HSV params for RED!");
        hsv_red.fromROSparam(hsv_red_symbols);

        XmlRpc::XmlRpcValue hsv_blue_symbols;
        ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/hsv_blue",hsv_blue_symbols),
                                                               "No HSV params for BLUE!");
        hsv_blue.fromROSparam(hsv_blue_symbols);

        // // Reading the segmentation thresholds for red tokens
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/h_low_red",hsv_red.H.min),
        //                                           "No H LOW threshold for RED tokens!");
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/s_low_red",hsv_red.S.min),
        //                                           "No S LOW threshold for RED tokens!");
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/v_low_red",hsv_red.V.min),
        //                                           "No V LOW threshold for RED tokens!");
        
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/h_high_red",hsv_red.H.max),
        //                                           "No H HIGH threshold for RED tokens!");
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/s_high_red",hsv_red.S.max),
        //                                           "No S HIGH threshold for RED tokens!");
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/v_high_red",hsv_red.V.max),
        //                                           "No V HIGH threshold for RED tokens!");

        // Reading the segmentation thresholds for blue tokens 
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/h_low_blue",hsv_blue.H.min),
        //                                            "No H LOW threshold for BLUE tokens!");
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/s_low_blue",hsv_blue.S.min),
        //                                            "No S LOW threshold for BLUE tokens!");
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/v_low_blue",hsv_blue.V.min),
        //                                            "No V LOW threshold for BLUE tokens!");

        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/h_high_blue",hsv_blue.H.max),
        //                                            "No H HIGH threshold for BLUE tokens!");
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/s_high_blue",hsv_blue.S.max),
        //                                            "No S HIGH threshold for BLUE tokens!");
        // ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/v_high_blue",hsv_blue.V.max),
        //                                            "No V HIGH threshold for BLUE tokens!");

        ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/area_threshold",area_threshold),
                                                                        "No area threshold!");
        
        ROS_INFO("Red  tokens in\t%s", hsv_red.toString().c_str() );
        ROS_INFO("Blue tokens in\t%s", hsv_blue.toString().c_str());
        ROS_INFO("Area threshold: %g",area_threshold);

        // cv::namedWindow("red  masked image to board");
        // cv::namedWindow("blue masked image to board");
    }

    ~BoardState()
    {
    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
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
            cv::Mat hsv_filt_mask = ttt::hsv_threshold(img_hsv_mask, i==0?hsv_red:hsv_blue);
            // if (i==0) cv::imshow("red  masked image to board",hsv_filt_mask);
            // if (i==1) cv::imshow("blue masked image to board",hsv_filt_mask);

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

        // cv::waitKey(50);
    }
};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_cells");
    ttt::BoardState cd;
    ros::spin();
    return 0;
}
