
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/lexical_cast.hpp>

#include "ttt/ttt_definitions.h"
#include "ttt_cells.h"
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

struct color_range {
    int min;
    int max;
};

struct hsv_color {
    color_range H;
    color_range S;
    color_range V;
};

class BoardState
{
private:

    ros::NodeHandle node_handle;
    ros::Publisher  board_publisher;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber     image_subscriber;

    std::string board_config;

    t_Board board; // A vector of cells representing the board game

    double TOKEN_AREA_THRESHOLD;

    hsv_color red_params;
    hsv_color blue_params;

    double get_hsv_colored_area_in_cell(const cv::Mat& img, const t_Cell& cell, const cv::Scalar& lower_values, const cv::Scalar& higher_values) const
    {
        /* we extract the cell from the original image */
        cv::Mat cropped_cell = Cells::masked_cell_image(img,cell);
        // cv::namedWindow("cropped", CV_WINDOW_AUTOSIZE);
        // cv::imshow("cropped",cropped_cell);
        // cv::waitKey();

        /* converting to hsv color space */
        cv::Mat aux_img;
        cv::cvtColor(cropped_cell,aux_img,CV_BGR2HSV);
        // cv::namedWindow("hsv", CV_WINDOW_AUTOSIZE);
        // cv::imshow("hsv",aux_img);
        // cv::waitKey();

        /* extracting just the pixeles within the hsv range values */
        cv::inRange(aux_img.clone(),lower_values,higher_values,aux_img);
        //cv::namedWindow("red range");
        //cv::imshow("red range",aux_img);

        /* we smooth the image to reduce the noise */
        cv::GaussianBlur(aux_img.clone(),aux_img,cv::Size(3,3),0,0);
        //cv::namedWindow("smoothed");
        //cv::imshow("smoothed",aux_img);

        /* the area formed by the remaining pixeles are computed based on the moments*/
        cv::Moments segmented_moments = cv::moments(aux_img,true);
        // ROS_DEBUG_STREAM("segmented area=" << segmented_moments.m00);

        //cv::waitKey(0);
        return segmented_moments.m00; //m00 represents the area
    }

    ttt_board_sensor::ttt_board last_msg_board; //! Last TTT board state message sent. This is used to avoid the publication of the same board state messages.

public:
    BoardState()
        : image_transport(node_handle)
    {
        image_subscriber = image_transport.subscribe("image_in", 1, &BoardState::imageCb, this);
        board_publisher  = node_handle.advertise<ttt_board_sensor::ttt_board>("/new_board", 1);
        ROS_ASSERT_MSG(board_publisher,"Empty publisher");

        /* Reading cells definition data from the parameter server */
        ROS_ASSERT_MSG(Cells::read_from_parameter_server(board,Cells::CELLS_DATA_PARAM_NAME),"No cell data to display!");

        /* Reading the segmentation thresholds for red tokens */
        ROS_ASSERT_MSG(node_handle.hasParam("h_low_red"),"No H LOW threshold for RED tokens!");
        node_handle.getParam("h_low_red",red_params.H.min);
        ROS_ASSERT_MSG(node_handle.hasParam("s_low_red"),"No S LOW threshold for RED tokens!");
        node_handle.getParam("s_low_red",red_params.S.min);
        ROS_ASSERT_MSG(node_handle.hasParam("v_low_red"),"No V LOW threshold for RED tokens!");
        node_handle.getParam("v_low_red",red_params.V.min);
        ROS_ASSERT_MSG(node_handle.hasParam("h_high_red"),"No H HIGH threshold for RED tokens!");
        node_handle.getParam("h_high_red",red_params.H.max);
        ROS_ASSERT_MSG(node_handle.hasParam("s_high_red"),"No S HIGH threshold for RED tokens!");
        node_handle.getParam("s_high_red",red_params.S.max);
        ROS_ASSERT_MSG(node_handle.hasParam("v_high_red"),"No V HIGH threshold for RED tokens!");
        node_handle.getParam("v_high_red",red_params.V.max);
        ROS_INFO_STREAM("Red tokens in H=[" << red_params.H.min<< ".." << red_params.H.max <<
                        "] S=[" << red_params.S.min << ".." << red_params.S.max <<
                        "] V=[" << red_params.V.min << ".." << red_params.V.max<< "]");

        /* Reading the segmentation thresholds for blue tokens */
        ROS_ASSERT_MSG(node_handle.hasParam("h_low_blue"),"No H LOW threshold for BLUE tokens!");
        node_handle.getParam("h_low_blue",blue_params.H.min);
        ROS_ASSERT_MSG(node_handle.hasParam("s_low_blue"),"No S LOW threshold for BLUE tokens!");
        node_handle.getParam("s_low_blue",blue_params.S.min);
        ROS_ASSERT_MSG(node_handle.hasParam("v_low_blue"),"No V LOW threshold for BLUE tokens!");
        node_handle.getParam("v_low_blue",blue_params.V.min);
        ROS_ASSERT_MSG(node_handle.hasParam("h_high_blue"),"No H HIGH threshold for BLUE tokens!");
        node_handle.getParam("h_high_blue",blue_params.H.max);
        ROS_ASSERT_MSG(node_handle.hasParam("s_high_blue"),"No S HIGH threshold for BLUE tokens!");
        node_handle.getParam("s_high_blue",blue_params.S.max);
        ROS_ASSERT_MSG(node_handle.hasParam("v_high_blue"),"No V HIGH threshold for BLUE tokens!");
        node_handle.getParam("v_high_blue",blue_params.V.max);
        ROS_INFO_STREAM("Blue tokens in H=[" << blue_params.H.min<< ".." << blue_params.H.max <<
                        "] S=[" << blue_params.S.min << ".." << blue_params.S.max <<
                        "] V=[" << blue_params.V.min << ".." << blue_params.V.max<< "]");

        ROS_ASSERT_MSG(node_handle.hasParam("color_area_threshold"),"No color area threshold!");
        node_handle.getParam("color_area_threshold",TOKEN_AREA_THRESHOLD);
    }

    ~BoardState()
    {
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
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
        short unsigned int counter=0;
        foreach (t_Cell cell, this->board)
        {
            /* computing the red area in the cell */
            double red_cell_area = this->get_hsv_colored_area_in_cell(cv_ptr->image,cell,
                                                                      cv::Scalar(red_params.H.min,red_params.S.min,red_params.V.min),
                                                                      cv::Scalar(red_params.H.max,red_params.S.max,red_params.V.max));
            ROS_DEBUG_STREAM("Red area of cell " << counter+1 << " = " << red_cell_area);

            /* computing the blue area in the cell */
            double blue_cell_area = this->get_hsv_colored_area_in_cell(cv_ptr->image,cell,
                                                                      cv::Scalar(blue_params.H.min,blue_params.S.min,blue_params.V.min),
                                                                      cv::Scalar(blue_params.H.max,blue_params.S.max,blue_params.V.max));
            ROS_DEBUG_STREAM("Blue area of cell " << counter+1 << " = " << blue_cell_area);

            /* determining the state of the cell considering that two tokens can be heaped */
            t_Cell_State cell_state;
            if (red_cell_area > this->TOKEN_AREA_THRESHOLD || blue_cell_area > this->TOKEN_AREA_THRESHOLD)
            {
                red_cell_area>blue_cell_area?cell_state=red:cell_state=blue;
            }
            else cell_state=empty;

            ROS_DEBUG_STREAM_COND(cell_state==empty, "Cell " << counter+1 << " is EMPTY");
            ROS_DEBUG_STREAM_COND(cell_state==red, "Cell " << counter+1 << " is RED");
            ROS_DEBUG_STREAM_COND(cell_state==blue, "Cell " << counter+1 << " is BLUE");

            msg_board.data[counter]=cell_state;

            counter++;
        }
        if(last_msg_board!=msg_board)
        {
            board_publisher.publish(msg_board);
            last_msg_board=msg_board;
            ROS_DEBUG("NEW TTT BOARD STATE PUBLISHED");
        }
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
