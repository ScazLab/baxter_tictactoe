
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/lexical_cast.hpp>

#include "baxterTictactoe/tictactoe_utils.h"
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

    bool doShow;

    // Last TTT board state message sent. Used to avoid the publication of the same board state messages.
    ttt_board_sensor::ttt_board last_msg_board; 

    void init()
    {

        // [ANNO] hsvColoRange(int min, int max) for red and blue pieces specified in board_state_sensor.launch

        image_subscriber = image_transport.subscribe("image_in", 1, &BoardState::image_callback, this);
        board_publisher  = node_handle.advertise<ttt_board_sensor::ttt_board>("/new_board", 1);
        ROS_ASSERT_MSG(board_publisher,"Empty publisher");

        // Reading cells definition data from the parameter server
        ROS_ASSERT_MSG(board.load("/baxter_tictactoe/board_file"),"No cell data to display!");

        XmlRpc::XmlRpcValue hsv_red_symbols;
        ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/hsv_red",hsv_red_symbols),
                                                              "No HSV params for RED!");
        hsv_red=hsvColorRange(hsv_red_symbols);

        XmlRpc::XmlRpcValue hsv_blue_symbols;
        ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/hsv_blue",hsv_blue_symbols),
                                                               "No HSV params for BLUE!");
        hsv_blue=hsvColorRange(hsv_blue_symbols);


        ROS_ASSERT_MSG(node_handle.getParam("baxter_tictactoe/area_threshold",area_threshold),
                                                                        "No area threshold!");
        
        // [ANNO] printed info upon launch

        ROS_INFO("Red  tokens in\t%s", hsv_red.toString().c_str() );
        ROS_INFO("Blue tokens in\t%s", hsv_blue.toString().c_str());
        ROS_INFO("Area threshold: %g",area_threshold);
        ROS_INFO("Show param set to %i",doShow);

        if (doShow)
        {
            cv::namedWindow("red  masked image of the board");
            cv::namedWindow("blue masked image of the board");
        }
    }

public:
    BoardState()          : image_transport(node_handle), doShow(false) { init(); };
    BoardState(bool _show): image_transport(node_handle), doShow(_show) { init(); };

    ~BoardState()
    {
        if (doShow)
        {
            cv::destroyWindow("red  masked image of the board");
            cv::destroyWindow("blue masked image of the board");
        }
    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        board.resetState();

        //converting ROS image format to opencv image format

        // [ANNO] initialize cv_bridge::CvImageConstPtr w/ name cv_ptr
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            // [ANNO] convert a ROS /sensor_msgs/Image message into a CvImage
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // [ANNO] members defined in /msg/ttt_board.msg

        ttt_board_sensor::ttt_board msg_board;
        msg_board.data.assign(undefined); // initially the state of each cell is undefined
        msg_board.header.stamp = msg->header.stamp;
        msg_board.header.frame_id = msg->header.frame_id;

        // convert the original image to hsv color space

        // [ANNO] initialize undefined cv::Mat w/ name img_hsw
        cv::Mat img_hsv;
        // [ANNO] convert img_hsv's color model to HSV
        cv::cvtColor(cv_ptr->image,img_hsv,CV_BGR2HSV);

        // mask the original image to the board
        cv::Mat img_hsv_mask = board.mask_image(img_hsv);

        for (int i = 0; i < 2; ++i)
        {
            // [ANNO] i==0?hsv_red:hsv_blue
            // [ANNO] when i = 0, arg is hsv_red; when i = 1; arg is hsv_blue
            // [ANNO] extract red/blue elements from img_hsv_mask and store it in cv::Mat hsv_filt_mask
            cv::Mat hsv_filt_mask = ttt::hsv_threshold(img_hsv_mask, i==0?hsv_red:hsv_blue);
            if (doShow)
            {
                if (i==0) cv::imshow("red  masked image of the board",hsv_filt_mask);
                if (i==1) cv::imshow("blue masked image of the board",hsv_filt_mask);
            }

            // [ANNO] for every cell in a board
            for (int j = 0; j < board.cells.size(); ++j)
            {
                Cell *cell = &(board.cells[j]);

                // [ANNO] extract elements within the boundaries (i.e contours) of cell[i]
                // [ANNO] that are red
                cv::Mat crop = cell->mask_image(hsv_filt_mask);

                // [ANNO] we smooth the image to reduce the noise (avoid false positives)
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
};

}

int main(int argc, char** argv)
{
    printf("HI!\n");
    ros::init(argc, argv, "show_cells");

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

    ttt::BoardState cd(show);
    // ttt::BoardState cd();
    ros::spin();

    return 0;
}

/***************** tictactoe_utils ****************/
/*                                                */ 
/*                 hsv_threshold()                */
/*                                                */
/**************************************************/


cv::Mat ttt::hsv_threshold(const cv::Mat& _src, hsvColorRange _hsv)
{
    // [ANNO] make a copy of _src and name it res
    cv::Mat res = _src.clone();

    // If H.lower is higher than H.upper it means that we would like to
    // detect something in the range [0-upper] & [lower-180] (i.e. the red)
    // So the thresholded image will be made with two opencv calls to inRange
    // and then the two will be merged into one
    if (_hsv.H.min > _hsv.H.max)
    {
        // [ANNO] inRange must be called twice because a color may be found
        // [ANNO] in two hue ranges (e.g red can be found in H = 0 to 10 & H = 160 to 180)
        cv::Mat resA = _src.clone();
        cv::Mat resB = _src.clone();

        // [ANNO] extracts every element in cv::Mat _src that is between H = 0 to _hsv.H.max
        // [ANNO] to cv::MAt resA and sets it color value to white (against black bg of resA)
        // [ANNO] (for an e.g, see https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/)
        cv::inRange(_src, cv::Scalar(         0, _hsv.S.min, _hsv.V.min),
                          cv::Scalar(_hsv.H.max, _hsv.S.max, _hsv.V.max), resA);
        cv::inRange(_src, cv::Scalar(_hsv.H.min, _hsv.S.min, _hsv.V.min),
                          cv::Scalar(       180, _hsv.S.max, _hsv.V.max), resB);

        // [ANNO] all white elements in cv::Mat resA and resB have value 1. Hence, (resA OR resB)
        // [ANNO] for all elements will merge the two matrices, stored in cv::Mat res
        cv::bitwise_or(resA,resB,res);
    }
    else
    {
        // [ANNO] similar to above, only that the color does not 'bleed over' from 0 to 180 (e.g red is found
        // [ANNO] from H = 160 to 180 and from H = 0 to 10) and hence only one inRange call is necessary
        cv::inRange(_src, cv::Scalar(_hsv.get_hsv_min()),
                          cv::Scalar(_hsv.get_hsv_max()), res);
    }

    return res;
}
