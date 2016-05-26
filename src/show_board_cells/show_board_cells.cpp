#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>
#include <string>

#include <QFile>
#include <QXmlStreamReader>

#include <boost/lexical_cast.hpp>

#include "baxterTictactoe/tictactoe_utils.h"

#include "baxter_tictactoe/DefineCells.h"

using namespace ttt;
using namespace std;
using namespace baxter_tictactoe;

bool BOARD_LOADED = false; 

class CellDisplay
{
private:

    ros::NodeHandle nh_;    
    ros::ServiceClient client;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    const short POINT_RADIUS;

    static const char WINDOW[];

    std::string board_config;

    ttt::Board board; // A vector of cells representing the board game
    ttt::Cell cell;

public:
    CellDisplay()
        : it_(nh_),POINT_RADIUS(5)
    {
        image_sub_ = it_.subscribe("image_in", 1, &CellDisplay::image_callback, this);

        // if(!board.load("/board_file"))
        // {
        //     ROS_FATAL_STREAM("No cell data to display!");
        //     ROS_BREAK();
        // }

        cv::namedWindow(CellDisplay::WINDOW);
    }

    ~CellDisplay()
    {
        cv::destroyWindow(CellDisplay::WINDOW);
    }

    string int_to_string( const int a )
    {
        stringstream ss;
        ss << a;
        return ss.str();
    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        board.cells.clear();
        ROS_INFO("[imageCallback(client node)] Image callback has been successfully executed");

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

        client = nh_.serviceClient<DefineCells>("define_cells");
        ROS_ASSERT_MSG(client, "Empty client");

        DefineCells srv;

        if(client.call(srv))
        {
            int cells_num = srv.response.board.cells.size();
            ROS_INFO("(1) cells_num: %d", cells_num); 
            for(int i = 0; i < cells_num; i++)
            {
                cell.contours.clear();
                int edges_num = srv.response.board.cells[i].contours.size();
                ROS_INFO("(2) edges_num: %d", edges_num); 
                for(int j = 0; j < edges_num; j++)
                {
                    ROS_INFO("(3) Cell %d Edge %d [X: %0.2f Y:%0.2f]", i + 1, j + 1, 
                            srv.response.board.cells[i].contours[j].x, srv.response.board.cells[i].contours[j].y);
                    
                    cv::Point point(srv.response.board.cells[i].contours[j].x, srv.response.board.cells[i].contours[j].y);
                    cell.contours.push_back(point);
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
                board.cells.push_back(cell);
            }

            ROS_INFO("[defineCells CLIENT] Displaying response received:");
            for(int i = 0; i < board.cells.size(); i++)
            {
                ROS_INFO("Board cell: %d State: %d", i + 1, board.cells[i].state);
                for(int j = 0; j < board.cells[i].contours.size(); j++)
                {
                    ROS_INFO("Edge %d: [X:%d Y:%d]", j + 1, board.cells[i].contours[j].x, board.cells[i].contours[j].y);
                }
            }
            ROS_INFO("[defineCells CLIENT] Board data was successfully requested from service node");
        }
        else
        {
            ROS_ERROR("[defineCells CLIENT] Service node was not able to send data");
            return;
        }     

        cv::Mat img_aux = cv_ptr->image.clone();

        // drawing all cells of the board game
        cv::drawContours(img_aux,board.as_vector_of_vectors(),-1, cv::Scalar(123,125,0),2); // drawing just the borders
        for(int i = 0; i < board.cells.size(); i++)
        {
            cv::Point cell_centroid;
            board.cells[i].get_cell_centroid(cell_centroid);
            //cv::circle(img_aux, p,5,cv::Scalar(0,0, 255),-1);
            // cv::line(img_aux, cell_centroid, cell_centroid, cv::Scalar(255,255,0), 2, 8);
            cv::putText(img_aux, int_to_string(i+1), cell_centroid, cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(255,255,0));
        }

        // for(size_t i=0;i!=board.cells.size();++i)
        // {
        //     cv::Point cell_centroid;
        //     board.cells[i].get_cell_centroid(cell_centroid);
        //     //cv::circle(img_aux, p,5,cv::Scalar(0,0, 255),-1);
        //     cv::putText(img_aux, boost::lexical_cast<std::string>(i+1), cell_centroid, cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(255,255,0));
        // }

        cv::putText(img_aux, "Press 's' key to see the filtered images for each cell",
                    cv::Point(10,400), cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::imshow(CellDisplay::WINDOW, img_aux);

        int c = cv::waitKey(3);
        if( (c & 255) == 27 ) // ESC key pressed
        {
            ROS_INFO_STREAM("Exiting with ESC key ..." << board.cells.size() << " cells selected");
            ros::shutdown();
        }
        else if ((char)c =='s')
        {
            this->sensing_cells(cv_ptr->image);
        }
        ROS_INFO("[imageCallback(client node)] Image callback has been successfully executed");
    }

    void sensing_cells(const cv::Mat& img)
    {
        ROS_DEBUG("@sensing_cells");
        short int counter=0;
        foreach (Cell cell, board.cells) {
            cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
            cv::drawContours(mask, std::vector<std::vector<cv::Point> >(1,cell.contours), -1, cv::Scalar(255), CV_FILLED);  // CV_FILLED fills the connected components found with white (white RGB value = 255,255,255)

            cv::Mat im_crop(img.rows, img.cols, CV_8UC3);                           // let's create a new 8-bit 3-channel image with the same dimensions
            im_crop.setTo(cv::Scalar(0));                                           // we fill the new image with a color, in this case we set background to black.
            img.copyTo(im_crop, mask);                                              // copy just the elements from the original image indicated by the non-zero elements from mask to crop
            cv::normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);      // normalize so imwrite(...)/imshow(...) shows the mask correctly!

            // show the images
            /*cv::imshow("original", img);
            cv::imshow("mask", mask);*/
            std::string win_name="cell";
            win_name+=boost::lexical_cast<std::string>(++counter);
            cv::imshow(win_name, im_crop);
        }
    }
};

const char CellDisplay::WINDOW[] = "Cell display";


int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_cells");
    CellDisplay cd;
    ros::spin();
    return 0;
}
