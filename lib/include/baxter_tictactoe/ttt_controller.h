#ifndef __TTT_CONTROLLERS_H__
#define __TTT_CONTROLLERS_H__

#include <mutex>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <robot_perception/hsv_detection.h>

#include <robot_interface/robot_interface.h>
#include <robot_interface/arm_ctrl.h>
#include <robot_interface/gripper.h>

#include "tictactoe_utils.h"

#define HOVER_BOARD_X   0.575  // [m]
#define HOVER_BOARD_Y   0.100  // [m]
#define HOVER_BOARD_Z   0.445  // [m]

class TTTController : public ArmCtrl
{
private:
    ros::Rate r;

    image_transport::ImageTransport _img_trp;
    image_transport::Subscriber     _img_sub;

    bool _legacy_code;   // Flag to enable the legacy code [who does not work]

    hsvColorRange  hsv_red;
    hsvColorRange hsv_blue;

    geometry_msgs::Point _tiles_pile_pos;

    std::vector<geometry_msgs::Point>  _offsets;   // Legacy, it does not work

    std::vector<geometry_msgs::Point> _board_centers_poss;
    std::vector<geometry_msgs::Point> _board_corners_poss;

    cv::Mat  _curr_img;
    cv::Size _img_size;
    bool _is_img_empty;

    std::mutex mutex_img;

    bool createCVWindows();

    bool destroyCVWindows();

    bool tilesPilePosFromParam(XmlRpc::XmlRpcValue _params);

    bool    boardPossFromParam(XmlRpc::XmlRpcValue _params);

    /**
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();

    /* SCAN BOARD */

        /*
         * Hovers arm above board
         *
         * @return     true/false if success/failure
         */
        bool hoverAboveBoard();

        /**
         * Hovers arm above the board
         *
         * @return true/false if success/failure
         */
        bool hoverAboveCenterOfBoard();

        /**
         * Hovers arm above specified cell
         *
         * @return true/false if success/failure
         */
        bool hoverAboveCell();

        /*
         * Hovers arm above tokens
         *
         * @param      double indicating requested height of arm (z-axis)
         * @return     true/false if success/failure
         */
        bool hoverAboveTokens(double height);

        /*
         * moves arm downwards until collision w/ a surface; calculates
         * distance between surface and arm starting point
         *
         * @param     dist indicates the distance between the surface
         *                 and the arm's starting point
         */
        void setDepth(float &dist);

        /*
         * Calculates cell offsets; also prompts user to move board withing 'reachable zone'
         * (displayed on screen) if board is out of reach of Baxter's arm
         *
         */
        void processImage(float dist);

        /*
         * isolates black colored object in raw image
         *
         * @param      Mat displaying black colored objects in raw image
         */
        void isolateBlack(cv::Mat &output);

        /*
         * isolates board boundaries from image
         *
         * @param     input Mat, output Mat displaying board boundaries, output Contours
         *            storing board contours, integer indicating the area of the board,
         *            and a vector<cv::Point> of the board's four corners
         */
        void isolateBoard(baxter_tictactoe::Contours &contours, int &board_area,
                          std::vector<cv::Point> &board_corners, cv::Mat input, cv::Mat &output);

        /*
         * finds cell with the higher centroid
         *
         * @param      returns true if cell i has a higher centroid than cell j; false otherwise
         */
        static bool descendingX(std::vector<cv::Point> i, std::vector<cv::Point> j);

        /*
         * calculates offset distance from arm to each board cell
         *
         * @param      board area, cell contours, output Mat displaying cells, and height
         *            from arm to board surface
         */
        void setOffsets(int board_area, baxter_tictactoe::Contours contours, float dist,
                        cv::Mat &output, std::vector<cv::Point> &centroids);

        std::vector<geometry_msgs::Point> getOffsets() { return _offsets; };

        /**
         * calculates the perimeter of the area representing all points reachable to the Baxter arm
         *
         * @param contours       board contours
         * @param dist           distance between starting position and play surface
         * @param board_corners  coordinates of all 4 board corners
         * @param c              input vector containing cell centroids
         * @param cell_to_corner vector representing distance between center of corner cell
         *                       and corner of corner cell
         */
        void setZone(baxter_tictactoe::Contours contours, float dist, std::vector<cv::Point> board_corners,
                     std::vector<cv::Point> c, std::vector<cv::Point> &cell_to_corner);

        /*
         * checks if Baxter's arm has a joint angles solution for all the calculated cell offsets
         *
         * @return     true if offsets are all reachable; false otherwise
         */
        bool offsetsReachable();

        /*
         * checks if Baxter's arm has a joint angles solution for a certain point on the board
         * scanning image
         *
         * @return     true if point is reachable; false otherwise
         */
        bool pointReachable(cv::Point centroid, float dist);

    /* PICKUP TOKEN */
        /*
         * Picks up the token from the pile of tokens
         *
         * @return     true/false if success/failure
         */
        bool pickUpToken();

        /*
         * identifies token and calculates offset distance required to move hand camera
         * to token
         *
         * @param     offset offset between the arm's x-y coordinates and the token
         *
         * @return     true/false if success/failure
         */
        bool computeTokenOffset(cv::Point &offset);

        /*
         * isolates blue colored object in raw image
         *
         * @param      Mat displaying blue colored objects in raw image
         */
        void isolateBlue(cv::Mat &output);

        /*
         * Detects the pool of tokens in the image
         *
         * @return  Binary matrix displaying the pool of objects
         */
        cv::Mat detectPool();

        /*
         * Isolates token from the pool
         *
         * @param       pool input matrix displaying the pool of objects
         * @return      output Mat displaying token,
         */
        cv::Mat isolateToken(cv::Mat pool);


    bool pickUpTokenImpl();

    bool scanBoardImpl();

    bool putDownTokenImpl();

public:
    TTTController(std::string name, std::string limb, bool legacy_code = false,
                  bool _use_robot = true, bool use_forces = false);
    ~TTTController();

    /*
     * image callback function that displays the image stream from the hand camera
     *
     * @param      The image
     * @return     N/A
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    bool goHome();

    bool startAction(std::string a, int o = -1);
};

#endif
