#include <ros/ros.h>
#include <ros/console.h>
// Standard libraries
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
// Image-handling libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// ROS message libraries
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>
// baxter_tictactoe libraries
#include "vacuum_gripper/vacuum_gripper.h"
// Threading libraries
#include <pthread.h>

#define START 0
#define REST 1
#define SCAN 2
#define PICK_UP 3
#define PUT_DOWN 4

class Utils 
{
    public: 

        /*
         * checks if end effector has made contact with a token by checking if 
         * the range of the infrared sensor has fallen below the threshold value
         * 
         * param      current range values of the IR sensor, and a string 
         *            (strict/loose) indicating whether to use a high or low
         *            threshold value
         *             
         * return     true if end effector has made contact; false otherwise
         */

        static bool hasCollided(float range, float max_range, float min_range, std::string mode);

        /*
         * checks if the arm has completed its intended move by comparing
         * the requested pose and the current pose
         * 
         * param      requested pose and current pose, and a string (strict/loose)
         *            indicating the desired level of checking accuracy 
         *             
         * return     true if the parameters of the current pose is equal to the 
         *            requested pose; false otherwise 
         */
 
        static bool hasPoseCompleted(geometry_msgs::Pose a, geometry_msgs::Pose b, std::string mode);

        /*
         * checks if two numbers rounded up to 2 decimal points are within 0.0z (z is specified no.) to each other 
         * 
         * param      two floats x and y specifying the numbers to be checked,
         *            and a float z determining the desired accuracy
         *             
         * return     true if they are within 0.0z; false otherwise
         */
 
        static bool withinXHundredth(float x, float y, float z);

        /*
         * checks if two decimal numbers are equal to each other up to z of decimal points
         * 
         * param      two floats x and y, and a float z specifying the desired accuracy
         *             
         * return     true if they are equal up to z decimal points; false otherwise
         */
 
        static bool equalXDP(float x, float y, float z);
 
        /*
         * sets the position of a pose
         * 
         * param      Pose* pose, and three floats indicating the x-y-z coordinates of a position
         *             
         * return     N/A
         */

        static void setPosition(geometry_msgs::Pose * pose, float x, float y, float z);

        /*
         * sets the orientation of a pose
         * 
         * param      Pose* pose, and three floats indicating the x-y-z-w coordinates of an orientation
         *             
         * return     N/A
         */
         
        static void setOrientation(geometry_msgs::Pose * pose, float x, float y, float z, float w);
 
        /*
         * sets the joint names of a JointCommand
         * 
         * param      JointCommand * joint_cmd, and a string (left/right) indicating which arm is
         *            being moved
         *             
         * return     N/A
         */

        static void setNames(baxter_core_msgs::JointCommand * joint_cmd, std::string limb);

        /*
         * converts an integer to a string
         * 
         * param      integer to be converted
         *             
         * return     converted string
         */
 
        static std::string intToString( const int a );
};

// Class initializes overhead functions necessary to start a thread from within a class, 
// and overhead ROS features: subscriber/publishers, services, callback functions etc.
class ROSThreadClass
{
    public:
        ROSThreadClass(std::string limb);
        virtual ~ROSThreadClass();

        /*
         * starts thread that executes the internal thread entry function
         * 
         * param      N/A
         * 
         * return     true if thread was successfully launched; false otherwise
         */        

        bool StartInternalThread();

        /*
         * prevents any following code from being executed before thread is exited
         * 
         * param      N/A
         * 
         * return     true if thread was successfully launched; false otherwise
         */      

        void WaitForInternalThreadToExit();

        /*
         * callback function that sets the current pose to the pose received from 
         * the endpoint state topic
         * 
         * param      N/A
         * 
         * return     N/A
         */

        void endpointCallback(const baxter_core_msgs::EndpointState& msg);

        /*
         * infrared sensor callback function that sets the current range to the range received
         * from the left hand range state topic
         * 
         * param      ImageConstPtr is equal to 'typedef boost::shared_ptr< ::sensor_msgs::Image const>'
         * 
         * return     N/A
         */

        void IRCallback(const sensor_msgs::RangeConstPtr& msg);

        /*
         * image callback function that displays the image stream from the hand camera 
         * 
         * param      ImageConstPtr is equal to 'typedef boost::shared_ptr< ::sensor_msgs::Image const>'
         * 
         * return     N/A
         */

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        /*
         * returns current state 
         * 
         * param      N/A
         * return     N/A
         */

        geometry_msgs::Point getState();

    protected:
        ros::Publisher _joint_cmd_pub;
        cv::Mat _curr_img;
        float _curr_range, _curr_max_range, _curr_min_range;
        geometry_msgs::Pose _curr_pose;
        std::string _limb;
        ttt::Vacuum_Gripper * _gripper;
        ros::Time _init_time;
        geometry_msgs::Point _state; // member 'x' records state change, member 'y' stores time state was changed 

        /*
         * function that will be spun out as a thread
         * 
         * param      N/A
         * 
         * return     N/A
         */

        virtual void InternalThreadEntry() = 0;

        /*
         * moves arm to the requested pose
         * 
         * param      requested PoseStamped, and string (strict/loose) indicating
         *            desired accuracy of pose checking
         * 
         * return     N/A
         */

        void goToPose(geometry_msgs::PoseStamped req_pose_stamped);

        void goToPose(geometry_msgs::PoseStamped req_pose_stamped, std::string mode);

        /*
         * use built in IK solver to find joint angles solution for desired pose
         * 
         * param      requested PoseStamped
         * 
         * return     array of joint angles solution
         */

        std::vector<double> getJointAngles(geometry_msgs::PoseStamped * pose_stamped);

        /*
         * set internal class state
         * 
         * param      integer indicating the new state
         * 
         * return     N/A
         */    

        void setState(int state);

    private:
        static void * InternalThreadEntryFunc(void * This);
        ros::NodeHandle _n;
        ros::Subscriber _endpt_sub;
        ros::Subscriber _ir_sub;
        ros::ServiceClient _ik_client;
        image_transport::ImageTransport _img_trp;
        image_transport::Subscriber _img_sub;
        pthread_t _thread;      
};

class MoveToRestClass : public ROSThreadClass
{
    public:
        MoveToRestClass(std::string limb);
        ~MoveToRestClass();

    protected:

        /*
         * moves arm to rest position
         * 
         * param      N/A
         * return     N/A
         */

        void InternalThreadEntry();
};

class PickUpTokenClass : public ROSThreadClass
{
    public:
        PickUpTokenClass(std::string limb);
        ~PickUpTokenClass();

    protected:

        /*
         * picks up token
         * 
         * param      N/A
         * return     N/A
         */

        void InternalThreadEntry();

    private:
        typedef std::vector<std::vector<cv::Point> > Contours;

        /*
         * move arm downwards and suck token upon collision
         * 
         * param      N/A
         * return     N/A
         */

        void gripToken();

        /*
         * hover arm above tokens
         * 
         * param      string (high/low) indicating requested height of arm
         * return     N/A
         */

        void hoverAboveTokens(std::string height);

        /*
         * check if hand camera detects token 
         * 
         * param      Point representing offset between the arm's x-y coordinates
         *            and the token
         * return     N/A
         */

        void checkForToken(cv::Point2d * offset);

        /*
         * identifies token and calculates offset distance required to move hand camera
         * to token
         * 
         * param      Point representing offset between the arm's x-y coordinates
         *            and the token
         * return     N/A
         */

        void processImage(cv::Point2d * offset);

        /*
         * isolates blue colored object in raw image
         * 
         * param      Mat displaying blue colored objects in raw image
         * return     N/A
         */

        void isolateBlue(cv::Mat * output);

        /*
         * isolates black colored object in raw image
         * 
         * param      Mat displaying black colored objects in raw image
         * return     N/A
         */

        void isolateBlack(cv::Mat * output);

        /*
         * isolates board boundaries from image 
         * 
         * param      input Mat, output Mat displaying board boundaries,
         *            and integer indicating lowest y coordinate of board boundaries
         * return     N/A
         */

        void isolateBoard(cv::Mat input, cv::Mat * output, int * board_y);

        /*
         * isolates token from image
         * 
         * param      input Mat, output Mat displaying token,
         *            and integer indicating lowest y coordinate of board boundaries,
         *            and contours of blue-colored objects in image
         * return     N/A
         */

        void isolateToken(cv::Mat input, int board_y, cv::Mat *output, Contours *contours);

        /*
         * calculates offset distance from arm to token
         * 
         * param      token contours, an integer indicating lowest y coordinate of board boundaries,
         *            and contours of blue-colored objects in image, and an output Mat displaying token
         * return     N/A
         */

        void setOffset(Contours contours, cv::Point2d *offset, cv::Mat *output);
};

class ScanBoardClass : public ROSThreadClass 
{
    public:
        ScanBoardClass(std::string limb);
        ~ScanBoardClass();

        std::vector<geometry_msgs::Point> getOffsets();

    protected:
        void InternalThreadEntry();

    private:
        typedef std::vector<std::vector<cv::Point> > Contours;
        std::vector<geometry_msgs::Point> _offsets;

        /*
         * hover arm above tokens
         * 
         * param      N/A
         * return     N/A
         */

        void hoverAboveTokens();

        /*
         * hover arm above board
         * 
         * param      N/A
         * return     N/A
         */

        void hoverAboveBoard();

        void scan();

        void setDepth(float *dist);

        void processImage(std::string mode, float dist);

        void isolateBlack(cv::Mat * output);

        void isolateBoard(Contours * contours, int * board_area, cv::Mat input, cv::Mat * output);

        /*
         * finds cell with the higher centroid
         * 
         * param      returns true if cell i has a higher centroid than cell j; false otherwise
         * return     N/A
         */

        static bool descendingX(std::vector<cv::Point> i, std::vector<cv::Point> j);

        /*
         * calculates offset distance from arm to each board cell
         * 
         * param      board area, cell contours, output Mat displaying cells, and height
         *            from arm to board surface
         * return     N/A
         */

        void setOffsets(int board_area, Contours contours, cv::Mat * output, float dist);

        bool offsetsReachable();
};

class PutDownTokenClass : public ROSThreadClass
{
    public:
        PutDownTokenClass(std::string limb);     
        ~PutDownTokenClass();

        void setCell(int cell);
        void setOffsets(std::vector<geometry_msgs::Point> offsets);

    protected:
        void InternalThreadEntry();

    private:
        int _cell;
        geometry_msgs::Point _center; 
        std::vector<geometry_msgs::Point> _offsets;

        void hoverAboveCell();

        void hoverAboveBoard();

        void hoverAboveTokens();
};

class ArmController
{
    private:
        std::string _limb;
        MoveToRestClass * _rest_class;
        PickUpTokenClass * _pick_class;
        ScanBoardClass * _scan_class;
        PutDownTokenClass * _put_class; 

    public:
        ArmController(std::string limb);
        ~ArmController();

        /*
         * get most recent state change
         * 
         * param      N/A
         * return     integer indicating arm's latest state
         */

        int getState();

        void moveToRest();

        void pickUpToken();

        void scanBoard();

        void putDownToken(int cell);     
};