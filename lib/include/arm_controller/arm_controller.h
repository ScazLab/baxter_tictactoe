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

        static bool hasCollided(float range, float max_range, float min_range, std::string mode);
 
        static bool hasPoseCompleted(geometry_msgs::Pose a, geometry_msgs::Pose b, std::string mode);
 
        static bool withinXHundredth(float x, float y, float z);
 
        static bool equalXDP(float x, float y, float z);
 
        static void setPosition(geometry_msgs::Pose * pose, float x, float y, float z);
         
        static void setOrientation(geometry_msgs::Pose * pose, float x, float y, float z, float w);
 
        static void setNames(baxter_core_msgs::JointCommand * joint_cmd, std::string limb);
 
        static std::string intToString( const int a );
};

// Class initializes overhead functions necessary to start a thread from within a class, 
// and overhead ROS features: subscriber/publishers, services, callback functions etc.
class ROSThreadClass
{
    public:
        ROSThreadClass(std::string limb);
        virtual ~ROSThreadClass();

        /** Returns true if the thread was successfully started, false if there was an error starting the thread */
        bool StartInternalThread();

        /** Will not return until the internal thread has exited. */
        void WaitForInternalThreadToExit();

        void endpointCallback(const baxter_core_msgs::EndpointState& msg);

        void IRCallback(const sensor_msgs::RangeConstPtr& msg);

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

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

        virtual void InternalThreadEntry() = 0;

        void goToPose(geometry_msgs::PoseStamped req_pose_stamped);

        void goToPose(geometry_msgs::PoseStamped req_pose_stamped, std::string mode);

        std::vector<double> getJointAngles(geometry_msgs::PoseStamped * pose_stamped);

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
        void InternalThreadEntry();
};

class PickUpTokenClass : public ROSThreadClass
{
    public:
        PickUpTokenClass(std::string limb);
        ~PickUpTokenClass();

    protected:
        void InternalThreadEntry();

    private:
        typedef std::vector<std::vector<cv::Point> > Contours;

        void gripToken();

        void hoverAboveTokens();

        void checkForToken(cv::Point2d * offset);

        void processImage(cv::Point2d * offset);

        void isolateBlue(cv::Mat * output);

        void isolateBlack(cv::Mat * output);

        void isolateBoard(cv::Mat input, cv::Mat * output, int * board_y);

        void isolateToken(cv::Mat input, int board_y, cv::Mat *output, Contours *contours);

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
        std::vector<cv::Point> _centroids;
        std::vector<float> _center_to_cell;

        void hoverAboveTokens();

        void hoverAboveBoard();

        void scan();

        void setDepth(float *dist);

        void processImage(std::string mode, float dist);

        void isolateBlack(cv::Mat * output);

        void isolateBoard(Contours * contours, int * board_area, cv::Mat input, cv::Mat * output);

        static bool descendingX(std::vector<cv::Point> i, std::vector<cv::Point> j);

        void setOffsets(int board_area, Contours contours, cv::Mat * output, float dist, int j);
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

        int getState();

        void moveToRest();

        void pickUpToken();

        void scanBoard();

        void putDownToken(int cell);     
};