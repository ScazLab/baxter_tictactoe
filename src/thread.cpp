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

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace cv;

class Utils 
{

public: 

    static bool hasCollided(float range, float max_range, float min_range)
    {
        if(range <= max_range && range >= min_range && range <= 0.060) return true;
        else return false;
    }

    static bool hasPoseCompleted(Pose a, Pose b)
    {
        bool same_pose = true;
        if(!withinXHundredth(a.position.x, b.position.x, 1))       {same_pose = false; /*cout << "[pos x] " << (same_pose == false ? "false" : "true") << endl;*/} 
        if(!withinXHundredth(a.position.y, b.position.y, 1))       {same_pose = false; /*cout << "[pos y] " << (same_pose == false ? "false" : "true") << endl;*/} 
        if(!withinXHundredth(a.position.z, b.position.z, 2))       {same_pose = false; /*cout << "[pos z] " << (same_pose == false ? "false" : "true") << endl;*/}    
        if(!withinXHundredth(a.orientation.x, b.orientation.x, 4)) {same_pose = false; /*cout << "[ori x] " << (same_pose == false ? "false" : "true") << endl;*/}  
        if(!withinXHundredth(a.orientation.y, b.orientation.y, 4)) {same_pose = false; /*cout << "[ori y] " << (same_pose == false ? "false" : "true") << endl;*/}  
        if(!withinXHundredth(a.orientation.z, b.orientation.z, 4)) {same_pose = false; /*cout << "[ori z] " << (same_pose == false ? "false" : "true") << endl;*/}  
        if(!withinXHundredth(a.orientation.w, b.orientation.w, 4)) {same_pose = false; /*cout << "[ori w] " << (same_pose == false ? "false" : "true") << endl;*/}
        return same_pose; 
    }

    static bool withinXHundredth(float x, float y, float z)
    {
        float diff = abs(x - y);
        float diffTwoDP = roundf(diff * 100) / 100;
        return diffTwoDP <= (0.01 * z) ? true : false;
    }

    static bool equalXDP(float x, float y, float z)
    {
        float xTwoDP = roundf(x * pow(10, z)) / pow(10, z);
        float yTwoDP = roundf(y * pow(10, z)) / pow(10, z);
        return xTwoDP == yTwoDP ? true : false;    
    }

    static void setPosition(Pose * pose, float x, float y, float z)
    {
        (*pose).position.x = x;
        (*pose).position.y = y;
        (*pose).position.z = z;
    }
     
    static void setOrientation(Pose * pose, float x, float y, float z, float w)
    {
        (*pose).orientation.x = x;
        (*pose).orientation.y = y;
        (*pose).orientation.z = z;
        (*pose).orientation.w = w;
    }

    static void setNames(JointCommand * joint_cmd, string limb)
    {
        (*joint_cmd).names.push_back(limb + "_s0");
        (*joint_cmd).names.push_back(limb + "_s1");
        (*joint_cmd).names.push_back(limb + "_e0");
        (*joint_cmd).names.push_back(limb + "_e1");
        (*joint_cmd).names.push_back(limb + "_w0");
        (*joint_cmd).names.push_back(limb + "_w1");
        (*joint_cmd).names.push_back(limb + "_w2");
    }
};

// Class initializes overhead functions necessary to start a thread from within a class, 
// and overhead ROS features: subscriber/publishers, services, callback functions etc.
class ROSThreadClass
{
public:
    ROSThreadClass(string limb): _img_trp(_n), _limb(limb)
    {
        _joint_cmd_pub = _n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + _limb + "/joint_command", 1);   
        _endpt_sub = _n.subscribe("/robot/limb/" + _limb + "/endpoint_state", 1, &ROSThreadClass::endpointCallback, this);
        _img_sub = _img_trp.subscribe("/cameras/left_hand_camera/image", 1, &ROSThreadClass::imageCallback, this);
        _ir_sub = _n.subscribe("/robot/range/left_hand_range/state", 1, &ROSThreadClass::IRCallback, this);
        _ik_client = _n.serviceClient<SolvePositionIK>("/ExternalTools/" + _limb + "/PositionKinematicsNode/IKService");
        _gripper = new ttt::Vacuum_Gripper(ttt::left);
    }
    virtual ~ROSThreadClass() {/* empty */}

    /** Returns true if the thread was successfully started, false if there was an error starting the thread */
    bool StartInternalThread()
    {
       return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);
    }

    /** Will not return until the internal thread has exited. */
    void WaitForInternalThreadToExit()
    {
       (void) pthread_join(_thread, NULL);
    }

    void endpointCallback(const baxter_core_msgs::EndpointState& msg) {_curr_pose = msg.pose;}
    void IRCallback(const sensor_msgs::RangeConstPtr& msg) {_curr_range = msg->range; _curr_max_range = msg->max_range; _curr_min_range = msg->min_range;}
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {cv_ptr = cv_bridge::toCvShare(msg);}
        catch(cv_bridge::Exception& e) {ROS_ERROR("[Arm Controller] cv_bridge exception: %s", e.what());}
        _curr_img = cv_ptr->image.clone();
    }

protected:
    ros::Publisher _joint_cmd_pub;

    cv::Mat _curr_img;
    float _curr_range, _curr_max_range, _curr_min_range;
    geometry_msgs::Pose _curr_pose;

    std::string _limb;
    ttt::Vacuum_Gripper * _gripper;

    virtual void InternalThreadEntry() = 0;

private:
    static void * InternalThreadEntryFunc(void * This) {((ROSThreadClass *)This)->InternalThreadEntry(); return NULL;}
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
    MoveToRestClass(string limb): ROSThreadClass(limb) {}
    ~MoveToRestClass(){}

    void printCallback() {cout << "[func] " << _curr_pose << endl;}

protected:
    void InternalThreadEntry()
    {
        while(_curr_pose.position.x == 0 && _curr_pose.position.y == 0 && _curr_pose.position.z == 0)
        {
            ros::Rate(100).sleep();
        }

        PoseStamped _req_pose_stamped;
        
       _req_pose_stamped.header.frame_id = "base";
       Utils::setPosition(   &_req_pose_stamped.pose, 0.292391, _limb == "left" ? 0.611039 : -0.611039, 0.181133);
       Utils::setOrientation(&_req_pose_stamped.pose, 0.028927, 0.686745, 0.00352694, 0.726314);

        while(!Utils::hasPoseCompleted(_curr_pose, _req_pose_stamped.pose))
        {
            ros::Rate loop_rate(500);

            JointCommand joint_cmd;
            joint_cmd.mode = JointCommand::POSITION_MODE;

            // joint_cmd.names
            Utils::setNames(&joint_cmd, _limb);
            joint_cmd.command.resize(7);
            // joint_cmd.angles
            joint_cmd.command[0] = _limb == "left" ? 1.1508690861110316   : -1.3322623142784817;
            joint_cmd.command[1] = _limb == "left" ? -0.6001699832601681  : -0.5786942522297723;
            joint_cmd.command[2] = _limb == "left" ? -0.17449031462196582 : 0.14266021327334347;
            joint_cmd.command[3] = _limb == "left" ? 2.2856313739492666   : 2.2695245756764697 ;
            joint_cmd.command[4] = _limb == "left" ? 1.8680051044474626   : -1.9945585194480093;
            joint_cmd.command[5] = _limb == "left" ? -1.4684031092033123  : -1.469170099597255 ;
            joint_cmd.command[6] = _limb == "left" ? 0.1257864246066039   : -0.011504855909140603;

            _joint_cmd_pub.publish(joint_cmd);
            loop_rate.sleep();
        }
        cout << "DONE" << endl;

        pthread_exit(NULL);  
    }  
};

class PickUpTokenClass : public ROSThreadClass
{
public:
    PickUpTokenClass(string limb, int cell_num): ROSThreadClass(limb), _cell_num(cell_num)
    {
        namedWindow("[PickUpToken]", WINDOW_NORMAL);
    }

    ~PickUpTokenClass() {destroyWindow("[PickUpToken]");}

protected:
    void InternalThreadEntry()
    {
        while((_curr_range == 0 && _curr_min_range == 0 && _curr_max_range == 0) || _curr_img.empty())
        {
            ros::Rate(100).sleep();
        }

        while(!Utils::hasCollided(_curr_range, _curr_min_range, _curr_max_range))
        {
            cv::Point offset = processImage();
        }
    }

private:
    int _cell_num;

    void isolateBlue(Mat * output)
    {
        Mat hsv;
        cvtColor(_curr_img, hsv, CV_BGR2HSV);
        inRange(hsv, Scalar(60,90,10), Scalar(130,256,256), *output);  
    }

    void isolateTokenContours(Mat input, Mat *output)
    {
        vector<vector<cv::Point> > contours, token_contours;
        findContours(input, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        int largest_index = 0, largest_area = 0;
        for(int i = 0; i < contours.size(); i++)
        {
            bool not_gripper = true;
            for(int j = 0; j < contours[i].size(); j++)
            {
                vector<cv::Point> contour = contours[i];
                if(contour[j].y < 65) {not_gripper = false; break;}
            }

            bool is_triangle = true;
            vector<cv::Point> contour;
            approxPolyDP(contours[i], contour, 0.11 * arcLength(contours[i], true), true);

            if(contour.size() != 3) is_triangle = false;

            if(contourArea(contours[i]) > 200 && not_gripper == true && is_triangle == true)
            {
                token_contours.push_back(contours[i]);
            }
        }

        *output = Mat::zeros(_curr_img.size(), CV_8UC1);
        // draw 'blue triangles' portion of token
        for(int i = 0; i < token_contours.size(); i++)
        {
            drawContours(*output, token_contours, i, Scalar(255,255,255), CV_FILLED);
        }

        circle(*output, cv::Point((_curr_img.size().width / 2) + 45, 65), 3, Scalar(180, 40, 40), CV_FILLED);
    }

    cv::Point processImage()
    {
        Mat blue, token_rough, token; 

        isolateBlue(&blue);
        isolateTokenContours(blue.clone(), &token_rough);




        // // if 'noise' contours are present, do nothing
        // if(token_contours.size() == 4)
        // {
        //     // find highest and lowest x and y values from token triangles contours
        //     // to find x-y coordinate of top left token edge and token side length
        //     float y_min = getTokenPoints(token_contours, "y_min");
        //     float x_min = getTokenPoints(token_contours, "x_min");
        //     float y_max = getTokenPoints(token_contours, "y_max");
        //     float x_max = getTokenPoints(token_contours, "x_max");

        //     // reconstruct token's square shape
        //     Rect token(x_min, y_min, y_max - y_min, y_max - y_min);
        //     rectangle(img_token, token, Scalar(255,255,255), CV_FILLED);

        //     // find and draw the center of the token and the image
        //     double x_mid = x_min + ((x_max - x_min) / 2);
        //     double y_mid = y_min + ((y_max - y_min) / 2);
        //     circle(img_token, cv::Point(x_mid, y_mid), 3, Scalar(0, 0, 0), CV_FILLED);
        //     circle(img_token, cv::Point(img_token.size().width / 2, img_token.size().height / 2), 3, Scalar(180, 40, 40), CV_FILLED);

        //     double token_area = (x_max - x_min) * (y_max - y_min);
        //     _offset_token.x = (4.7807 /*constant*/ / token_area) * (x_mid - (img_token.size().width / 2));
        //     _offset_token.y = ((4.7807 /*constant*/ / token_area) * ((img_token.size().height / 2) - y_mid)) - 0.013; /*distance between gripper center and camera center*/
        // }
        // // when hand camera is blind due to being too close to token, go straight down;
        // else
        // {
        //     _offset_token.x = 0;
        //     _offset_token.y = 0;
        // }

        imshow("[PickUpToken]", token_rough);
        waitKey(30);
    }

};





class ArmController
{

private:
    std::string _limb;

public:
    ArmController(string limb): _limb(limb){}
    ~ArmController(){}

    void moveToRest() 
    {
        MoveToRestClass * rest_class = new MoveToRestClass(_limb);
        rest_class->StartInternalThread();
    }

    void pickUpToken(int cell_num) {
        PickUpTokenClass * pick_up_class = new PickUpTokenClass(_limb, cell_num);
        pick_up_class->StartInternalThread();
    }

    // void putDownToken() {PutDownToken * rest_class = new PutDownToken(limb);}
};


/* Main */

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "thread");

    ArmController * left_ac = new ArmController("left");
    // ArmController * right_ac = new ArmController("right");
    // left_ac->moveToRest();
    left_ac->pickUpToken(5);
    // right_ac->moveToRest();

    ros::spin();
    ros::shutdown();
    return 0;
}
