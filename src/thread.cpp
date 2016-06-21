#include <ros/ros.h>
#include <ros/console.h>
#include <pthread.h>
#include <unistd.h>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;

/*STACK OVERFLOW CODE: ONLY USE FOR TESTING PURPOSES*/
class MyThreadClass
{
public:
   MyThreadClass() {/* empty */}
   virtual ~MyThreadClass() {/* empty */}

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

protected:
   /** Implement this method in your subclass with the code you want your thread to run. */
   virtual void InternalThreadEntry() = 0;

private:
   static void * InternalThreadEntryFunc(void * This) {((MyThreadClass *)This)->InternalThreadEntry(); return NULL;}
   pthread_t _thread;
};

class Test : public MyThreadClass
{

private: 
    ros::NodeHandle _n;
    ros::Subscriber _endpt_sub;
    ros::Publisher _joint_cmd_pub;
    Pose _pose;
    string _limb;

public:
    Test(string limb): _limb(limb)
    {    
        cout << "[constructor] initialization" << endl;
        cout << "[constructor] " << _limb << endl;
        _endpt_sub = _n.subscribe("/robot/limb/" + _limb + "/endpoint_state", 1, &Test::endpointCallback, this);
        _joint_cmd_pub = _n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + _limb + "/joint_command", 1);   
        cout << "[constructor] handling of subscriber/publisher" << endl;
    }

    void endpointCallback(const EndpointState& msg) {_pose = msg.pose; /* cout << "[callback] " << _pose << endl;*/}

    // void endpointCallback2(const EndpointState& msg) {}

    void printCallback() {cout << "[func] " << _pose << endl;}

protected:
    void InternalThreadEntry()
    {
        while(_pose.position.x == 0)
        {
            ros::Rate(100).sleep();
            cout << "[thread] pose: " << _pose << endl;
        }

        PoseStamped _req_pose_stamped;
        
        // req_pose_stamped.pose
        if(true){
           _req_pose_stamped.header.frame_id = "base";
           _req_pose_stamped.pose.position.x = 0.292391;
           _req_pose_stamped.pose.position.y = _limb == "left" ? 0.611039 : -0.611039;
           _req_pose_stamped.pose.position.z = 0.181133;
           _req_pose_stamped.pose.orientation.x = 0.028927;
           _req_pose_stamped.pose.orientation.y = 0.686745;
           _req_pose_stamped.pose.orientation.z = 0.00352694;
           _req_pose_stamped.pose.orientation.w = 0.726314;}

        while(!hasPoseCompleted(_req_pose_stamped.pose))
        {
            ros::Rate loop_rate(500);

            JointCommand joint_cmd;
            joint_cmd.mode = JointCommand::POSITION_MODE;

            // joint_cmd.names
            if(true){
                joint_cmd.names.push_back(_limb + "_s0");
                joint_cmd.names.push_back(_limb + "_s1");
                joint_cmd.names.push_back(_limb + "_e0");
                joint_cmd.names.push_back(_limb + "_e1");
                joint_cmd.names.push_back(_limb + "_w0");
                joint_cmd.names.push_back(_limb + "_w1");
                joint_cmd.names.push_back(_limb + "_w2");}
            joint_cmd.command.resize(7);
            // joint_cmd.angles
            if(true){
                joint_cmd.command[0] = _limb == "left" ? 1.1508690861110316   : -1.3322623142784817;
                joint_cmd.command[1] = _limb == "left" ? -0.6001699832601681  : -0.5786942522297723;
                joint_cmd.command[2] = _limb == "left" ? -0.17449031462196582 : 0.14266021327334347;
                joint_cmd.command[3] = _limb == "left" ? 2.2856313739492666   : 2.2695245756764697 ;
                joint_cmd.command[4] = _limb == "left" ? 1.8680051044474626   : -1.9945585194480093;
                joint_cmd.command[5] = _limb == "left" ? -1.4684031092033123  : -1.469170099597255 ;
                joint_cmd.command[6] = _limb == "left" ? 0.1257864246066039   : -0.011504855909140603;}

            _joint_cmd_pub.publish(joint_cmd);
            loop_rate.sleep();
        }
        cout << "DONE" << endl;

        pthread_exit(NULL);  
    }  

    bool hasPoseCompleted(Pose req_pose)
    {
        // cout << "curr " << _pose << " req " << req_pose << endl;
        bool same_pose = true;
        if(!withinXHundredth(_pose.position.x, req_pose.position.x, 1))       {same_pose = false; /*cout << "[pos x] " << (same_pose == false ? "false" : "true") << endl;*/} 
        if(!withinXHundredth(_pose.position.y, req_pose.position.y, 1))       {same_pose = false; /*cout << "[pos y] " << (same_pose == false ? "false" : "true") << endl;*/} 
        if(!withinXHundredth(_pose.position.z, req_pose.position.z, 2))       {same_pose = false; /*cout << "[pos z] " << (same_pose == false ? "false" : "true") << endl;*/}    
        if(!withinXHundredth(_pose.orientation.x, req_pose.orientation.x, 4)) {same_pose = false; /*cout << "[ori x] " << (same_pose == false ? "false" : "true") << endl;*/}  
        if(!withinXHundredth(_pose.orientation.y, req_pose.orientation.y, 4)) {same_pose = false; /*cout << "[ori y] " << (same_pose == false ? "false" : "true") << endl;*/}  
        if(!withinXHundredth(_pose.orientation.z, req_pose.orientation.z, 4)) {same_pose = false; /*cout << "[ori z] " << (same_pose == false ? "false" : "true") << endl;*/}  
        if(!withinXHundredth(_pose.orientation.w, req_pose.orientation.w, 4)) {same_pose = false; /*cout << "[ori w] " << (same_pose == false ? "false" : "true") << endl;*/}
        return same_pose; 
    }

    bool withinXHundredth(float x, float y, float z)
    {
        float diff = abs(x - y);
        float diffTwoDP = roundf(diff * 100) / 100;
        return diffTwoDP <= (0.01 * z) ? true : false;
    }

    bool equalXDP(float x, float y, float z)
    {
        // ROS_INFO("x: %0.3f y: %0.3f", x, y);
        float xTwoDP = roundf(x * pow(10, z)) / pow(10, z);
        float yTwoDP = roundf(y * pow(10, z)) / pow(10, z);

        // ROS_INFO("xTwoDP: %0.3f yTwoDP: %0.3f", xTwoDP, yTwoDP);
        return xTwoDP == yTwoDP ? true : false;    
    }
};

class ArmController : public MyThreadClass
{

private:
    ros::NodeHandle _n;
    ros::Subscriber _endpt_sub;
    ros::Publisher _joint_cmd_pub;
    Pose _pose;
    string _limb;

public:
    Pose _curr_pose;

    ArmController(string limb): _limb(limb)
    { 
        _endpt_sub = _n.subscribe("/robot/limb/" + _limb + "/endpoint_state", 1, &ArmController::endpointCallback, this);
    }
    virtual ~ArmController(){}

    void endpointCallback(const EndpointState& msg) {_curr_pose = msg.pose; cout << _curr_pose << endl;}

    // bool StartInternalThread()
    // {
    //    return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);
    // }

    // * Will not return until the internal thread has exited. 
    // void WaitForInternalThreadToExit()
    // {
    //    (void) pthread_join(_thread, NULL);
    // }

protected:
    virtual void InternalThreadEntry(){}
};




int main(int argc, char * argv[])
{
    ros::init(argc, argv, "thread");

    Test * left = new Test("left");
    Test * right = new Test("right");

    left->StartInternalThread();
    right->StartInternalThread();

    // ArmController * left_arm_controller = new ArmController("left");
    // ArmController * right_arm_controller = new ArmController("right");

    ros::spin();
    ros::shutdown();
    return 0;
}
