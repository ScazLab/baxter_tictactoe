/*
 * ArmController
 * ------------------------------
 * 
 * ArmController handles all four types of arm movements during the game:
 * 1) the right arm moving away from the camera's point of view
 * to avoid blocking the board detection nodes, 
 * 2) the left arm moving to a standby position between each turn,
 * 3) the left arm moving to pick up a token, and 
 * 4) the left arm moving to place a token on a board cell
 * 
 */ 

#include <ros/ros.h>
#include <ros/console.h>
// Standard libraries
#include <string>
#include <iostream>
#include <cmath>
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
#include <sensor_msgs/Range.h>
// baxter_tictactoe libraries
#include "vacuum_gripper/vacuum_gripper.h"

enum GoalType {LOOSEPOSE, STRICTPOSE, COLLISION};
enum PoseType {LOOSE, STRICT};

class ArmController
{

private:
    ros::NodeHandle n;

    // publishes joint angles commands in order to move arm
    ros::Publisher joint_cmd_pub;
    // subscribes to end-effector endpoint in order to find current pose
    ros::Subscriber endpt_sub;
    // subscribes to left hand range in order to find current range
    ros::Subscriber ir_sub;
    // requests inverse kinematics service to find joint angles to reach desired pose
    ros::ServiceClient ik_client;

    // 'node handle' for images (e.g img_sub = img_trp.subscribe(...) instead of img_sub = n.subscribe(...))
    image_transport::ImageTransport img_trp;
    // subscribes to hand camera image stream in order to locate tile
    image_transport::Subscriber img_sub;

    // PoseStamped message to be used as request value for IK solver service
    geometry_msgs::PoseStamped req_pose_stamped;
    // Pose message used to store current pose; updated by endpointCallback()
    geometry_msgs::Pose curr_pose;

    float curr_range;
    float curr_max_range;
    float curr_min_range;

    float _curr_x_offset;
    float _curr_y_offset;

    ttt::Vacuum_Gripper * gripper;

    // string indicating whether class instance is meant to control right/left limb
    std::string limb;
    double OFFSET_CONSTANT;
    int NUM_JOINTS;
    double CENTER_X;
    double CENTER_Y;
    double CELL_SIDE;
    float IR_RANGE_THRESHOLD;


    /*************************Movement Functions************************/

    /*
     * hover the left arm above the stack of tokens
     * 
     * param      N/A
     *             
     * return     N/A
     */

    void hoverAboveTokens(GoalType goal);

    /*
     * lower left arm and grip a token
     * 
     * param      N/A
     *             
     * return     true if arm succesfully gripped token; false otherwise
     */

    bool gripToken();

    /*
     * hover the left arm above the center of the board
     * 
     * param      N/A
     *             
     * return     N/A
     */

    void hoverAboveBoard();
    void gripTest(float z_offset);

    /*
     * release token in the specified cell
     * 
     * param      an integer specifying which cell the token should
     *             be placed in
     *             
     * return     N/A
     */
       
    void releaseToken(int cell_num);


    /*************************Location Control Functions************************/

    /*
     * takes in a position and orientation that the user wants the arm to be in,
     * and uses Baxter's built-in inverse kinematics solver to produce the joint angles
     * at which the right/left joints must have to reach that position and orientation
     * 
     * param      a PoseStamped specifying the desired end position and orientation,
     *             
     * return     a vector of joint angles the right/left arm joints should have to reach the desired
     *             position/orientation, in order from shoulder to wrist
     */

    std::vector<float> getJointAngles(geometry_msgs::PoseStamped pose_stamped);

    /*
     * takes in an array of joint angles and commands the joints to take on the
     * angles specified, moving the arm's position/orientation in the process
     * 
     * param      a vector of joint angles, in order from shoulder to wrist
     *             
     * return     true if goal is achieved within 10 seconds; false otherwise
     */

    bool publishMoveCommand(std::vector<float> joint_angles, GoalType goal);


    /*************************Checking Functions************************/

    bool checkForTimeout(int len, GoalType goal, ros::Time start_time);

    /*
     * checks if the arm has completed its intended move by comparing
     * the requested pose and the current pose
     * 
     * param      N/A
     *             
     * return     true if the parameters of the current pose is equal to the 
     *             requested pose; false otherwise 
     */

    bool hasPoseCompleted(PoseType pose);

    /*
     * checks if end effector has made contact with a token by checking if 
     * the range of the infrared sensor has fallen below the threshold value
     * 
     * param      N/A
     *             
     * return     true if end effector has made contact; false otherwise
     */

    bool hasCollided();

    /*
     * checks if two numbers rounded up to 2 decimal poitns are within 0.0z (z is specified no.) to each other 
     * 
     * param      two floats x and y specifying the numbers to be checked,
     *            and a float z determining the desired accuracy
     *             
     * return     true if they are within 0.01; false otherwise
     */

    bool withinXHundredth(float x, float y, float z);

    /*
     * checks if two decimal numbers are equal to each other up to z of decimal points
     * 
     * param      two floats x and y, and a float z specifying the desired accuracy
     *             
     * return     true if they are equal up to 2 decimal points; false otherwise
     */

    bool equalXDP(float x, float y, float z);

    /*************************Visual Servoing Functions************************/

    std::vector<std::vector<cv::Point> > getTokenContours(cv::Mat img_hsv_blue);

    float getTokenPoints(std::vector<std::vector<cv::Point> > token_contours, std::string point);


public:

    ArmController(std::string limb);
    ~ArmController();


    /*************************Callback Functions************************/

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


    /*************************Movement Functions************************/

    /*
     * move to position above tokens and pick up tokens
     * 
     * param      N/A
     * 
     * return     N/A
     */

    void pickUpToken();

    /*
     * move arm to position above specified cell and place token 
     * 
     * param      an integer specifying which cell the token should
     *             be placed in
     * 
     * return     N/A
     */

    void placeToken(int cell_num);

    /*
     * moves the arm to a rest position when not performing a move
     * 
     * param      N/A
     * 
     * return     N/A
     */

    void moveToRest();

};