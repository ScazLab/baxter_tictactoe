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

#include "ros/ros.h"
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/PoseStamped.h>

enum Limb {RIGHT, LEFT};

class ArmController
{

private:
    ros::NodeHandle n;
    // [NOTE] ros::Publisher name should convey the type of message it is publishing
    ros::Publisher r_joint_cmd_pub;
    ros::Publisher l_joint_cmd_pub;
    // [NOTE] ros::ServiceClient name should convey the type of service it is requesting
    ros::ServiceClient r_ik_client;
    ros::ServiceClient l_ik_client;

    int NUM_JOINTS;

    /*
     * takes in a position and orientation that the user wants the right/left arm to be in,
     * and uses Baxter's built-in inverse kinematics solver to produce the joint angles
     * at which the right/left joints must have to reach that position and orientation
     * 
     * @param      a PoseStamped specifying the desired end position and orientation, and
     *             the limb the user wants to move
     *             
     * @return     a vector of joint angles the right/left arm joints should have to reach the desired
     *             position/orientation, in order from shoulder to wrist
     */

    std::vector<float> getJointAngles(geometry_msgs::PoseStamped pose_stamped, Limb limb);

    /*
     * takes in an array of joint angles and commands the right/left joints to take on the
     * angles specified, moving the arm's position/orientation in the process
     * 
     * @param      a vector of joint angles, in order from shoulder to wrist, and 
     *             the limb the user wants to move
     *             
     * @return     N/A
     */

    void publishMoveCommand(std::vector<float> joint_angles, Limb limb);
    bool reachedPose();

public:

    ArmController();
    ~ArmController();

    // [NOTE] Style note on naming conventions of functions: OpenCV functions follow
    // a word1Word2 convention as opposed to word1_word2. Given that this is the case,
    // it is better to standardize baxter_tictactoe functions to use the same naming
    // convention.

    // [NOTE] Style note on naming conventions of variables: Given the above, variables
    // should follow a word1_word2 convention for maximum contrast

    void pickUpTile();
    void placeTile();

    /*
     * moves the left arm to a standby position it adopts before and after each of 
     * Baxter's turn
     * 
     * @param      N/A
     * 
     * @return     N/A
     */
    void moveLeftToStandby();

    /*
     * moves the left arm to a standby position it adopts before and after each of 
     * Baxter's turn
     * 
     * @param      N/A
     * 
     * @return     N/A
     */
    void moveRightToRest();
};

