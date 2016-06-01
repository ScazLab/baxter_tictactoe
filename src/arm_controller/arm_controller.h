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

    std::vector<float> getJointAngles(geometry_msgs::PoseStamped pose_stamped, Limb limb);
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
    void moveLeftToStandby();
    void moveRightToRest();
};

