#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (size_t i = 0; i < msg->velocity.size(); ++i) {
        ROS_DEBUG_STREAM("Joint " << msg->name[i] << " is moving at " << msg->velocity[i]);
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener_joint_states");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/robot/limb/right/joint_states", 1, joint_states_cb);

    ros::spin();

    return 0;
}
