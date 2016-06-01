#include "ros/ros.h"
<<<<<<< HEAD
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/PoseStamped.h>
// #include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Point.h"
// #include "geometry_msgs/Quaternion.h"

using namespace baxter_core_msgs;
using namespace geometry_msgs;

int NUM_JOINTS= 7;
=======
#include "baxter_core_msgs/JointCommand.h"
>>>>>>> Handle merge conflict

int main(int argc, char **argv)
{
  ros::init(argc, argv, "right_arm_joint_test_pub");
  ros::NodeHandle n;
  ros::Publisher right_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
<<<<<<< HEAD
  ros::ServiceClient ik_client;

  ik_client = n.serviceClient<SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");

  SolvePositionIK srv;
  PoseStamped poseSt;

  poseSt.header.frame_id = "base";
  poseSt.pose.position.x = 0.473749561242;
  poseSt.pose.position.y = -0.690844692814;
  poseSt.pose.position.z = 0.408007113257;

  poseSt.pose.orientation.x = -0.0574340933229;
  poseSt.pose.orientation.y = 0.721856881714;
  poseSt.pose.orientation.z = 0.14480378245;
  poseSt.pose.orientation.w = 0.674281715483;

  srv.request.pose_stamp.push_back(poseSt);
  srv.request.seed_mode = 0;

  std::vector<float> joint_angles;

  if(ik_client.call(srv))
  {
    ROS_INFO("Service called");
    std::cout << srv.request << std::endl;
    std::cout << srv.response << std::endl;
    joint_angles.resize(NUM_JOINTS);
    for(int i = 0; i < srv.response.joints[0].position.size(); i++)
    {
      joint_angles[i] = srv.response.joints[0].position[i];
    }
  }

  if(joint_angles.size() != NUM_JOINTS){
    ROS_ERROR("Service did not return the same number of joint angles as there are joints");
    return -1;
  }

  for(int i = 0; i < joint_angles.size(); i++){
    ROS_INFO("joint_angles[%d]: %0.6f", i, joint_angles[i]);
  }  

=======
  
>>>>>>> Handle merge conflict
  // publish at at least 5 Hz, or else Baxter switches back to Position mode and holds position
  ros::Rate loop_rate(100);
  baxter_core_msgs::JointCommand cmd;
  
  // command in velocity mode
  cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
  
  // command joints in the order shown in baxter_interface
  cmd.names.push_back("right_s0");
  cmd.names.push_back("right_s1");
  cmd.names.push_back("right_e0");
  cmd.names.push_back("right_e1");
  cmd.names.push_back("right_w0");
  cmd.names.push_back("right_w1");
  cmd.names.push_back("right_w2");
  
  // set your calculated velocities
<<<<<<< HEAD
  cmd.command.resize(NUM_JOINTS);

  // move to untucked position (can also be achieved by running 'rosrun baxter_tools tuck_arms.py -u')
  
  for(int i = 0; i < NUM_JOINTS; i++){
    cmd.command[i] = joint_angles[i];
  }
=======
  cmd.command.resize(cmd.names.size());

  // move to untucked position (can also be achieved by running 'rosrun baxter_tools tuck_arms.py -u')
  cmd.command[0] = 0.08705340971249723;
  cmd.command[1] = -1.0105098440195164;
  cmd.command[2] = 1.1527865620958884;
  cmd.command[3] = 1.945471134235676;
  cmd.command[4] = -0.6645971763513555;
  cmd.command[5] = 1.0208642143377429;
  cmd.command[6] = 0.4962427848809313;
  
  // for(size_t i = 0; i < cmd.names.size(); i++)
  //   cmd.command[i] = 0.0;
>>>>>>> Handle merge conflict

  std::cout<<cmd<<std::endl;

  while(ros::ok()){
    ROS_INFO("In the loop");
    //update cmd.command commands here
    right_cmd_pub.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
    std::cout<<cmd<<std::endl;
  }
<<<<<<< HEAD

=======
>>>>>>> Handle merge conflict
  return 0;
}