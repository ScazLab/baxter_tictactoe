#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "right_arm_joint_test_pub");
  ros::NodeHandle n;
  ros::Publisher right_cmd_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
  
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

  std::cout<<cmd<<std::endl;

  while(ros::ok()){
    ROS_INFO("In the loop");
    //update cmd.command commands here
    right_cmd_pub.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
    std::cout<<cmd<<std::endl;
  }
  return 0;
}