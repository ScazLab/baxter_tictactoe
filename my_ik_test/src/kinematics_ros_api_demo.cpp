#include <ros/ros.h>
#include <moveit_msgs/GetPositionIK.h>

int main(int argc, char **argv)
{

    ROS_INFO("A simple demo of ros api for kinematics");

    ros::init(argc, argv, "kinematics_ros_api_demo", ros::init_options::AnonymousName);

    // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Start a service client
    ros::NodeHandle node_handle;
    ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
    while(!service_client.exists())
    {
      ROS_INFO("Waiting for service");
      sleep(1.0);
    }

    // Fill in the request message
    moveit_msgs::GetPositionIK::Request service_request;
    moveit_msgs::GetPositionIK::Response service_response;
    service_request.ik_request.group_name = "left_arm";
//    service_request.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
    service_request.ik_request.pose_stamped.pose.position.x = 0.5;
    service_request.ik_request.pose_stamped.pose.position.y = 0.5;
    service_request.ik_request.pose_stamped.pose.position.z = 0.5;
    service_request.ik_request.pose_stamped.pose.orientation.x = 0.0;
    service_request.ik_request.pose_stamped.pose.orientation.y = 0.0;
    service_request.ik_request.pose_stamped.pose.orientation.z = 0.0;
    service_request.ik_request.pose_stamped.pose.orientation.w = 1.0;
    service_request.ik_request.avoid_collisions = true;

    // Call the service
    service_client.call(service_request, service_response);

    ROS_INFO_STREAM ("Response error code = " << service_response.error_code);
    ROS_INFO_STREAM ("Response solution = " << service_response.solution);

    //ros::waitForShutdown();
    ros::shutdown();
}
