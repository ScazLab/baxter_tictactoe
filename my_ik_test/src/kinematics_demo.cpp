#include <ros/ros.h>

#include <Eigen/Geometry>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/joint_state_group.h>

int main(int argc, char **argv)
{

    ROS_INFO("A simple demo of kinematics");

    ros::init(argc, argv, "kinematics_demo", ros::init_options::AnonymousName);

    // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader model_loader("robot_description");
//    ROS_INFO_STREAM("Description:" << model_loader.getRobotDescription());
    robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
    ROS_INFO_STREAM("Model frame:" << kinematic_model->getModelFrame());
    kinematic_model->printModelInfo();

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    ROS_INFO("Default model state:");
    kinematic_state->printStateInfo();
    ROS_ASSERT_MSG(kinematic_state->hasJointStateGroup("left_arm"), "Group name not found");
    robot_state::JointStateGroup* joint_state_group = kinematic_state->getJointStateGroup("left_arm");
    ROS_INFO_STREAM("Active DOF for " << joint_state_group->getName() << " = " << joint_state_group->getVariableCount());

    /* Compute FK for a set of random joint values*/
    joint_state_group->setToRandomValues();
    const Eigen::Affine3d &end_effector_state = joint_state_group->getRobotState()->getLinkState("left_gripper")->getGlobalLinkTransform();
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

    joint_state_group->setToRandomValues();

    /* Get the joint values*/
    std::vector<double> joint_values;
    joint_state_group->getVariableValues(joint_values);
    const std::vector<std::string> &joint_names = joint_state_group->getJointModelGroup()->getJointModelNames();

    /* Compute de IK */
    bool found_ik = joint_state_group->setFromIK(end_effector_state, 5, 0.1);
    if(found_ik)
    {
        joint_state_group->getVariableValues(joint_values);
        //std::vector<std::string> joint_names=joint_state_group->getJointNames();
        for(std::size_t i=0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }

    /* Compute de Jacobian */
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    joint_state_group->getJacobian(joint_state_group->getJointModelGroup()->getLinkModelNames().back(),
                                   reference_point_position,
                                   jacobian);
    ROS_INFO_STREAM("Jacobian: " << jacobian);

    //ros::waitForShutdown();
    ros::shutdown();
}
