#include <ros/ros.h>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/exceptions.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <baxter_msgs/SolvePositionIK.h>
#include <baxter_msgs/JointCommandMode.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h> // Eigen <-> Pose
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/move_group_interface/move_group.h>

#include "keypad_definitions.h"
#include "src/utils/T_ThreadSafe.h"
#include "src/utils/BufferToggle.h"

#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <math.h>



class Keypad
{

private:
    std::string _service_name;
    std::string _topic_command_arm;
    std::string _topic_mode_arm;
    std::string _group_name;
    std::string _most_distal_link;
    std::string _topic_state_arm;

    ros::NodeHandle _n;
    ros::ServiceClient _client;
    ros::Publisher _commander;
    ros::Publisher _mode;
    //ros::Subscriber _sub_joints_states;
    //_sub_joints_states = _n.subscribe(_topic_state_arm, 1, &Keypad::joint_states_cb, (Keypad*)this);
    //_sub_joints_states.shutdown(); //unsubscribe

    robot_model_loader::RobotModelLoader * _model_loader;
    robot_model::RobotModelPtr _kinematic_model;
    robot_state::RobotState* _kinematic_state;
    robot_state::JointStateGroup* _joint_state_group;

    // this connecs to a running instance of the move_group node
    moveit::planning_interface::MoveGroup* _group;

    std::vector<double> _joint_value;

    BufferToggle bt; // it changes the configuration of the terminal so we don't have to press enter after each key

    void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg)
    {
        for (size_t i = 0; i < msg->velocity.size(); ++i) {
            if(fabs(msg->velocity[i])>KeypadDefinitions::ALMOST_ZERO)
            {
                ROS_DEBUG_STREAM("Joint " << msg->name[i] << " is moving at " << msg->velocity[i]);
                return;
            }
        }
    }

public:

    Keypad(std::string arm)
    {
        if(arm=="right")
        {
            _service_name = KeypadDefinitions::right_arm_ik_service_name;
            _topic_command_arm = KeypadDefinitions::topic_command_right_arm;
            _topic_mode_arm = KeypadDefinitions::topic_mode_right_arm;
            _group_name = KeypadDefinitions::right_group_name;
            _most_distal_link = KeypadDefinitions::most_distal_link_right_arm;
            _topic_state_arm = KeypadDefinitions::topic_right_arm_state;
        }
        else if (arm=="left")
        {
            _service_name = KeypadDefinitions::left_arm_ik_service_name;
            _topic_command_arm = KeypadDefinitions::topic_command_left_arm;
            _topic_mode_arm = KeypadDefinitions::topic_mode_left_arm;
            _group_name = KeypadDefinitions::left_group_name;
            _most_distal_link = KeypadDefinitions::most_distal_link_left_arm;
            _topic_state_arm = KeypadDefinitions::topic_left_arm_state;
        }
        else //other thing
            ROS_ASSERT_MSG(0,"Wrong named arm");

        // Used ROS topics and services
        try {
            _client = _n.serviceClient<baxter_msgs::SolvePositionIK>(_service_name);
            _commander = _n.advertise<baxter_msgs::JointPositions>(_topic_command_arm,1);
            ROS_ASSERT_MSG(_commander,"Empty commander publisher");
            _mode = _n.advertise<baxter_msgs::JointCommandMode>(_topic_mode_arm,1);
            ROS_ASSERT_MSG(_commander,"Empty mode publisher");
        } catch (ros::InvalidNameException e) {
            ROS_FATAL_STREAM("Invalid name exception.");
        }

        // Kinematics initializationrobot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        _model_loader = new robot_model_loader::RobotModelLoader("robot_description");
        _kinematic_model = _model_loader->getModel();
        ROS_INFO("Model frame: %s", _kinematic_model->getModelFrame().c_str());
        _kinematic_state = new robot_state::RobotState(_kinematic_model);
        _kinematic_state->setToDefaultValues();
        _joint_state_group = _kinematic_state->getJointStateGroup(_group_name);

        _group = new move_group_interface::MoveGroup(_group_name);

        // getting initial joint values
        //_joint_value = this->get_joint_values();

        bt.off();
    }

    ~Keypad()
    {
        bt.on();
    }

    std::vector<double> get_joint_values()
    {
        std::vector<double> joint_values;
        _joint_state_group->getVariableValues(joint_values);
        return joint_values;
    }

    static bool is_selected_right_arm(int arg_c, char **arg_v)
    {
        ROS_DEBUG_STREAM("@is_selected_right_arm. " << arg_c << " params in command line");


        ROS_ASSERT_MSG(arg_c>1,"%s",KeypadDefinitions::HELP.c_str());
        for (int i=0; i < arg_c; ++i) {
            //ROS_DEBUG_STREAM("Param " << i << ": " << argv[i]);
            if(!strcmp(arg_v[1],"right")) return true;
            if(!strcmp(arg_v[1],"left")) return false;
        }
        ROS_ASSERT_MSG(0,"%s",KeypadDefinitions::HELP.c_str()); //right|left not found
        return false; //just to avoid painfull compiling messages
    }

    geometry_msgs::Pose get_current_pose()
    {
        ROS_DEBUG_STREAM("@get_current_pose " << __FILE__ << ", " << __LINE__ << ", " << __DATE__ << ", " << __TIME__ << ", ");
        // getting initial position of the end effector (Forward Kinematics)
        const Eigen::Affine3d &end_effector_state = _joint_state_group->getRobotState()->getLinkState(_most_distal_link)->getGlobalLinkTransform();
        geometry_msgs::Pose p;
        tf::poseEigenToMsg(end_effector_state,p);
        ROS_DEBUG_STREAM("Current pose: \n" << p);
        return p;
    }

    void run()
    {
        char c;
        ros::Rate r(10); // 10 hz
        while (ros::ok())
        {
            ROS_DEBUG("Enter UP, DOWN, RIGHT, LEFT, PAGE_UP, or PAGE_DOWN");
            c = std::cin.get();
            if (std::cin.good())
            {
                ROS_INFO("Read character = %d",c);
                if (c==KeypadDefinitions::ESC) //ESC key
                    break;
                else
                {
                    geometry_msgs::Pose next_pose = this->get_current_pose();
                    if (c==KeypadDefinitions::UP)
                    {
                        ROS_DEBUG("UP");
                        next_pose.position.z+=KeypadDefinitions::INCREMENT;
                    }
                    else if (c==KeypadDefinitions::DOWN)
                    {
                        ROS_DEBUG("DOWN");
                        next_pose.position.z-=KeypadDefinitions::INCREMENT;
                    }
                    else if (c==KeypadDefinitions::RIGHT)
                    {
                        ROS_DEBUG("RIGHT");
                        next_pose.position.y+=KeypadDefinitions::INCREMENT;
                    }
                    else if (c==KeypadDefinitions::LEFT)
                    {
                        ROS_DEBUG("LEFT");
                        next_pose.position.y-=KeypadDefinitions::INCREMENT;
                    }
                    else if (c==KeypadDefinitions::FORWARD)
                    {
                        ROS_DEBUG("PG_UP");
                        next_pose.position.x+=KeypadDefinitions::INCREMENT;
                    }
                    else if (c==KeypadDefinitions::BACKWARD)
                    {
                        ROS_DEBUG("PG_DN");
                        next_pose.position.x-=KeypadDefinitions::INCREMENT;
                    }
                    else if (c=='?')
                    {
                        ROS_DEBUG("?");
                        ROS_INFO_STREAM("\nUP " << KeypadDefinitions::UP << "\nDOWN " << KeypadDefinitions::DOWN << "\nRIGHT " << KeypadDefinitions::RIGHT << "\nLEFT " << KeypadDefinitions::LEFT<< "\nFORWARD " << KeypadDefinitions::FORWARD << "\nBACKWARD " << KeypadDefinitions::BACKWARD);
                    }

                    ROS_INFO_STREAM("Moving to new target pose: \n" << next_pose);
                    _group->setPoseTarget(next_pose);
                    _group->move();
                    ROS_INFO_STREAM("Move Done!");

                }
            }
            else ROS_WARN("Input from keyboard is not good");
            r.sleep();
        }
    }


    baxter_msgs::JointPositions solve_position_ik(const geometry_msgs::Pose& p)
    {
        ROS_DEBUG("@solve_position_ik");
        baxter_msgs::JointPositions jp;

        geometry_msgs::PoseStamped ps;
        ps.header.stamp=ros::Time::now();
        ps.header.frame_id="base";
        ps.pose=p;

        baxter_msgs::SolvePositionIK pik;
        pik.request.pose_stamp.push_back(ps);
        if(_client.call(pik)){ //call to the IK service
            if (pik.response.joints.size()<1) ROS_WARN("No IK service response");
            else
            {
                ROS_DEBUG_STREAM("response.joints.size=" << pik.response.joints.size());
                for (size_t i = 0; i < pik.response.joints.size(); ++i)
                {
                    if(pik.response.isValid[i])
                    {
                        //ROS_DEBUG_STREAM("Joints in response " << i);
                        //for (size_t var = 0; var < pik.response.joints[i].angles.size(); ++var) {
                        //    ROS_DEBUG_STREAM("Joint " << pik.response.joints[i].names[var] << ": angle " << pik.response.joints[i].angles[var]);
                        //}
                        jp.angles.reserve(pik.response.joints[i].angles.size());
                        std::copy(pik.response.joints[i].angles.begin(),pik.response.joints[i].angles.end(),std::back_inserter(jp.angles));
                        jp.names.reserve(pik.response.joints[i].names.size());
                        std::copy(pik.response.joints[i].names.begin(),pik.response.joints[i].names.end(),std::back_inserter(jp.names));
                        break; // we always use the first valid response
                    }
                    else ROS_WARN_STREAM("Response " << i << " not valid!");
                }
            }
        }
        else {
            ROS_ERROR_STREAM("Erros calling to the service " << _service_name);
            throw ros::Exception("Erros calling to the service " + _service_name);
        }
        return jp;
    }

    void move_to_init_pose()
    {
        ROS_DEBUG("@move_to_init_pose");
        geometry_msgs::Pose init_pose;
        init_pose.position.x=0.5;
        init_pose.position.z=0.3;
        init_pose.orientation.x=1;
        init_pose.orientation.y=0;
        init_pose.orientation.z=0;
        init_pose.orientation.w=0;
        if (_group_name==KeypadDefinitions::right_group_name)
        {
            init_pose.position.y=-0.5;
        }
        else if(_group_name==KeypadDefinitions::left_group_name)
        {
            init_pose.position.y=0.5;
        }
        ROS_DEBUG_STREAM("Moving to init pose: \n" << init_pose);
        _group->setPoseTarget(init_pose);
        _group->move();
        ROS_DEBUG_STREAM("Move Done!");
    }
};


int main(int argc, char **argv)
{
    ROS_INFO("A simple keypad for commanding Baxter's arms");

    bool is_right_arm=Keypad::is_selected_right_arm(argc,argv);

    ros::init(argc, argv, "baxter_keypad", ros::init_options::AnonymousName);
    // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_DEBUG("Creating the keypad");
    Keypad * kp;
    if(is_right_arm) kp = new Keypad("right");
    else kp = new Keypad("left");
    kp->move_to_init_pose();

    ROS_DEBUG("Running the keypad");
    kp->run(); //this function should never return

    ros::shutdown();
}
