#ifndef KEYPAD_DEFINITIONS_H
#define KEYPAD_DEFINITIONS_H

#include <string>
#include "src/utils/T_ThreadSafe.h"

class KeypadDefinitions {
public:
    static const std::string right_arm_ik_service_name;
    static const std::string left_arm_ik_service_name;
    static const std::string topic_command_right_arm;
    static const std::string topic_mode_right_arm;
    static const std::string topic_command_left_arm;
    static const std::string topic_mode_left_arm;
    static const std::string most_distal_link_left_arm;
    static const std::string most_distal_link_right_arm;
    static const std::string topic_left_arm_state;
    static const std::string topic_right_arm_state;
    static const std::string right_group_name;
    static const std::string left_group_name;

    static const std::string HELP;
    static const std::string HELP_COMMANDS;

    static const char UP;
    static const char LEFT;
    static const char RIGHT;
    static const char DOWN;
    static const char ESC;
    static const char FORWARD;
    static const char BACKWARD;

    static const double INCREMENT;
    static const double ALMOST_ZERO;

    static ThreadSafeVariable<bool> arm_stopped;
};

const std::string KeypadDefinitions::right_arm_ik_service_name = "/sdk/robot/limb/right/solve_ik_position";
const std::string KeypadDefinitions::left_arm_ik_service_name = "/sdk/robot/limb/left/solve_ik_position";
const std::string KeypadDefinitions::topic_command_right_arm = "/robot/limb/right/command_joint_angles";
const std::string KeypadDefinitions::topic_mode_right_arm = "/robot/limb/right/joint_command_mode";
const std::string KeypadDefinitions::topic_command_left_arm = "/robot/limb/left/command_joint_angles";
const std::string KeypadDefinitions::topic_mode_left_arm = "/robot/limb/left/joint_command_mode";
const std::string KeypadDefinitions::most_distal_link_left_arm = "left_gripper";
const std::string KeypadDefinitions::most_distal_link_right_arm = "right_gripper";
const std::string KeypadDefinitions::topic_left_arm_state = "/robot/limb/left/joint_states";
const std::string KeypadDefinitions::topic_right_arm_state = "/robot/limb/right/joint_states";
const std::string KeypadDefinitions::right_group_name = "right_arm";
const std::string KeypadDefinitions::left_group_name = "left_arm";

const std::string KeypadDefinitions::HELP="Parameter required: left|right";

const char KeypadDefinitions::UP = 'w';
const char KeypadDefinitions::DOWN = 's';
const char KeypadDefinitions::LEFT = 'a';
const char KeypadDefinitions::RIGHT = 'd';
const char KeypadDefinitions::FORWARD = 'e';
const char KeypadDefinitions::BACKWARD = 'q';
const char KeypadDefinitions::ESC = 27;

const double KeypadDefinitions::INCREMENT = 0.1;
const double KeypadDefinitions::ALMOST_ZERO = 0.02;

#endif // KEYPAD_DEFINITIONS_H
