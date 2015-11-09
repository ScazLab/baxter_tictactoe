cmake_minimum_required(VERSION 2.8.3)
project(tictactoe)
find_package(catkin REQUIRED
    genmsg actionlib_msgs actionlib
    )

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#Required for generating the messages related to actionlib
add_action_files(DIRECTORY action FILES PlaceToken.action)
generate_messages(DEPENDENCIES actionlib_msgs)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories($(tictactoe_INCLUDE_DIRS))
include_directories($(PROJECT_SOURCE_DIR)/src/trajectory_player $(PROJECT_SOURCE_DIR)/src/trajectory_xml_parser $(PROJECT_SOURCE_DIR)/src/utils $(PROJECT_SOURCE_DIR)/src/move_maker $(PROJECT_SOURCE_DIR)/src/vacuum_gripper)

find_package(Qt4 COMPONENTS QtCore QtGui)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

catkin_package()

# Targets

# trajectory xml parser
add_library(trajectory_xml_parser src/trajectory_xml_parser/trajectory_xml_parser.cpp src/trajectory_xml_parser/trajectory_xml_parser.h)
target_link_libraries(trajectory_xml_parser ${QT_LIBRARIES})
add_executable(test_trajectory_xml_parser src/trajectory_xml_parser/test_trajectory_xml_parser.cpp)
target_link_libraries(test_trajectory_xml_parser trajectory_xml_parser)
add_executable(test_loading_trajectory_xml src/trajectory_xml_parser/test_loading_trajectory_xml.cpp)
target_link_libraries(test_loading_trajectory_xml trajectory_xml_parser)
# conversor from raw to xml trajectory files
add_executable(trajectory_from_raw_to_xml src/trajectory_xml_parser/trajectory_from_raw_to_xml.cpp)
target_link_libraries(trajectory_from_raw_to_xml trajectory_xml_parser)
# concat several xml trajectory files
add_executable(concat_trajectory_xml_files src/trajectory_xml_parser/concat_trajectory_xml_files.cpp)
target_link_libraries(concat_trajectory_xml_files trajectory_xml_parser)

# vacuum gripper
add_library(vacuum_gripper src/vacuum_gripper/vacuum_gripper.cpp src/vacuum_gripper/vacuum_gripper.h)
add_executable(test_vacuum_gripper src/vacuum_gripper/test_vacuum_gripper.cpp)
target_link_libraries(test_vacuum_gripper vacuum_gripper)

# robot state
add_library(robot_state src/robot_state/robot_state.cpp src/robot_state/robot_state.h)
add_executable(test_robot_state src/robot_state/test_robot_state.cpp)
target_link_libraries(test_robot_state robot_state)

# trajectory player library
add_library(trajectory_player src/trajectory_player/trajectory_player.cpp src/trajectory_player/trajectory_player.h)
target_link_libraries(trajectory_player vacuum_gripper)
add_executable(test_trajectory_player src/trajectory_player/test_trajectory_player.cpp)
target_link_libraries(test_trajectory_player trajectory_player trajectory_xml_parser)

# move_maker
add_library(move_maker src/move_maker/move_maker.cpp src/move_maker/move_maker.h src/utils/my_ros_utils.h src/move_maker/ttt_moves_trajectories.h)
target_link_libraries(move_maker trajectory_player trajectory_xml_parser)
add_executable(move_maker_server src/move_maker/move_maker_server.cpp)
target_link_libraries(move_maker_server move_maker)
add_executable(move_maker_simpleclient src/move_maker/move_maker_simpleclient.cpp)
# Boost thread to move_maker_simpleclient
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(move_maker_simpleclient ${Boost_LIBRARIES}) 

target_link_libraries(move_maker_simpleclient move_maker)

#tests for utils
add_executable(test_thread_safe_variable src/utils/test_thread_safe_variable.cpp src/utils/T_ThreadSafe.h)
add_executable(test_buffer_toggle src/utils/test_buffer_toggle.cpp src/utils/BufferToggle.h)

# ttt brain
include_directories($(ttt_board_sensor_INCLUDE_DIRS))
include_directories($(sound_play_INCLUDE_DIRS))
add_executable(ttt_brain src/brain/ttt_brain.cpp)
target_link_libraries(ttt_brain ttt vacuum_gripper robot_state ${QT_LIBRARIES})
