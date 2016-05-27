# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Álvaro Castro González, Olivier Mangin, Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(baxterTictactoe)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

## Declare a C++ library
add_library(${PROJECT_NAME}       include/${PROJECT_NAME}/tictactoe_utils.h                     src/tictactoe_utils.cpp)
add_library(trajectory_xml_parser include/trajectory_xml_parser/trajectory_xml_parser.h         src/trajectory_xml_parser.cpp)
add_library(vacuum_gripper        include/vacuum_gripper/vacuum_gripper.h                       src/vacuum_gripper.cpp)
add_library(robot_state           include/robot_state/robot_state.h                             src/robot_state.cpp)
add_library(trajectory_player     include/trajectory_player/trajectory_player.h                 src/trajectory_player.cpp)
add_library(move_maker            include/move_maker/move_maker.h                               src/move_maker.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}   baxter_tictactoe_generate_messages_cpp       ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(move_maker        ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}        ${QT_LIBRARIES}          ${OpenCV_LIBRARIES}    ${catkin_LIBRARIES})
target_link_libraries(trajectory_xml_parser  ${QT_LIBRARIES}               ${catkin_LIBRARIES})
target_link_libraries(trajectory_player      vacuum_gripper ${catkin_LIBRARIES})
target_link_libraries(move_maker             trajectory_player             trajectory_xml_parser)

## Mark libraries for installation
install (TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY lib/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Specify the source files
# set(folder_source_baxter_tictactoe src/board_sensor/board_motion_detection.cpp
#                                    src/board_sensor/board_scheme.cpp
#                                    src/board_sensor/board_state_sensing.cpp
#                                    src/board_sensor/hsv_range_finder.cpp
#                                    src/board_sensor/show_board_cells.cpp
#                                    src/board_sensor/tictactoe_utils.cpp
#                                    src/brain/ttt_brain.cpp
#                                    src/move_maker/move_maker.cpp
#                                    src/move_maker/move_maker_server.cpp
#                                    src/move_maker/move_maker_simpleclient.cpp
#                                    src/robot_state/robot_state.cpp
#                                    src/robot_state/test_robot_state.cpp 
#                                    src/trajectory_player/trajectory_player.cpp 
#                                    src/trajectory_player/test_trajectory_player.cpp 
#                                    src/trajectory_xml_parser/trajectory_xml_parser.cpp 
#                                    src/trajectory_xml_parser/trajectory_from_raw_to_xml.cpp 
#                                    src/trajectory_xml_parser/test_trajectory_xml_parser.cpp 
#                                    src/trajectory_xml_parser/test_loading_trajectory_xml.cpp 
#                                    src/trajectory_xml_parser/concat_trajectory_xml_files.cpp
#                                    src/utils/test_buffer_toggle.cpp 
#                                    src/utils/test_thread_safe_variable.cpp
#                                    src/vacuum_gripper/vacuum_gripper.cpp 
#                                    src/vacuum_gripper/test_vacuum_gripper.cpp)

## Specify the header files
# set(folder_header_baxter_tictactoe include/board_motion_detection.h
#                                    include/BufferToggle.h
#                                    include/move_maker.h
#                                    include/robot_state.h
#                                    include/ros_time_utils.h
#                                    include/trajectory_player.h
#                                    include/trajectory_xml_parser.h
#                                    include/T_ThreadSafe.h
#                                    include/ttt_moves_trajectories.h
#                                    include/vacuum_gripper.h)

# source_group("Source Files" FILES ${folder_source_baxter_tictactoe})
# source_group("Header Files" FILES ${folder_header_baxter_tictactoe})
