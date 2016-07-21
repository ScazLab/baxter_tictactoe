# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Álvaro Castro González, Olivier Mangin, Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(baxterTictactoe)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

## Declare a C++ library
add_library(${PROJECT_NAME}       include/${PROJECT_NAME}/tictactoe_utils.h     include/${PROJECT_NAME}/T_ThreadSafe.h src/tictactoe_utils.cpp)
add_library(trajectory_xml_parser include/trajectory_xml_parser/trajectory_xml_parser.h         src/trajectory_xml_parser.cpp)
add_library(vacuum_gripper        include/vacuum_gripper/vacuum_gripper.h                       src/vacuum_gripper.cpp)
add_library(robot_state           include/robot_state/robot_state.h                             src/robot_state.cpp)
add_library(trajectory_player     include/trajectory_player/trajectory_player.h                 src/trajectory_player.cpp)
add_library(move_maker            include/move_maker/move_maker.h                               src/move_maker.cpp)
add_library(arm_controller        include/arm_controller/arm_controller.h                       src/arm_controller.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}   baxter_tictactoe_generate_messages_cpp       ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(move_maker        ${catkin_EXPORTED_TARGETS})
add_dependencies(arm_controller    ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}        ${QT_LIBRARIES}               ${OpenCV_LIBRARIES}          ${catkin_LIBRARIES})
target_link_libraries(trajectory_xml_parser  ${QT_LIBRARIES}               ${catkin_LIBRARIES})
target_link_libraries(trajectory_player      vacuum_gripper                ${catkin_LIBRARIES})
target_link_libraries(move_maker             trajectory_player             trajectory_xml_parser)
target_link_libraries(arm_controller         vacuum_gripper                ${catkin_LIBRARIES})

## Mark libraries for installation
install (TARGETS ${PROJECT_NAME} arm_controller vacuum_gripper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/baxterTictactoe ${PROJECT_SOURCE_DIR}/include/arm_controller ${PROJECT_SOURCE_DIR}/include/vacuum_gripper
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

