# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Álvaro Castro González, Olivier Mangin, Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(baxterTictactoe)

include_directories(${PROJECT_SOURCE_DIR}/include)

## Declare a C++ library
add_library(${PROJECT_NAME}   include/${PROJECT_NAME}/tictactoe_utils.h
                              include/${PROJECT_NAME}/T_ThreadSafe.h
                              src/tictactoe_utils.cpp)
add_library(arm_control       include/arm_control/gripper.h
                              include/arm_control/arm_controller.h
                              src/gripper.cpp
                              src/arm_controller.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}   baxter_tictactoe_generate_messages_cpp
                                   ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS})

add_dependencies(arm_control       ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}        ${QT_LIBRARIES}
                                             ${OpenCV_LIBRARIES}
                                             ${catkin_LIBRARIES})

target_link_libraries(arm_control         ${catkin_LIBRARIES})

IF(COMPILE_OLD_SW STREQUAL true)
    add_library(move_maker      include/move_maker/trajectory_xml_parser.h
                                include/move_maker/trajectory_player.h
                                include/move_maker/move_maker.h
                                src/trajectory_xml_parser.cpp
                                src/trajectory_player.cpp
                                src/move_maker.cpp)
    add_library(robot_state     include/robot_state/robot_state.h
                                src/robot_state.cpp)

    add_dependencies(move_maker         ${catkin_EXPORTED_TARGETS})
    target_link_libraries(move_maker    arm_control 
                                        ${QT_LIBRARIES}
                                        ${catkin_LIBRARIES})
ENDIF()

## Mark libraries for installation
install (TARGETS ${PROJECT_NAME} arm_control
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/baxterTictactoe
                  ${PROJECT_SOURCE_DIR}/include/arm_control
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

