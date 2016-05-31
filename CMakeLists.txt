# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Álvaro Castro González, Olivier Mangin, Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.
cmake_minimum_required(VERSION 2.8.3)
project(baxter_tictactoe)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             message_generation
             genmsg
             actionlib_msgs
             actionlib
             rosconsole
             sound_play
             baxter_core_msgs
    	     std_msgs
    	     cv_bridge
    	     image_transport
)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
find_package(OpenCV 2.4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(FILES
    Point.msg
    MsgBoard.msg 
    MsgCell.msg 
)

## Generate services in the 'srv' folder
add_service_files(FILES
    SetTrajectoryType.srv
    DefineCells.srv
)

#Required for generating the messages related to actionlib
add_action_files(DIRECTORY action FILES 
    PlaceToken.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES 
    actionlib_msgs 
    std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS lib/include/
    LIBRARIES baxterTictactoe 
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(lib/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}
)

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

## Add libraries
add_subdirectory(lib)

## Declare a C++ executables
add_executable(test_trajectory_xml_parser       src/test_trajectory_xml_parser/test_trajectory_xml_parser.cpp)
add_executable(test_loading_trajectory_xml      src/test_loading_trajectory_xml/test_loading_trajectory_xml.cpp)
add_executable(trajectory_from_raw_to_xml       src/trajectory_from_raw_to_xml/trajectory_from_raw_to_xml.cpp)
add_executable(concat_trajectory_xml_files      src/concat_trajectory_xml_files/concat_trajectory_xml_files.cpp)
add_executable(test_vacuum_gripper              src/test_vacuum_gripper/test_vacuum_gripper.cpp)
add_executable(test_trajectory_player           src/test_trajectory_player/test_trajectory_player.cpp)
add_executable(move_maker_server                src/move_maker_server/move_maker_server.cpp)
add_executable(move_maker_simpleclient          src/move_maker_simpleclient/move_maker_simpleclient.cpp)
add_executable(test_thread_safe_variable        src/test_thread_safe_variable/test_thread_safe_variable.cpp lib/include/T_ThreadSafe.h)
add_executable(tictactoe_brain                  src/tictactoe_brain/tictactoeBrain.h                        src/tictactoe_brain/tictactoeBrain.cpp src/tictactoe_brain/tictactoe_brain.cpp                                              )
add_executable(cells_definition                 src/cells_definition/cells_definition.cpp                   src/cells_definition/cells_definition.h)
add_executable(cells_definition_auto            src/cells_definition/cells_definition_auto.cpp              src/cells_definition/cells_definition_auto.h)
add_executable(show_board_cells                 src/show_board_cells/show_board_cells.cpp)
add_executable(hsv_range_finder                 src/hsv_range_finder/hsv_range_finder.cpp)
add_executable(board_scheme                     src/board_scheme/board_scheme.cpp)
add_executable(board_state_sensing              src/board_state_sensing/board_state_sensing.cpp             src/board_state_sensing/board_state_sensing.h)
add_executable(board_motion_detection           src/board_motion_detection/board_motion_detection.cpp       src/board_motion_detection/board_motion_detection.h)

add_executable(right_arm_joint_pub_test         src/right_arm_joint_pub_test.cpp)

## Add cmake target dependencies of the executable
add_dependencies(test_trajectory_xml_parser     baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(test_loading_trajectory_xml    baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_from_raw_to_xml     baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(concat_trajectory_xml_files    baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(test_vacuum_gripper            baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(test_trajectory_player         baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(move_maker_server              baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(move_maker_simpleclient        baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(test_thread_safe_variable      baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(tictactoe_brain                baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(cells_definition               baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(cells_definition_auto          baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(show_board_cells               baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(hsv_range_finder               baxter_tictactoe_generate_messages_cpp 		${${PROJECT_NAME}_EXPORTED_TARGETS} 	${catkin_EXPORTED_TARGETS})
add_dependencies(board_scheme                   baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(board_state_sensing            baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(board_motion_detection         baxter_tictactoe_generate_messages_cpp 		${${PROJECT_NAME}_EXPORTED_TARGETS} 	${catkin_EXPORTED_TARGETS})

add_dependencies(right_arm_joint_pub_test       baxter_tictactoe_generate_messages_cpp      ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(test_trajectory_xml_parser        trajectory_xml_parser)
target_link_libraries(test_loading_trajectory_xml       trajectory_xml_parser)
target_link_libraries(trajectory_from_raw_to_xml        trajectory_xml_parser)
target_link_libraries(concat_trajectory_xml_files       trajectory_xml_parser)
target_link_libraries(test_vacuum_gripper               vacuum_gripper                                                      ${catkin_LIBRARIES})
target_link_libraries(test_trajectory_player            trajectory_player   trajectory_xml_parser                           ${catkin_LIBRARIES})
target_link_libraries(move_maker_server                 move_maker)
target_link_libraries(move_maker_simpleclient           ${Boost_LIBRARIES}  move_maker)
target_link_libraries(tictactoe_brain                   baxterTictactoe     vacuum_gripper                                  ${catkin_LIBRARIES})
target_link_libraries(cells_definition                  baxterTictactoe     ${OpenCV_LIBS}          ${QT_LIBRARIES}         ${catkin_LIBRARIES})
target_link_libraries(cells_definition_auto             baxterTictactoe     ${OpenCV_LIBS}                                  ${catkin_LIBRARIES})
target_link_libraries(show_board_cells                  baxterTictactoe     ${OpenCV_LIBS}          ${QT_LIBRARIES}         ${catkin_LIBRARIES})
target_link_libraries(hsv_range_finder                  baxterTictactoe     ${OpenCV_LIBS}                                  ${catkin_LIBRARIES})
target_link_libraries(board_scheme                                          ${OpenCV_LIBS}                                  ${catkin_LIBRARIES})
target_link_libraries(board_state_sensing               baxterTictactoe     ${OpenCV_LIBS}          ${QT_LIBRARIES}         ${catkin_LIBRARIES})
target_link_libraries(board_motion_detection            baxterTictactoe     ${OpenCV_LIBS}                                  ${catkin_LIBRARIES})

target_link_libraries(right_arm_joint_pub_test          baxterTictactoe     ${OpenCV_LIBS}          ${QT_LIBRARIES}         ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/ 
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

