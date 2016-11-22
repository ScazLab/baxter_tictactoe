# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Alessandro Roncone, Olivier Mangin, Álvaro Castro González
# CopyPolicy: Released under the terms of the GNU GPL v3.0.
cmake_minimum_required(VERSION 2.8.3)
project(baxter_tictactoe)

set(COMPILE_TESTS  false)
set(COMPILE_OLD_SW false)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             message_generation
             genmsg
             rosconsole
             sound_play
             baxter_core_msgs
    	     std_msgs
    	     cv_bridge
    	     image_transport
             baxter_collaboration
             eigen_conversions
)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
find_package(OpenCV 2.4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Threads)

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
    ScanState.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    baxter_core_msgs
    geometry_msgs
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
    INCLUDE_DIRS lib/include
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
                    ${eigen_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}
)

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

## Add libraries
add_subdirectory(lib)

## Declare a C++ executables
add_executable(tictactoe_brain                  src/tictactoe_brain/tictactoeBrain.h
                                                src/tictactoe_brain/tictactoeBrain.cpp
                                                src/tictactoe_brain/tictactoe_brain.cpp)
add_executable(show_board_cells                 src/show_board_cells/show_board_cells.cpp)
add_executable(hsv_range_finder                 src/hsv_range_finder/hsv_range_finder.cpp)
add_executable(board_scheme                     src/board_scheme/board_scheme.cpp)
add_executable(board_state_sensor               src/board_state_sensing/board_state_sensing.h
                                                src/board_state_sensing/board_state_sensing.cpp
                                                src/board_state_sensing/cells_definition.h
                                                src/board_state_sensing/cells_definition.cpp
                                                src/board_state_sensing/board_state_sensor.cpp)

## Add cmake target dependencies of the executable
add_dependencies(tictactoe_brain                baxter_tictactoe_generate_messages_cpp
                                                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                ${catkin_EXPORTED_TARGETS})
add_dependencies(show_board_cells               baxter_tictactoe_generate_messages_cpp
                                                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                ${catkin_EXPORTED_TARGETS})
add_dependencies(hsv_range_finder               baxter_tictactoe_generate_messages_cpp
                                                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                ${catkin_EXPORTED_TARGETS})
add_dependencies(board_scheme                   baxter_tictactoe_generate_messages_cpp
                                                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                ${catkin_EXPORTED_TARGETS})
add_dependencies(board_state_sensor             baxter_tictactoe_generate_messages_cpp
                                                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(tictactoe_brain           baxterTictactoe
                                                ttt_controller
                                                ${catkin_LIBRARIES}
                                                ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(show_board_cells          baxterTictactoe
                                                ${OpenCV_LIBS}
                                                ${QT_LIBRARIES}
                                                ${catkin_LIBRARIES})
target_link_libraries(hsv_range_finder          baxterTictactoe
                                                ${OpenCV_LIBS}
                                                ${catkin_LIBRARIES})
target_link_libraries(board_scheme              ${OpenCV_LIBS}
                                                ${catkin_LIBRARIES})
target_link_libraries(board_state_sensor        baxterTictactoe
                                                ${OpenCV_LIBS}
                                                ${QT_LIBRARIES}
                                                ${catkin_LIBRARIES})

# Compile old sw if required
IF (COMPILE_OLD_SW STREQUAL true)
    add_executable(trajectory_from_raw_to_xml       src/old/trajectory_from_raw_to_xml.cpp)
    add_executable(concat_trajectory_xml_files      src/old/concat_trajectory_xml_files.cpp)
    add_executable(move_maker_server                src/old/move_maker_server.cpp)
    add_executable(move_maker_simpleclient          src/old/move_maker_simpleclient.cpp)

    add_dependencies(trajectory_from_raw_to_xml     baxter_tictactoe_generate_messages_cpp
                                                    ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                    ${catkin_EXPORTED_TARGETS})
    add_dependencies(concat_trajectory_xml_files    baxter_tictactoe_generate_messages_cpp
                                                    ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                    ${catkin_EXPORTED_TARGETS})
    add_dependencies(move_maker_server              baxter_tictactoe_generate_messages_cpp
                                                    ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                    ${catkin_EXPORTED_TARGETS})
    add_dependencies(move_maker_simpleclient        baxter_tictactoe_generate_messages_cpp
                                                    ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                    ${catkin_EXPORTED_TARGETS})

    target_link_libraries(trajectory_from_raw_to_xml        move_maker)
    target_link_libraries(concat_trajectory_xml_files       move_maker)
    target_link_libraries(move_maker_server                 move_maker)
    target_link_libraries(move_maker_simpleclient           ${Boost_LIBRARIES}  move_maker)
ENDIF()

# Compile tests if required
IF(COMPILE_TESTS STREQUAL true)
    add_executable(test_vacuum_gripper          src/test/test_vacuum_gripper.cpp)
    add_executable(test_arm_controller          src/test/test_arm_controller.cpp)

    add_dependencies(test_vacuum_gripper        baxter_tictactoe_generate_messages_cpp)
    add_dependencies(test_arm_controller        baxter_tictactoe_generate_messages_cpp)

    target_link_libraries(test_vacuum_gripper                   ${catkin_LIBRARIES})
    target_link_libraries(test_arm_controller   ${OpenCV_LIBS}  ${catkin_LIBRARIES})

    IF(COMPILE_OLD_SW STREQUAL true)
        add_executable(test_trajectory_player           src/test/test_trajectory_player.cpp)
        add_executable(test_thread_safe_variable        src/test/test_thread_safe_variable.cpp)
        add_executable(test_trajectory_xml_parser       src/test/test_trajectory_xml_parser.cpp)
        add_executable(test_loading_trajectory_xml      src/test/test_loading_trajectory_xml.cpp)

        add_dependencies(test_trajectory_player         baxter_tictactoe_generate_messages_cpp)
        add_dependencies(test_thread_safe_variable      baxter_tictactoe_generate_messages_cpp)
        add_dependencies(test_trajectory_xml_parser     baxter_tictactoe_generate_messages_cpp)
        add_dependencies(test_loading_trajectory_xml    baxter_tictactoe_generate_messages_cpp)

        target_link_libraries(test_trajectory_player            move_maker      ${catkin_LIBRARIES})
        target_link_libraries(test_trajectory_xml_parser        move_maker)
        target_link_libraries(test_loading_trajectory_xml       move_maker)
    ENDIF()
ELSE()
    message(${PROJECT_NAME} ": Tests will not be compiled")
ENDIF()

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

# install(DIRECTORY include/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

