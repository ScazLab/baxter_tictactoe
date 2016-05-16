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
             #ttt_board_sensor
             baxter_core_msgs
	     std_msgs
	     cv_bridge
	     image_transport
)

find_package(Qt4 COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML 1)
SET(QT_DONT_USE_QTGUI 0)
include(${QT_USE_FILE})

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_service_files(FILES
    SetTrajectoryType.srv
)

add_message_files(FILES
    ttt_board.msg
)

#generate_messages(
#    DEPENDENCIES
#    std_msgs
#)

#Required for generating the messages related to actionlib
add_action_files(DIRECTORY action FILES PlaceToken.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime
)

MESSAGE(${catkin_INCLUDE_DIRS})
MESSAGE(${catkin_LIBRARIES})
# Targets

# trajectory xml parser
add_library(trajectory_xml_parser src/trajectory_xml_parser/trajectory_xml_parser.cpp src/trajectory_xml_parser/trajectory_xml_parser.h)
target_link_libraries(trajectory_xml_parser ${QT_LIBRARIES} ${catkin_LIBRARIES})
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
target_link_libraries(test_vacuum_gripper vacuum_gripper ${catkin_LIBRARIES})

# robot state
add_library(robot_state src/robot_state/robot_state.cpp src/robot_state/robot_state.h)
add_executable(test_robot_state src/robot_state/test_robot_state.cpp)
target_link_libraries(test_robot_state robot_state ${catkin_LIBRARIES})

# trajectory player library
add_library(trajectory_player src/trajectory_player/trajectory_player.cpp src/trajectory_player/trajectory_player.h)
target_link_libraries(trajectory_player vacuum_gripper)
add_executable(test_trajectory_player src/trajectory_player/test_trajectory_player.cpp)
target_link_libraries(test_trajectory_player trajectory_player trajectory_xml_parser ${catkin_LIBRARIES})

# move_maker
add_library(move_maker src/move_maker/move_maker.cpp src/move_maker/move_maker.h src/utils/my_ros_utils.h src/move_maker/ttt_moves_trajectories.h)
add_dependencies(move_maker ${catkin_EXPORTED_TARGETS})
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
add_executable(ttt_brain src/brain/ttt_brain.cpp)
target_link_libraries(ttt_brain
                      vacuum_gripper robot_state
                      ${QT_LIBRARIES} ${catkin_LIBRARIES})

# Launchers
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Build a ttt_board_sensor library
add_library(${PROJECT_NAME} src/board_sensor/tictactoe_utils.cpp)
target_link_libraries(${PROJECT_NAME}
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(cells_delimitation include/ttt/board_cells_delimitation.h
                                  src/board_sensor/board_cells_delimitation.cpp)
target_link_libraries(cells_delimitation ${PROJECT_NAME} ${OpenCV_LIBS} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(show_board_cells src/board_sensor/show_board_cells.cpp)
target_link_libraries(show_board_cells ${PROJECT_NAME} ${OpenCV_LIBS} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(hsv_range_finder src/board_sensor/hsv_range_finder.cpp)
add_dependencies(hsv_range_finder ${catkin_EXPORTED_TARGETS})
target_link_libraries(hsv_range_finder ${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(board_scheme src/board_sensor/board_scheme.cpp)
add_dependencies(board_scheme ${catkin_EXPORTED_TARGETS})
target_link_libraries(board_scheme ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(board_state_sensing src/board_sensor/board_state_sensing.cpp)
target_link_libraries(board_state_sensing ${PROJECT_NAME} ${OpenCV_LIBS} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(board_motion_detection include/ttt/board_motion_detection.h
                                      src/board_sensor/board_motion_detection.cpp)
target_link_libraries(board_motion_detection ${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
        TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME  DESTINATION bin  # This is for Windows
)

install(DIRECTORY include/ 
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)





