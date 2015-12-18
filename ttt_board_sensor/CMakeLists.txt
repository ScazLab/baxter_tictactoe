cmake_minimum_required(VERSION 2.8.3)
project(ttt_board_sensor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             std_msgs
             message_generation
             rosconsole
             cv_bridge
             image_transport
)

find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML 1)
SET(QT_DONT_USE_QTGUI 0)
include(${QT_USE_FILE})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(FILES
    ttt_board.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
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

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Build a ttt_board_sensor library
add_library(${PROJECT_NAME} src/tictactoe_utils.cpp)
target_link_libraries(${PROJECT_NAME}
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(cells_delimitation include/ttt/board_cells_delimitation.h
                                  src/board_cells_delimitation.cpp)
target_link_libraries(cells_delimitation ${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(show_board_cells src/show_board_cells.cpp)
target_link_libraries(show_board_cells ${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(hsv_range_finder src/hsv_range_finder.cpp)
add_dependencies(hsv_range_finder ${catkin_EXPORTED_TARGETS})
target_link_libraries(hsv_range_finder ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(board_scheme src/board_scheme.cpp)
add_dependencies(board_scheme ${catkin_EXPORTED_TARGETS})
target_link_libraries(board_scheme ${catkin_LIBRARIES})

add_executable(board_state_sensing src/board_state_sensing.cpp)
target_link_libraries(board_state_sensing ${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(board_motion_detection include/ttt/board_motion_detection.h
                                      src/board_motion_detection.cpp)
target_link_libraries(board_motion_detection ${PROJECT_NAME} ${catkin_LIBRARIES})

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

