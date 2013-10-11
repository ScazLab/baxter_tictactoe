cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML 1)
SET(QT_DONT_USE_QTGUI 0)
include(${QT_USE_FILE})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories ($(ttt_board_sensor_INCLUDE_DIRS))
include_directories (${ttt_board_sensor_SOURCE_DIR}/hsv_range_finder)
include_directories (${ttt_board_sensor_SOURCE_DIR}/tictactoe_board_display)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(ttt src/ttt_cells.cpp src/ttt_cells.h src/ttt_definitions.h)

rosbuild_add_executable(cells_delimitation src/board_cells_delimitation.cpp src/board_cells_delimitation.h)
TARGET_LINK_LIBRARIES(cells_delimitation ttt)
TARGET_LINK_LIBRARIES(cells_delimitation ${QT_LIBRARIES})

rosbuild_add_executable(show_board_cells src/show_board_cells.cpp)
TARGET_LINK_LIBRARIES(show_board_cells ttt)
TARGET_LINK_LIBRARIES(show_board_cells ${QT_LIBRARIES})

rosbuild_add_executable(hsv_range_finder src/hsv_range_finder/hsv_range_finder.cpp)

rosbuild_add_executable(board_scheme src/tictactoe_board_display/board_scheme.cpp)

rosbuild_add_executable(board_state_sensing src/board_state_sensing.cpp)
TARGET_LINK_LIBRARIES(board_state_sensing ttt)
TARGET_LINK_LIBRARIES(board_state_sensing ${QT_LIBRARIES})
