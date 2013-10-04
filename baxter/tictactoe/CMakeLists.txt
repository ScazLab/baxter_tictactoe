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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


include_directories($(tictactoe_INCLUDE_DIRS))

find_package(Qt4 COMPONENTS QtCore QtGui)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

rosbuild_add_library(trajectory_xml_parser src/trajectory_xml_parser/trajectory_xml_parser.cpp)
target_link_libraries(trajectory_xml_parser ${QT_LIBRARIES})

rosbuild_add_executable(test_trajectory_xml_parser src/trajectory_xml_parser/test_trajectory_xml_parser.cpp)
target_link_libraries(test_trajectory_xml_parser trajectory_xml_parser)

rosbuild_add_library(trajectory_player src/trajectory_player/trajectory_player.cpp)

#tests at utils
rosbuild_add_executable(test_thread_safe_variable src/utils/test_thread_safe_variable.cpp)
rosbuild_add_executable(test_buffer_toggle src/utils/test_buffer_toggle.cpp)
