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

#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(my_ik_test REQUIRED)
#include_directories($(my_ik_test_INCLUDE_DIRS))
#pkg_check_modules(inverse_kinematics REQUIRED)
#include_directories($(inverse_kinematics_INCLUDE_DIRS))
#pkg_check_modules(baxter_msgs REQUIRED)
#include_directories(${baxter_msgs_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(ik_client src/ik_client.cpp)

include_directories($(my_ik_test_INCLUDE_DIRS))

rosbuild_add_executable(keypad src/keypad.cpp)

rosbuild_add_executable(listening_joint_states src/listening_joint_states.cpp)

rosbuild_add_executable(test_thread_safe_variable src/utils/test_thread_safe_variable.cpp)

rosbuild_add_executable(test_buffer_toggle src/utils/test_buffer_toggle.cpp)

#find_package(Eigen REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})

#find_package(moveit_core REQUIRED)
#include_directories(${moveit_core_INCLUDE_DIRS})
#link_directories(${moveit_core_LIBRARY_DIRS})


rosbuild_add_executable(moveit_test src/moveit_test.cpp)

rosbuild_add_executable(kinematics_demo src/kinematics_demo.cpp)

rosbuild_add_executable(kinematics_ros_api_demo src/kinematics_ros_api_demo.cpp)

rosbuild_add_executable(pick_place_tutorial src/pick_place_tutorial.cpp)

rosbuild_add_executable(follow_joint_trajectory_demo src/follow_joint_trajectory_demo.cpp)

rosbuild_add_executable(two_trajectories_demo src/two_trajectories_demo.cpp)

find_package(Qt4 COMPONENTS QtCore QtGui)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

rosbuild_add_executable(play_trajectory_from_file src/play_trajectory_from_file.cpp)
TARGET_LINK_LIBRARIES(play_trajectory_from_file ${QT_LIBRARIES})

rosbuild_add_executable(motion_detection src/motion_detection.cpp)
