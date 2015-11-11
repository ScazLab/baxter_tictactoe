cmake_minimum_required(VERSION 2.8.3)
project(ttt_board_sensor)
find_package(catkin REQUIRED std_msgs message_generation rosconsole cv_bridge image_transport)

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
generate_messages(DEPENDENCIES std_msgs)

#include_directories ($(ttt_board_sensor_INCLUDE_DIRS))
#include_directories (${ttt_board_sensor_SOURCE_DIR}/hsv_range_finder)
#include_directories (${ttt_board_sensor_SOURCE_DIR}/tictactoe_board_display)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS message_runtime
)

# Targets

add_library(ttt src/ttt_cells.cpp src/ttt_cells.h src/ttt_definitions.h)
add_dependencies(ttt ${catkin_EXPORTED_TARGETS})

add_executable(cells_delimitation src/board_cells_delimitation.cpp src/board_cells_delimitation.h)
target_link_libraries(cells_delimitation ttt ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(show_board_cells src/show_board_cells.cpp)
target_link_libraries(show_board_cells ttt ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(hsv_range_finder src/hsv_range_finder/hsv_range_finder.cpp)
add_dependencies(hsv_range_finder ${catkin_EXPORTED_TARGETS})
target_link_libraries(hsv_range_finder ttt ${catkin_LIBRARIES})

add_executable(board_scheme src/tictactoe_board_display/board_scheme.cpp)
add_dependencies(board_scheme ${catkin_EXPORTED_TARGETS})
target_link_libraries(board_scheme ${catkin_LIBRARIES})

add_executable(board_state_sensing src/board_state_sensing.cpp)
target_link_libraries(board_state_sensing ttt ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(board_motion_detection src/motion_detection/board_motion_detection.cpp src/motion_detection/board_motion_detection.h)
target_link_libraries(board_motion_detection ttt ${catkin_LIBRARIES})
