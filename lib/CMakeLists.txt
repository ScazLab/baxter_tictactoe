# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Álvaro Castro González, Olivier Mangin, Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
# project(baxter_tictactoe)

include_directories(${PROJECT_SOURCE_DIR}/include)

## Declare a C++ library
add_library(${PROJECT_NAME}   include/${PROJECT_NAME}/tictactoe_utils.h
                              include/${PROJECT_NAME}/ttt_controller.h
                              src/${PROJECT_NAME}/tictactoe_utils.cpp
                              src/${PROJECT_NAME}/ttt_controller.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}   baxter_tictactoe_generate_messages_cpp
                                   ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}   ${OpenCV_LIBRARIES}
                                        ${QT_LIBRARIES}
                                        ${catkin_LIBRARIES})

## Mark libraries for installation
install (TARGETS ${PROJECT_NAME}
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

