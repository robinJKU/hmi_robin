cmake_minimum_required(VERSION 2.8.3)
project(speech_database)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs sound_play)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} )

## Declare a cpp executable
add_executable(speech_database src/speed_database.cpp)
target_link_libraries(speech_database ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(speech_database   ${catkin_LIBRARIES})
target_link_libraries(speech_database curl)
target_link_libraries(speech_database jansson)

