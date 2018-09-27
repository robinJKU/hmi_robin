cmake_minimum_required(VERSION 2.8.3)
project(speech_database)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sound_play)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sound_play
)

include_directories(${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

## Declare a cpp executable
add_executable(speech_database src/speech_database.cpp)
add_dependencies(speech_database ${catkin_EXPORTED_TARGETS})
target_link_libraries(speech_database ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(speech_database   ${catkin_LIBRARIES})
target_link_libraries(speech_database curl)
target_link_libraries(speech_database jansson)

#############
## Install ##
#############

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS speech_database
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
