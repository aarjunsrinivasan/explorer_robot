cmake_minimum_required(VERSION 3.2.1)
project(explorer_robot)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES explorer_robot
#  CATKIN_DEPENDS genmsg geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(explorerBot src/main.cpp src/explorerBot.cpp)
target_link_libraries(explorerBot ${catkin_LIBRARIES})

