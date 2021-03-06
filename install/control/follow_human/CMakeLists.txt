cmake_minimum_required(VERSION 2.8.3)
project(follow_human)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -Wall -Wextra)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  pid
  roscpp
  std_msgs
  roslib
  cv_bridge
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/follow_human.cfg
  #...
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES follow_human
  CATKIN_DEPENDS darknet_ros_msgs pid roscpp std_msgs roslib cv_bridge dynamic_reconfigure
  #DEPENDS system_lib
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/follow_human.cpp
# )

add_executable(${PROJECT_NAME}_main src/main.cpp src/follow_human.cpp)
target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
add_dependencies(${PROJECT_NAME}_main ${PROJECT_NAME}_gencfg)

