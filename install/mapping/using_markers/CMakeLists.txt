cmake_minimum_required(VERSION 2.8.3)
project(using_markers)

add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  visualization_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES using_markers
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})