cmake_minimum_required(VERSION 2.8.3)
project(detection)
add_compile_options(-std=c++17 -Wall -Wextra)
#set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(tesseract REQUIRED tesseract)
pkg_check_modules(yaml-cpp REQUIRED yaml-cpp)
message([INFO] "Tesseract version: ${tesseract_VERSION}")
message([INFO] "yaml-cpp version: ${yaml-cpp_VERSION}")


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  darknet_ros_msgs
  octomap_msgs
  octomap_ros
  roscpp
  std_msgs
  roslib
  tf2_ros
  message_filters
  tf2
  dynamic_reconfigure
  darknet_ros_msgs
)
generate_dynamic_reconfigure_options(
   cfg/detection.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES detection
  CATKIN_DEPENDS cv_bridge image_transport darknet_ros_msgs octomap_msgs roscpp std_msgs roslib octomap_ros tf2 dynamic_reconfigure darknet_ros_msgs
)



###########
## Build ##
###########
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#################
## Executables ##
#################

add_executable(${PROJECT_NAME}_main src/main.cpp src/calc_pose.cpp)
target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} ${OpenCV_LIBS} lept yaml-cpp ${tesseract_LIBRARIES})
#add_dependencies(${PROJECT_NAME}_main)
