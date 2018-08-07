cmake_minimum_required(VERSION 2.8.3)
project(calibration)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES calibration
#  CATKIN_DEPENDS pcl_ros roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(libyaml_editor src/yaml_editor.cpp)

add_executable(lidar_init_calibration src/yaml_editor.cpp nodes/lidar_calibration/lidar_init_calibration.cpp)
target_link_libraries(lidar_init_calibration ${catkin_LIBRARIES} yaml-cpp)
