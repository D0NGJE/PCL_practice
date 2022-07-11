cmake_minimum_required(VERSION 3.0.2)
project(PCL_practice)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES PCL_practice
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kd_tree src/kd_tree.cpp)
target_link_libraries(kd_tree ${catkin_LIBRARIES})
