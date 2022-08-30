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

add_executable(filter src/filter.cpp)
target_link_libraries(filter ${catkin_LIBRARIES})

add_executable(normal_estimation src/normal_estimation.cpp)
target_link_libraries(normal_estimation ${catkin_LIBRARIES})

add_executable(octree src/octree.cpp)
target_link_libraries(octree ${catkin_LIBRARIES})

add_executable(dbscan src/dbscan.cpp)
target_link_libraries(dbscan ${catkin_LIBRARIES})
