cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_example_ros_ws)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    pcl_ros
    geometry_msgs
)
find_package(PCL 1.7 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

### EXECUTABLES ###############################################
add_executable(detect_plane_node
  src/plane_detection_node.cpp
)
target_link_libraries(detect_plane_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
