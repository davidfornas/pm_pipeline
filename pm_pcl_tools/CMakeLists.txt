cmake_minimum_required(VERSION 2.8.3)
project(pm_pcl_tools)

find_package(catkin REQUIRED COMPONENTS  roscpp sensor_msgs pcl_conversions )

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pm_pcl_tools
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions
  DEPENDS PCL
)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

add_library(pm_pcl_tools
  src/pcl_tools.cpp
)
target_link_libraries(pm_pcl_tools ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
