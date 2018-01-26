cmake_minimum_required(VERSION 2.8.3)
project(pm_manipulation)

find_package(catkin REQUIRED COMPONENTS roscpp tf mar_robot_arm5e pm_tools pm_perception nav_msgs geometry_msgs)

find_package(PCL)
find_package(VISP)
find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pm_manipulation
  CATKIN_DEPENDS roscpp tf mar_robot_arm5e pm_tools pm_perception nav_msgs geometry_msgs
  DEPENDS PCL VISP OpenCV
)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})


add_library(pm_manipulation src/slider_grasp_planner.cpp)
target_link_libraries(pm_manipulation ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${VISP_LIBRARIES})


