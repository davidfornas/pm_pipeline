cmake_minimum_required(VERSION 2.8.3)
project(pm_manipulation)

find_package(catkin REQUIRED COMPONENTS roscpp tf mar_robot_arm5e pm_tools nav_msgs geometry_msgs)

find_package(PCL)
find_package(VISP)
find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pm_manipulation
  CATKIN_DEPENDS roscpp tf mar_robot_arm5e pm_tools nav_msgs geometry_msgs
  DEPENDS PCL VISP OpenCV
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

## Declare a cpp library
add_library(pm_manipulation 
  src/pm_grasp_planning.cpp
  src/pm_grasp_planning_split.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(pm_manipulation ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${VISP_LIBRARIES})


