cmake_minimum_required(VERSION 2.8.3)
project(pm_measure)

find_package(catkin REQUIRED COMPONENTS  roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pm_measure
  CATKIN_DEPENDS roscpp
)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(pm_measure 
  src/timing.cpp
)
target_link_libraries(pm_measure ${catkin_LIBRARIES})
