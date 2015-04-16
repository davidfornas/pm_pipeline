cmake_minimum_required(VERSION 2.8.3)
project(pm_perception)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs pm_tools)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(VISP REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pm_perception
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs pm_tools
  DEPENDS PCL Boost VISP OpenCV
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

## Declare a cpp library
add_library(pm_perception 
  src/background_removal.cpp
  src/object_segmentation.cpp
  src/border_detection.cpp
  src/pose_estimation.cpp
)

## Declare a cpp executable
# add_executable(pm_perception_node src/pm_perception_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pm_perception_node pm_perception_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pm_perception ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} ${OpenCV})

