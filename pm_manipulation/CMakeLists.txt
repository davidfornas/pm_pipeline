cmake_minimum_required(VERSION 2.8.3)
project(pm_manipulation)

find_package(catkin REQUIRED COMPONENTS roscpp pm_perception tf mar_robot_arm5e pm_tools )

find_package(PCL)
find_package(VISP)
find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pm_manipulation
  CATKIN_DEPENDS roscpp pm_perception tf mar_robot_arm5e pm_tools
  DEPENDS PCL VISP OpenCV
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

## Declare a cpp library
add_library(pm_manipulation 
  src/grasp_hypothesis_evaluation.cpp
  src/hypothesis_generation.cpp
  src/pm_grasp_planning.cpp
  src/trajectory_following.cpp
)

## Declare a cpp executable
# add_executable(pm_manipulation_node src/pm_manipulation_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pm_manipulation_node pm_manipulation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pm_manipulation ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${VISP_LIBRARIES})


