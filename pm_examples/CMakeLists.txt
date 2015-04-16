cmake_minimum_required(VERSION 2.8.3)
project(pm_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp pm_manipulation pm_perception pm_tools
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pm_examples
  CATKIN_DEPENDS roscpp pm_manipulation pm_perception pm_tools
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(pm_examples
#   src/${PROJECT_NAME}/pm_examples.cpp
# )

## Declare a cpp executable
add_executable(grasp_planning_example src/grasp_planning_example.cpp)
add_executable(border_following_example src/border_following_example.cpp)
add_executable(border_following_full_example src/border_following_full_example.cpp)
add_executable(concave_hull_example src/concave_hull_example.cpp)
add_executable(cloud_merger_example src/cloud_merger_example.cpp)
add_executable(pose_estimation_example src/pose_estimation_example.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pm_examples_node pm_examples_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(grasp_planning_example ${catkin_LIBRARIES} )
target_link_libraries(border_following_example ${catkin_LIBRARIES} )
target_link_libraries(border_following_full_example ${catkin_LIBRARIES} )
target_link_libraries(concave_hull_example ${catkin_LIBRARIES} )
target_link_libraries(cloud_merger_example ${catkin_LIBRARIES} )
target_link_libraries(pose_estimation_example ${catkin_LIBRARIES} )
