cmake_minimum_required(VERSION 2.8.3)
project(pm_examples)

find_package(catkin REQUIRED COMPONENTS roscpp pm_manipulation pm_perception pm_tools)

catkin_package(
  CATKIN_DEPENDS roscpp pm_manipulation pm_perception pm_tools
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(grasp_planning_example src/grasp_planning_example.cpp)
add_executable(border_following_example src/border_following_example.cpp)
add_executable(border_following_full_example src/border_following_full_example.cpp)
add_executable(concave_hull_example src/concave_hull_example.cpp)
add_executable(cloud_merger_example src/cloud_merger_example.cpp)
add_executable(pose_estimation_example src/pose_estimation_example.cpp)
add_executable(clustering_example src/clustering_example.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(grasp_planning_example ${catkin_LIBRARIES} )
target_link_libraries(border_following_example ${catkin_LIBRARIES} )
target_link_libraries(border_following_full_example ${catkin_LIBRARIES} )
target_link_libraries(concave_hull_example ${catkin_LIBRARIES} )
target_link_libraries(cloud_merger_example ${catkin_LIBRARIES} )
target_link_libraries(pose_estimation_example ${catkin_LIBRARIES} )
target_link_libraries(clustering_example ${catkin_LIBRARIES} )
