cmake_minimum_required(VERSION 2.8.3)
project(pm_examples)

find_package(catkin REQUIRED COMPONENTS roscpp pm_manipulation pm_tools pm_perception mar_perception)

catkin_package(
  CATKIN_DEPENDS roscpp pm_manipulation pm_tools pm_perception mar_perception
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(grasp_planning_example src/grasp_planning_example.cpp)
add_executable(grasp_planning_with_clustering src/grasp_planning_with_clustering.cpp)
add_executable(border_following_example src/border_following_example.cpp)
add_executable(border_following_full_example src/border_following_full_example.cpp)
add_executable(concave_hull_example src/concave_hull_example.cpp)
add_executable(cloud_merger_example src/cloud_merger_example.cpp)
add_executable(online_cloud_merger src/online_cloud_merger.cpp)
add_executable(pose_estimation_example src/pose_estimation_example.cpp)
add_executable(clustering_example src/clustering_example.cpp)
add_executable(process_cloud src/process_cloud.cpp)
add_executable(remove_nan src/remove_nan.cpp)
add_executable(hobbit_grasping src/hobbit_grasping.cpp)
add_executable(particlef_template_tracking_tutorial src/particlef_template_tracking_tutorial.cpp)
add_executable(tutorial src/tutorial.cpp)

#for MERBOTS MANIPULATION
add_executable(grasp_testing src/grasp_testing.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(grasp_planning_example ${catkin_LIBRARIES} )
target_link_libraries(grasp_planning_with_clustering ${catkin_LIBRARIES} )
target_link_libraries(border_following_example ${catkin_LIBRARIES} )
target_link_libraries(border_following_full_example ${catkin_LIBRARIES} )
target_link_libraries(concave_hull_example ${catkin_LIBRARIES} )
target_link_libraries(cloud_merger_example ${catkin_LIBRARIES} )
target_link_libraries(online_cloud_merger ${catkin_LIBRARIES} )
target_link_libraries(pose_estimation_example ${catkin_LIBRARIES} )
target_link_libraries(clustering_example ${catkin_LIBRARIES} )
target_link_libraries(process_cloud ${catkin_LIBRARIES} )
target_link_libraries(remove_nan ${catkin_LIBRARIES} )
target_link_libraries(hobbit_grasping ${catkin_LIBRARIES} )
target_link_libraries(particlef_template_tracking_tutorial ${catkin_LIBRARIES} )
target_link_libraries(tutorial ${catkin_LIBRARIES} )


target_link_libraries(grasp_testing ${catkin_LIBRARIES} )

