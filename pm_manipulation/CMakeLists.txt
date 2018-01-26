cmake_minimum_required(VERSION 2.8.3)
project(pm_manipulation)

find_package(catkin REQUIRED COMPONENTS roscpp tf mar_robot_arm5e pm_tools pm_perception nav_msgs geometry_msgs std_msgs interactive_markers)

find_package(PCL)
find_package(VISP)
find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pm_manipulation
  CATKIN_DEPENDS roscpp tf mar_robot_arm5e pm_tools pm_perception nav_msgs geometry_msgs
  DEPENDS PCL VISP OpenCV
)
#link_directories(${catkin_LIBRARY_DIRS})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})


add_library(pm_manipulation src/slider_grasp_planner.cpp)
target_link_libraries(pm_manipulation ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${VISP_LIBRARIES})

#Old example: add_executable(planning_and_visualization_simple src/planning_and_visualization_simple.cpp)
add_executable(interactive_marker_test src/interactive_marker_test.cpp)
add_executable(slider_grasp_server src/slider_grasp_server.cpp src/slider_grasp_planner.cpp)
add_executable(waypoint_server src/waypoint_server.cpp)
add_executable(fake_pose_publisher src/fake_pose_publisher.cpp)
add_executable(object_pose_from_marker_pose src/object_pose_from_marker_pose.cpp)

#Old example: target_link_libraries(grasp_vis ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(interactive_marker_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(slider_grasp_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(waypoint_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(fake_pose_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(object_pose_from_marker_pose ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )