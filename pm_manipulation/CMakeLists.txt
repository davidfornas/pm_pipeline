cmake_minimum_required(VERSION 2.8.3)
project(pm_manipulation)

find_package(catkin REQUIRED COMPONENTS roscpp tf mar_robot_arm5e mar_perception mar_ros_bridge pm_tools
        pm_perception nav_msgs geometry_msgs std_msgs interactive_markers)

find_package(PCL)
find_package(VISP)
find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pm_manipulation
  CATKIN_DEPENDS roscpp tf mar_robot_arm5e  mar_perception mar_ros_bridge pm_tools pm_perception nav_msgs geometry_msgs
  DEPENDS PCL VISP OpenCV
)
#link_directories(${catkin_LIBRARY_DIRS})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(pm_manipulation src/slider_grasp_planner.cpp)
target_link_libraries(pm_manipulation ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${VISP_LIBRARIES})

add_executable(slider_grasp_planner_server src/slider_grasp_planner_server.cpp src/slider_grasp_planner.cpp)
add_executable(waypoint_server src/waypoint_server.cpp)
add_executable(publish_pose src/publish_pose.cpp)
add_executable(object_pose_from_marker_pose src/object_pose_from_marker_pose.cpp)
add_executable(joint_offset_test src/joint_offset_test.cpp src/joint_offset.cpp)
add_executable(set_joint_position src/set_joint_position.cpp)
add_executable(arm_server src/arm_server.cpp src/joint_offset.cpp)

target_link_libraries(slider_grasp_planner_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(waypoint_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(publish_pose ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(object_pose_from_marker_pose ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(joint_offset_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(set_joint_position ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(arm_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES})