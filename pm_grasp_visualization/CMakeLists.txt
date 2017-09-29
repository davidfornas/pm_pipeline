cmake_minimum_required(VERSION 2.8.3)
project(pm_grasp_visualization)

find_package(catkin REQUIRED COMPONENTS  roscpp pm_manipulation pm_tools pm_perception geometry_msgs std_msgs interactive_markers tf)

find_package(PCL REQUIRED)
find_package(VISP REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pm_manipulation pm_tools pm_perception geometry_msgs std_msgs interactive_markers tf
  DEPENDS PCL VISP OpenCV
)
link_directories(${catkin_LIBRARY_DIRS})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

#Old example: add_executable(planning_and_visualization_simple src/planning_and_visualization_simple.cpp)
add_executable(interactive_marker_test src/interactive_marker_test.cpp)
add_executable(slider_grasp_server src/slider_grasp_server.cpp)
add_executable(waypoint_server src/waypoint_server.cpp)
add_executable(fake_pose_publisher src/fake_pose_publisher.cpp)

#Old example: target_link_libraries(grasp_vis ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(interactive_marker_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(slider_grasp_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(waypoint_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(fake_pose_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )


