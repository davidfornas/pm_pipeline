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

add_executable(grasp_vis src/grasp_vis.cpp)
add_executable(grasp_marker src/grasp_marker.cpp)
add_executable(grasp_server src/grasp_server.cpp)

target_link_libraries(grasp_vis ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(grasp_marker ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
target_link_libraries(grasp_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )


