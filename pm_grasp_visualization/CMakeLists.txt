cmake_minimum_required(VERSION 2.8.3)
project(pm_grasp_visualization)

find_package(catkin REQUIRED COMPONENTS  roscpp pm_manipulation pm_tools pm_perception mar_perception geometry_msgs tf)

find_package(PCL REQUIRED)
find_package(VISP REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pm_manipulation pm_tools pm_perception mar_perception geometry_msgs tf
  DEPENDS PCL VISP OpenCV
)
link_directories(${catkin_LIBRARY_DIRS})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

add_executable(grasp_vis src/grasp_vis.cpp)
target_link_libraries(grasp_vis ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )


