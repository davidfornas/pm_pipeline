cmake_minimum_required(VERSION 2.8.3)
project(pm_tools)

find_package(catkin REQUIRED COMPONENTS  roscpp sensor_msgs pcl_conversions tf visualization_msgs sensor_msgs image_transport)

find_package(PCL REQUIRED)
find_package(VISP REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pm_tools
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions tf visualization_msgs sensor_msgs image_transport
  DEPENDS PCL VISP OpenCV
)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS}
)

add_library(pm_tools 
  src/pcl_tools.cpp
  src/pcl_segmentation.cpp
  src/pcl_merge.cpp
  src/pcl_clustering.cpp
  src/visp_tools.cpp
  src/virtual_image.cpp
)
target_link_libraries(pm_tools ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )

add_executable(registration_example src/registration_example.cpp)
target_link_libraries(registration_example pm_tools ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
#add_dependencies(conver ${arm5_controller_EXPORTED_TARGETS})

add_executable(convert src/conversions.cpp)
target_link_libraries(convert ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
