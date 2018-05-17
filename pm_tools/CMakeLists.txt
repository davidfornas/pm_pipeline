cmake_minimum_required(VERSION 2.8.3)
project(pm_tools)

find_package(catkin REQUIRED COMPONENTS  roscpp sensor_msgs pcl_conversions geometry_msgs interactive_markers tf visualization_msgs underwater_sensor_msgs image_transport)

find_package(PCL REQUIRED)
find_package(VISP REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions geometry_msgs interactive_markers tf underwater_sensor_msgs sensor_msgs image_transport
  DEPENDS PCL VISP OpenCV
)
link_directories(${catkin_LIBRARY_DIRS})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/pcl_tools.cpp
  src/pcl_segmentation.cpp
  src/pcl_merge.cpp
  src/pcl_clustering.cpp
  src/visp_tools.cpp
  src/virtual_image.cpp
  src/marker_tools.cpp
  src/tf_tools.cpp
  src/timing.cpp
        src/logger.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
