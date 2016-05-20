cmake_minimum_required(VERSION 2.8.3)
project(pm_perception)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs pm_tools)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(VISP REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs pm_tools
  DEPENDS PCL Boost VISP OpenCV
)
link_directories(${catkin_LIBRARY_DIRS})
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME} 
  src/background_removal.cpp
  src/object_segmentation.cpp
  src/border_detection.cpp
  src/pose_estimation.cpp
  src/cluster_measure.cpp
)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} ${OpenCV})

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})