cmake_minimum_required(VERSION 2.8.3)
project(pm_registration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pm_measure
  pm_tools
)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pm_registration
   CATKIN_DEPENDS roscpp pm_measure pm_tools
   DEPENDS PCL
)
link_directories(${catkin_LIBRARY_DIRS})
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
MESSAGE( STATUS "Hi!          " ${catkin_LIBRARIES} )

add_executable(registration_example src/registration_example.cpp)
target_link_libraries(registration_example /home/david/ros_ws/devel/lib/libpm_tools.so /home/david/ros_ws/devel/lib/libpm_perception.so ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(incremental_registration_example src/incremental_registration_example.cpp)
target_link_libraries(incremental_registration_example ${catkin_LIBRARIES} ${PCL_LIBRARIES})


