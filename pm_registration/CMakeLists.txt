cmake_minimum_required(VERSION 2.8.3)
project(pm_registration)

find_package(catkin REQUIRED COMPONENTS  roscpp pm_tools)
find_package(PCL REQUIRED)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pm_registration
   CATKIN_DEPENDS roscpp pm_tools
   DEPENDS PCL
)
link_directories(${catkin_LIBRARY_DIRS})
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ar_registration src/ar_registration.cpp src/marker_registration.cpp)
target_link_libraries(ar_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES})


