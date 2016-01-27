cmake_minimum_required(VERSION 2.8.3)
project(pm_registration)

find_package(catkin REQUIRED COMPONENTS roscpp )
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pm_measure
  pm_tools
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pm_registration
   CATKIN_DEPENDS pm_measure pm_tools
#  DEPENDS system_lib
)


add_executable(registration_example src/registration_example.cpp)
target_link_libraries(registration_example ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )
add_executable(incremental_registration_example src/incremental_registration_example.cpp)
target_link_libraries(incremental_registration_example ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VISP_LIBRARIES} )


# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
