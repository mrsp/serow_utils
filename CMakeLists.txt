cmake_minimum_required(VERSION 2.8.3)
project(serow_utils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  laser_geometry
)

catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs nav_msgs
  roscpp
  std_msgs
  tf
  laser_geometry
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(serow_utils  src/serow_utils_driver.cpp src/serow_utils.cpp)
target_link_libraries(serow_utils ${catkin_LIBRARIES})