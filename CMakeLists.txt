cmake_minimum_required(VERSION 2.8.3)
project(recovery_supervisor)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  map_msgs
  nav_msgs
  rosbag_storage
  roscpp
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES recovery_supervisor
  CATKIN_DEPENDS actionlib_msgs
    geometry_msgs
    map_msgs
    nav_msgs
    rosbag_storage
    roscpp
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(recovery_supervisor
  src/recovery_supervisor.cpp)

target_link_libraries(recovery_supervisor
  ${catkin_LIBRARIES}
)

