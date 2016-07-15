cmake_minimum_required(VERSION 2.8.3)
project(recovery_supervisor)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  rosbag_storage
  roscpp
  rviz
  sensor_msgs
  std_msgs
  tf
  tf2_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES recovery_supervisor points_to_path
  CATKIN_DEPENDS actionlib_msgs
    geometry_msgs
    map_msgs
    move_base_msgs
    nav_msgs
    rosbag_storage
    roscpp
    rviz
    sensor_msgs
    tf
    tf2_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(points_to_path
  src/points_to_path.cpp)

add_executable(recovery_supervisor
  src/recovery_supervisor.cpp)

target_link_libraries(recovery_supervisor
  ${catkin_LIBRARIES}
)

target_link_libraries(points_to_path
  ${catkin_LIBRARIES}
)

add_dependencies(recovery_supervisor ${catkin_EXPORTED_TARGETS})
