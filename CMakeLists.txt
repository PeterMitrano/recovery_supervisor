cmake_minimum_required(VERSION 2.8.3)
project(recovery_supervisor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES recovery_supervisor
  CATKIN_DEPENDS geometry_msgs roscpp
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

