cmake_minimum_required(VERSION 2.8.3)
project(recovery_supervisor_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs
)


add_message_files(FILES Feature.msg Demo.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(
)
