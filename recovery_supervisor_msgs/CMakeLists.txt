cmake_minimum_required(VERSION 2.8.3)
project(recovery_supervisor_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
)


add_message_files(FILES
  PosVelTimeGoalDemo.msg
  PosVelTimeGoalFeature.msg
  XYThetaDemo.msg
  XYThetaFeature.msg
  XYThetaGoalDemo.msg
  XYThetaGoalFeature.msg
)
generate_messages(DEPENDENCIES nav_msgs)

catkin_package()
