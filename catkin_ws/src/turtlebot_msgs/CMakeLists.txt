cmake_minimum_required(VERSION 2.8.12)
project(turtlebot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  GoalPose.msg
  Trajectory.msg
  cntrl_acc.msg
  cntrl_vel.msg
  cntrl_lin_vel.msg
  cntrl_ang_vel.msg
  cntrl_pos.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
