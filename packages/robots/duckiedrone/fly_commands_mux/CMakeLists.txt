cmake_minimum_required(VERSION 2.8.3)
project(fly_commands_mux)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  sensor_msgs
  roscpp
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
