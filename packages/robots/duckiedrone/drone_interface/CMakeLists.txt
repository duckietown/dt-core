cmake_minimum_required(VERSION 2.8.3)
project(drone_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

