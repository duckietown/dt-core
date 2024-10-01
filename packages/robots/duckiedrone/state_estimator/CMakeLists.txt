cmake_minimum_required(VERSION 2.8.3)
project(state_estimator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  duckietown_msgs
  message_generation
)


add_message_files(  
  FILES #todo either remove these or add them to duckietown_msgs
  StateGroundTruth.msg
  UkfStats.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
