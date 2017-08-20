cmake_minimum_required(VERSION 2.8.3)
project(line_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs # Every duckietown packages should use this.
  cv_bridge
  # XXX add anti_instagram?
)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_detector
#  CATKIN_DEPENDS duckietown_msgs roscpp rospy
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
