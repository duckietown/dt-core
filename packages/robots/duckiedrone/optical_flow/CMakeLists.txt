cmake_minimum_required(VERSION 2.8.3)
project(optical_flow)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  duckietown_msgs
  roscpp
  rospy
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/optical_flow.test)
endif()


catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
