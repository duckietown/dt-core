cmake_minimum_required(VERSION 2.8.3)
project(duckietown_visualization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  visualization_msgs
)

catkin_python_setup()

catkin_package()

