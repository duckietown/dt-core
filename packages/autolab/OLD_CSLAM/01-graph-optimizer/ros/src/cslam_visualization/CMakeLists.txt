cmake_minimum_required(VERSION 2.8.3)
project(cslam_visualization)

find_package(catkin REQUIRED COMPONENTS
  duckietown_visualization
  pose_graph_builder
  rospy
)

catkin_python_setup()

catkin_package()