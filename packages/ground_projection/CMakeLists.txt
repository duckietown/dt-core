cmake_minimum_required(VERSION 2.8.3)
project(ground_projection)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  duckietown_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  image_geometry
  )

catkin_python_setup()

#add_service_files(
#  FILES
#  GetGroundCoord.srv
#  GetImageCoord.srv
#  EstimateHomography.srv
#)
#
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#   geometry_msgs
#   duckietown_msgs
# )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
