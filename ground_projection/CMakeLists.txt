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
if ("$ENV{ROS_DISTRO}" STREQUAL "indigo")
  find_package(OpenCV 2.4.8 REQUIRED)
elseif ("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
  find_package(OpenCV 3 REQUIRED)
else()
  message(FATAL_ERROR "ROS distribution  not supported $ENV{ROS_DISTRO}")
endif()
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig)

# libyaml-cpp-dev
if (ANDROID)
  find_package(yaml-cpp)
  add_definitions(-DHAVE_NEW_YAMLCPP)
else()
  pkg_check_modules(YAML_CPP yaml-cpp)
  if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
    add_definitions(-DHAVE_NEW_YAMLCPP)
  endif()
  link_directories(${YAML_CPP_LIBRARY_DIRS})
endif()
include_directories(${YAML_CPP_INCLUDE_DIRS})




## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetGroundCoord.srv
  GetImageCoord.srv
  EstimateHomography.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  duckietown_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ground_projection
  CATKIN_DEPENDS image_transport roscpp rospy duckietown_msgs sensor_msgs std_msgs geometry_msgs message_runtime cv_bridge image_geometry
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



## Declare a C++ executable
add_executable(ground_projection
  src/ground_projection.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ground_projection ${ground_projection_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ground_projection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)
