#!/bin/bash

# Ensure that the hostnames and ip address are setup in /etc/hosts
echo "${ROS_MASTER_IP} ${ROS_MASTER}" >> /etc/hosts

export ROS_MASTER_URI=http://${ROS_MASTER}:11311

# Start the diagnostics
roslaunch duckietown_visualization publish_map.launch rviz_config:=/catkin-ws/cSLAM.rviz
