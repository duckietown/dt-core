#!/bin/bash


tag=duckietown/dt-core:daffy-devel-AC-cleanup-amd64

docker run -it \
		$(tag) \
		bash -c "source /opt/ros/noetic/setup.bash; catkin build; source /code/catkin_ws/devel/local_setup.bash; make
		test-circle"


