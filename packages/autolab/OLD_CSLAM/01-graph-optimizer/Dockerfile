FROM duckietown/dt-ros-commons:daffy-amd64




# DO NOT PUT ANYTHING HERE BEFORE THE G2O BUILDING! IT TAKES AGES!
ARG g2o_lib_dir=lib/src/g2opy
COPY ${g2o_lib_dir} /code/lib/src/g2opy

RUN cd /code/lib/src/g2opy ; mkdir build ; cd build ; cmake -D PYBIND11_PYTHON_VERSION=3.8 ..  ; make -j8; cd ..; python3 setup.py install

ARG graph_lib_dir=lib/
COPY ${graph_lib_dir} /code/lib/

RUN cd /code/lib; python3 setup.py develop --no-deps


## Pose graph optimizer
ARG ros_nodes=ros/src/pose_graph_builder
RUN mkdir -p /code/catkin_ws/src/pose_graph_builder
COPY ${ros_nodes} /code/catkin_ws/src/pose_graph_builder

# Setup the ros-visualization
COPY ros/src/cslam_visualization /code/catkin_ws/src/cslam_visualization
COPY ros/src/duckietown-visualization/duckietown_visualization /code/catkin_ws/src/duckietown_visualization
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin build \
    --workspace ${CATKIN_WS_DIR}/
RUN chmod +x /code/catkin_ws/src/pose_graph_builder/*.sh

ENV DUCKIETOWN_WORLD_FORK duckietown
ENV MAP_NAME robotarium2

CMD /bin/bash -c "cd ${CATKIN_WS_DIR}/src; git clone https://github.com/${DUCKIETOWN_WORLD_FORK}/duckietown-world.git ;source /opt/ros/noetic/setup.bash; /code/catkin_ws/src/pose_graph_builder/wrapper.sh"
