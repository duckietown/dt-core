ARG ARCH=arm32v7
ARG MAJOR=devel20
ARG BASE_TAG=${MAJOR}-${ARCH}

FROM duckietown/dt-ros-commons:${BASE_TAG}

RUN apt-get update && apt-get install -y ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-image-geometry ros-kinetic-rospy python-sklearn python-frozendict libyaml-cpp-dev && apt-get clean


ARG REPO_PATH="${CATKIN_WS_DIR}/src/dt-core"
WORKDIR "${REPO_PATH}"

# create repo directory
RUN mkdir -p "${REPO_PATH}"
COPY requirements.txt ${REPO_PATH}/requirements.txt

RUN pip install -r ${REPO_PATH}/requirements.txt

# copy entire repo
COPY . "${REPO_PATH}/"

ENV DUCKIEFLEET_ROOT "/data/config"


# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
  --workspace ${CATKIN_WS_DIR}/

LABEL maintainer="Andrea F. Daniele (afdaniele@ttic.edu)"
