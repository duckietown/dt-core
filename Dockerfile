ARG ARCH=arm32v7
ARG MAJOR=devel20
ARG BASE_TAG=${MAJOR}-${ARCH}

FROM duckietown/dt-ros-commons:${BASE_TAG}

ENV REPO_PATH="${CATKIN_WS_DIR}/src/dt-core"
WORKDIR "${REPO_PATH}"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

# copy entire repo
COPY . "${REPO_PATH}/"

RUN pip install -r ${REPO_PATH}/requirements.txt

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
  --workspace ${CATKIN_WS_DIR}/

LABEL maintainer="Andrea F. Daniele (afdaniele@ttic.edu)"
