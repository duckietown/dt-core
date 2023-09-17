#!/usr/bin/env bash

set -e

# remove existing OpenCV
pip3 uninstall -y opencv-python opencv-contrib-python

# install GPU-enabled OpenCV
wget -P /tmp https://duckietown-public-storage.s3.amazonaws.com/assets/opencv-cuda/opencv-4.5.0-cuda-10.2.tar.gz
tar -xzvf /tmp/opencv-4.5.0-cuda-10.2.tar.gz -C /usr/local
rm /tmp/opencv-4.5.0-cuda-10.2.tar.gz

# numpy needs to be pinned to avoid this: https://staging-ci.duckietown.org/view/Wall%20-%20daffy-staging/job/Docker%20Autobuild%20-%20daffy-staging%20-%20dt-core%20-%20arm64v8/43/consoleFull
pip3 install "numpy<=1.20.0"
