#!/bin/bash

set -eux

# download PyTorch
echo "Downloading PyTorch v${PYTORCH_VERSION}..."
PYTORCH_WHEEL_NAME="torch-${PYTORCH_VERSION}.cuda.cudnn-cp38-cp38-linux_aarch64.whl"
WHEEL_URL="https://duckietown-public-storage.s3.amazonaws.com/assets/python/wheels/${PYTORCH_WHEEL_NAME}"
wget -q "${WHEEL_URL}" -O "/tmp/${PYTORCH_WHEEL_NAME}"
# install PyTorch
echo "Installing PyTorch v${PYTORCH_VERSION}..."
pip3 install "/tmp/${PYTORCH_WHEEL_NAME}"
rm "/tmp/${PYTORCH_WHEEL_NAME}"

# download TorchVision
echo "Downloading TorchVision v${TORCHVISION_VERSION}..."
TORCHVISION_WHEEL_NAME="torchvision-${TORCHVISION_VERSION}-cp38-cp38-linux_aarch64.whl"
WHEEL_URL="https://duckietown-public-storage.s3.amazonaws.com/assets/python/wheels/${TORCHVISION_WHEEL_NAME}"
wget -q "${WHEEL_URL}" -O "/tmp/${TORCHVISION_WHEEL_NAME}"
# install TorchVision
echo "Installing TorchVision v${TORCHVISION_VERSION}..."
pip3 install "/tmp/${TORCHVISION_WHEEL_NAME}"
rm "/tmp/${TORCHVISION_WHEEL_NAME}"

# clean
pip3 uninstall -y dataclasses