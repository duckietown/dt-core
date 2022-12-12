#!/bin/bash

set -e

# download PyTorch
echo "Downloading PyTorch v${PYTORCH_VERSION}..."
PYTORCH_WHEEL_NAME="torch-${PYTORCH_VERSION}.cuda.cudnn-cp38-cp38-linux_aarch64.whl"
WHEEL_URL="https://duckietown-public-storage.s3.amazonaws.com/assets/python/wheels/${PYTORCH_WHEEL_NAME}"
wget -q "${WHEEL_URL}" -O "/tmp/${PYTORCH_WHEEL_NAME}"
# install PyTorch
echo "Installing PyTorch v${PYTORCH_VERSION}..."
pip3 install "/tmp/${PYTORCH_WHEEL_NAME}"
rm "/tmp/${PYTORCH_WHEEL_NAME}"

# clean
pip3 uninstall -y dataclasses