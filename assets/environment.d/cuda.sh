#!/usr/bin/env bash

# configure environment for CUDA
export CUDA_VERSION="10.2"
export CUDNN_VERSION="8.0"
export NVIDIA_REQUIRE_CUDA="cuda>=${CUDA_VERSION} brand=tesla,driver>=396,driver<397 brand=tesla,driver>=410,driver<411 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441"
