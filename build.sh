#!/bin/bash

export DOCKER_BUILDKIT=1

docker build \
  --progress=plain \
  --tag yolo-tensorrt-ros-super-slim \
  -f Dockerfile .
