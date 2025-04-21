#!/bin/bash
source /opt/ros/melodic/setup.bash
python3 yolo_tensorrt_node.py --image_topic $IMAGE_TOPIC --output_topic $OUTPUT_TOPIC
