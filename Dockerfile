# ------------------------------------------------------
# BASE IMAGE
# ------------------------------------------------------
FROM nvidia/cuda:11.8.0-cudnn8-runtime-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------
# SYSTEM DEPENDENCIES
# ------------------------------------------------------
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    git \
    nano \
    curl \
    wget \
    lsb-release

# ------------------------------------------------------
# ADD ROS Noetic APT SOURCES
# ------------------------------------------------------
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# ------------------------------------------------------
# INSTALL ROS PACKAGES
# ------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    ros-noetic-rqt-image-view \
    ros-noetic-tf && \
    rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------
# PYTHON PACKAGES
# ------------------------------------------------------
RUN pip3 install --upgrade pip
RUN pip3 install ultralytics opencv-python onnx onnx-simplifier onnxruntime-gpu

# ------------------------------------------------------
# WORKSPACE SETUP
# ------------------------------------------------------
WORKDIR /workspace

# Copy all required files into the container
COPY bestsynthetic.pt /workspace/bestsynthetic.pt
COPY yolo_ros_inference.py /workspace/yolo_ros_inference.py
COPY coke_can_depth_processor.py /workspace/coke_can_depth_processor.py
COPY video_publisher.py /workspace/video_publisher.py
COPY test_video.avi /workspace/test_video.avi
COPY convert_to_engine.py /workspace/convert_to_engine.py
COPY TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz /workspace/

# ------------------------------------------------------
# INSTALL TensorRT
# ------------------------------------------------------
RUN tar -xvf TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz && \
    pip3 install TensorRT-8.6.1.6/python/tensorrt-8.6.1-cp38-none-linux_x86_64.whl

# ------------------------------------------------------
# ENVIRONMENT SETUP
# ------------------------------------------------------
# Add TensorRT libraries to LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH=/workspace/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib:${LD_LIBRARY_PATH}

# Automatically source ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# ------------------------------------------------------
# DEFAULT COMMAND
# ------------------------------------------------------
CMD ["bash"]

