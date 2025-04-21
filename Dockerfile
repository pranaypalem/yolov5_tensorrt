FROM nvidia/cuda:11.8.0-runtime-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

# --- Install system packages and ROS core ---
RUN apt update && apt install -y \
    curl gnupg2 lsb-release wget software-properties-common && \
    echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && apt install -y \
    ros-melodic-ros-core \
    ros-melodic-cv-bridge \
    ros-melodic-image-transport \
    libgl1-mesa-glx libglib2.0-0 \
    python3.8 python3.8-dev python3.8-distutils \
    python3-pip build-essential \
    && rm -rf /var/lib/apt/lists/*

# --- Set Python 3.8 as default ---
RUN ln -sf /usr/bin/python3.8 /usr/bin/python3 && \
    wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py && rm get-pip.py

# --- Install full CUDA Toolkit (includes cudaProfiler.h and headers for pycuda) ---
RUN apt update && apt install -y \
    cuda-toolkit-11-8 \
    && rm -rf /var/lib/apt/lists/*

# --- Install Python dependencies ---
COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# --- Set up application workspace ---
WORKDIR /workspace
COPY best.pt .
COPY convert_model.py entrypoint.sh yolo_tensorrt_node.py ./
RUN chmod +x /workspace/entrypoint.sh

ENTRYPOINT ["/workspace/entrypoint.sh"]

