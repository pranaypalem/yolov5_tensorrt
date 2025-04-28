# YOLOv8 + TensorRT ROS Inference (Dockerized)

This repository contains a minimal and production-ready Docker environment to run **YOLOv8 inference** in real-time using **TensorRT**, integrated with **ROS (Noetic)** for robotics workflows. Built for Jetson or remote GPU inference, with CPU fallback.

---

## 🚀 Docker Image on Docker Hub

📦 [DockerHub: pranaypalem/yolov8_tensorrt](https://hub.docker.com/r/pranaypalem/yolov8_tensorrt)

```bash
docker pull pranaypalem/yolov8_tensorrt:3.1.3
```

---

## 🔧 Features

- ✅ ROS Noetic (minimal setup)
- ✅ Inference via TensorRT engine (or ONNXRuntime as fallback)
- ✅ Subscribes to image topic from ROS
- ✅ Publishes annotated image with YOLO detections
- ✅ GPU + CPU fallback support
- ✅ Caches `.engine` file for fast reuse

---

## 🛠️ Requirements

- Docker with NVIDIA GPU support
- ROS master running (Jetson or another device)
- YOLOv8 `best.pt` model file
- **TensorRT tar.gz installer must be manually downloaded**:  
  > Download TensorRT from: [NVIDIA Developer TensorRT](https://developer.nvidia.com/tensorrt)
  >
  > Place `TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz` into the project directory before building or running.

---

## 🧪 How to Run

1. **Launch the Docker container:**

```bash
docker run -it --gpus all --net=host \
  -e ROS_MASTER_URI=http://172.20.10.4:11311 \
  -e ROS_IP=172.20.10.6 \
  pranaypalem/yolov8_tensorrt:3.1.3
```

2. **Convert the YOLOv8 model to TensorRT Engine:**

Inside the container:

```bash
python3 convert_to_engine.py
```

This will convert your `.pt` model to an optimized `.engine` file.

3. **Run the YOLO ROS inference node:**

```bash
python3 yolo_ros_inference.py
```

This script subscribes to a ROS image topic, runs YOLOv8 inference using TensorRT, and publishes the annotated image back.

---

## 📁 Folder Structure

```
.
├── Dockerfile
├── bestsynthetic.pt
├── coke_can_depth_processor.py
├── convert_to_engine.py
├── test_video.avi
├── video_publisher.py
├── yolo_ros_inference.py
├── TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz   # Must be manually downloaded
└── README.md
```

---

## 📂 Environment Variables

| Variable        | Description                          |
|------------------|--------------------------------------|
| `ROS_MASTER_URI` | IP address of the ROS master node    |
| `ROS_IP`         | Local machine IP to advertise to ROS |

---

## 🧠 Internals

- Converts `best.pt → best.onnx → best.engine`
- Runs YOLOv8 inference using TensorRT (or ONNXRuntime as fallback)
- Publishes annotated images back to ROS topics

---

## 🧑‍💻 Author

**Pranay Palem**  
Optimized for real-time robotics, computer vision pipelines, and GPU inference workflows.

---

## 📜 License

MIT License — use freely and responsibly.
