# YOLOv5 + TensorRT ROS Inference (Dockerized)

This repository contains a minimal and production-ready Docker environment to run **YOLOv5 inference** in real-time using **TensorRT**, integrated with **ROS (Melodic)** for robotics workflows. Built for Jetson or remote GPU inference, with CPU fallback.

---

## 🚀 Docker Image on Docker Hub

📦 [DockerHub: pranaypalem/yolov5_tensorrt](https://hub.docker.com/r/pranaypalem/yolov5_tensorrt)

```bash
docker pull pranaypalem/yolov5_tensorrt:1.0.0
```

---

## 🔧 Features

- ✅ ROS Melodic (minimal base)
- ✅ Inference via TensorRT engine (or ONNXRuntime as fallback)
- ✅ Subscribes to image topic from ROS
- ✅ Publishes annotated image with YOLO detections
- ✅ GPU + CPU fallback support
- ✅ Caches `.engine` file for fast reuse

---

## 🛠️ Requirements

- Docker with NVIDIA GPU support
- ROS master running on your Jetson (or another device)
- Compatible YOLOv5 `best.pt` file

---

## 🧪 Run Inference (on laptop using Jetson camera)

```bash
docker run -it --net=host --gpus all \
  -v ~/yolo_engine_cache:/workspace/engine_cache \
  -e ROS_MASTER_URI=http://<JETSON_IP>:11311 \
  -e ROS_IP=<YOUR_LAPTOP_IP> \
  -e IMAGE_TOPIC=/camera/rgb/image_raw \
  -e OUTPUT_TOPIC=/yolo/annotated \
  pranaypalem/yolov5_tensorrt:1.0.0
```

> Replace `<JETSON_IP>` and `<YOUR_LAPTOP_IP>` with your actual device IPs.

---

## 📁 Folder Structure

```
.
├── Dockerfile
├── build.sh
├── entrypoint.sh
├── requirements.txt
├── convert_model.py
├── yolo_tensorrt_node.py
├── best.pt                # Your YOLOv5 model file
└── README.md
```

---

## 📂 Environment Variables

| Variable        | Description                          |
|------------------|--------------------------------------|
| `IMAGE_TOPIC`    | Input image topic from ROS camera    |
| `OUTPUT_TOPIC`   | Annotated image output topic         |
| `ROS_MASTER_URI` | IP of Jetson or ROS master           |
| `ROS_IP`         | Local IP to advertise to ROS         |

---

## 🧠 Internals

- Converts `best.pt → best.onnx → best.engine`
- Runs YOLOv5 inference using TensorRT (if available) or ONNX
- Publishes overlayed image with bounding boxes

---

## 🧑‍💻 Author

**Pranay Palem**  
Built for real-time robotics, CV pipelines, and Jetson deployment.

---

## 📜 License

MIT License — use freely and responsibly.
