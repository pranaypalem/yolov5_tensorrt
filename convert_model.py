import os
import subprocess
from ultralytics import YOLO

engine_dir = "/workspace/engine_cache"
os.makedirs(engine_dir, exist_ok=True)
engine_path = os.path.join(engine_dir, "best.engine")

if os.path.exists(engine_path):
    print("TensorRT engine already exists.")
    exit(0)

print("Exporting best.pt to ONNX...")
model = YOLO("best.pt")
model.export(format="onnx", dynamic=True, simplify=True)
print("Export complete.")

print("Converting to TensorRT...")
subprocess.run([
    "trtexec",
    "--onnx=best.onnx",
    f"--saveEngine={engine_path}",
    "--fp16",
    "--workspace=2048"
], check=True)
print("TensorRT engine saved to:", engine_path)
