from ultralytics import YOLO

# Load the YOLO model
model = YOLO('/workspace/bestsynthetic.pt')

# Export directly to TensorRT engine ON GPU (device 0)
model.export(format='engine', device=0)

print("✅ Successfully exported bestsynthetic.pt to TensorRT engine on GPU!")

