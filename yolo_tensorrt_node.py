import rospy
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import onnxruntime

bridge = CvBridge()
parser = argparse.ArgumentParser()
parser.add_argument('--image_topic', required=True)
parser.add_argument('--output_topic', required=True)
args = parser.parse_args()

engine_path = "/workspace/engine_cache/best.engine"
use_trt = False

try:
    import tensorrt as trt
    import pycuda.driver as cuda
    import pycuda.autoinit

    TRT_LOGGER = trt.Logger(trt.Logger.INFO)
    with open(engine_path, "rb") as f:
        runtime = trt.Runtime(TRT_LOGGER)
        engine = runtime.deserialize_cuda_engine(f.read())
    context = engine.create_execution_context()
    input_shape = engine.get_binding_shape(0)
    input_size = tuple(input_shape[-2:])  # (H, W)
    d_input = cuda.mem_alloc(trt.volume(input_shape) * np.float32().itemsize)
    d_output = cuda.mem_alloc(1000000)  # Allocate 1MB for output
    bindings = [int(d_input), int(d_output)]
    output = np.empty([1000, 6], dtype=np.float32)
    use_trt = True
    print("Running inference using TensorRT.")
except Exception as e:
    print("TensorRT not available:", e)
    session = onnxruntime.InferenceSession("best.onnx", providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    input_size = (640, 640)
    print("Running inference using ONNXRuntime (CPU fallback).")

pub = rospy.Publisher(args.output_topic, Image, queue_size=1)

def draw_boxes(frame, detections):
    for det in detections:
        x1, y1, x2, y2, conf, cls = det
        if conf < 0.4:
            continue
        p1 = (int(x1), int(y1))
        p2 = (int(x2), int(y2))
        label = f"{int(cls)} {conf:.2f}"
        cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
        cv2.putText(frame, label, (p1[0], p1[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return frame

def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except:
        return

    resized = cv2.resize(frame, input_size)
    img = resized.astype(np.float32).transpose(2, 0, 1)[np.newaxis] / 255.0
    img = np.ascontiguousarray(img)

    if use_trt:
        cuda.memcpy_htod(d_input, img)
        context.execute_v2(bindings)
        cuda.memcpy_dtoh(output, d_output)
        detections = output[output[:, 4] > 0.5]  # filter conf
    else:
        detections = session.run(None, {input_name: img})[0]
        detections = detections[0][detections[0][:, 4] > 0.4]

    annotated = draw_boxes(resized.copy(), detections)
    pub.publish(bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

if __name__ == '__main__':
    rospy.init_node("yolo_tensorrt_node")
    rospy.Subscriber(args.image_topic, Image, image_callback, queue_size=1)
    rospy.spin()

