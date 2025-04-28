#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloROSNode:
    def __init__(self):
        rospy.init_node('yolo_ros_inference', anonymous=True)
        self.bridge = CvBridge()

        # Load YOLOv8 TensorRT model
        self.model = YOLO('/workspace/bestsynthetic.engine') 

        # ROS topics
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback, queue_size=1)
        self.annotated_pub = rospy.Publisher('/yolo/annotated_image', Image, queue_size=1)
        self.bbox_pub = rospy.Publisher('/yolo/bboxes', String, queue_size=1)

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        # Run TensorRT inference
        results = self.model(img)

        # Annotated image
        annotated = results[0].plot()

        # Convert to ROS Image and publish
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        # Publish bounding boxes
        bboxes = []
        if results[0].boxes is not None:
            for det in results[0].boxes.xywh.cpu().numpy():
                x, y, w, h = map(int, det)
                bboxes.append(f"{x},{y},{w},{h}")
            bbox_msg = String(data=';'.join(bboxes))
            self.bbox_pub.publish(bbox_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = YoloROSNode()
    node.run()

