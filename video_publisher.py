#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    rospy.init_node('video_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
    bridge = CvBridge()
    
    cap = cv2.VideoCapture('/workspace/test_video.avi')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    main()

