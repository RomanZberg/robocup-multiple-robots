#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Camera(Node):

    def __init__(self):
        super().__init__("webcam")
        self.camera_pub = self.create_publisher(Image,"/image_raw",1)
        self.timer = self.create_timer(0.1,self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()
        self.get_logger().info("Camera Publisher has been started")
    
    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.camera_pub.publish(self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))

        self.get_logger().info('Publishing video frame')