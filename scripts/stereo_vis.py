#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
import numpy as np
import socket
import time


def call_back_stereo(msg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #img_a = bridge.cv2_to_imgmsg(img, encoding='bgr8')

    cv2.imshow('img', img)
    print(img.shape)
    cv2.waitKey(1)


rospy.init_node('visualizer')
rospy.Subscriber('/stereo/left/image_rect_color',Image, call_back_stereo)
rospy.spin()