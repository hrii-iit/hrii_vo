#! /usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer

#global img


def only_image(msg):
    #global img
    bridge = CvBridge()
    img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # cv2.imshow('color', img)
    # cv2.waitKey(1)
    img_a = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    
    img_a.header = msg.header 
    pub_im.publish(img_a)


def only_depth(msg):
    bridge = CvBridge()
    #global img
    
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    depth_array = np.array(depth_image, dtype=np.float32)
    img_a = bridge.cv2_to_imgmsg(depth_array)
    
    img_a.header = msg.header 
    pub_im.publish(img_a)

    # cv2.imshow('depth', depth_array)
    # cv2.imshow('color', img)
    # cv2.waitKey(1)



def image_callback(image, depth):
    global img
    #print("Hi")
    # convert compressed image data to uncompressed image format
    bridge = CvBridge()
    img = bridge.compressed_imgmsg_to_cv2(image, desired_encoding='bgr8')
    depth_image = bridge.imgmsg_to_cv2(depth, desired_encoding='16UC1')
    depth_array = np.array(depth_image, dtype=np.float32)
    try:
        # color_array = np.asarray(color_image)
        cv2.imshow('depth', depth_array)
        cv2.imshow('color', img)
        cv2.waitKey(1)
    except:
        print("Error")
    #print(img.shape)

    # display the image in a new window
    #cv2.imshow('Compressed Image', img)
    #cv2.waitKey(1)

def callback_c(msg):
    global img
    bridge = CvBridge()
    global depth_array
    clipping_distance_in_meters = 1 #1 meter
    depth_scale = 0.0010000000474974513

    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    depth_array = np.array(depth_image, dtype=np.float32)
    try:
        # color_array = np.asarray(color_image)
        cv2.imshow('depth', depth_array)
        cv2.imshow('color', img)
        cv2.waitKey(1)
    except:
        print("Error")



if __name__ == '__main__':
    rospy.init_node('image_viewer')

    rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, only_image)
    # rospy.Subscriber('/camera/color/camera_info', CameraInfo, Camera_info)
    #rospy.Subscriber('/camera/depth/image_rect_raw', Image, only_depth)
    

    pub_im = rospy.Publisher('/cam/color/image_raw', Image, queue_size=10)
    #pub_depth = rospy.Publisher('/cam/depth/image_rect_raw', Image, queue_size=10)
    # image_sub = Subscriber('/camera/color/image_raw/compressed', CompressedImage)
    # depth_sub = Subscriber("/camera/depth/image_rect_raw", Image)
    # ts = ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=10, slop=0.1)
    # ts.registerCallback(image_callback)
    #rospy.Subscriber('/camera/color/image_rect_raw/compressedDepth',CompressedImage, image_callback)
    print("Hi")
    rospy.spin()
