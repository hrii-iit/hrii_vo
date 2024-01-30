#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
import numpy as np
import socket
import time

# global img
# global img_l
# global info
# global info_l
global bridge
bridge = CvBridge()


def main(data):
    global bridge 
    global img
    
    img = data

    # cv2.imshow("img", img)
    # cv2.waitKey(1)
# def main(data):
def main_a(data):
    global bridge 
    global img_l
    img_l = data

def main_b(data):
    global bridge
    global info 
    data.K = [259.04710805107067, 0.0, 316.47369032032486, 0.0, 258.2476705396247, 178.9590252855541, 0.0, 0.0, 1.0]
    data.D = [-0.03460961458058812, -0.001751389429465262, 0.00043587560582904254, 0.002749400793757686, 0.0]
    data.R = [0.9999943172320572, 0.002214213727912217, 0.0025421961290855487, -0.0022199569110863265, 0.9999949853642897, 0.0022585476721046784, -0.0025371824736373392, -0.0022641784031684708, 0.9999942180839118]
    data.P = [254.79454822788438, 0.0, 318.9628486633301, 0.0, 0.0, 254.79454822788438, 178.73063278198242, 0.0, 0.0, 0.0, 1.0, 0.0]
    #print(data)
    info = data

def main_c(data):
    global info_l
    global info
    global img
    global img_l

    data.K = [261.18376727322317, 0.0, 325.05181526605537, 0.0, 260.8700524830581, 177.46786393679642, 0.0, 0.0, 1.0]
    data.R = [0.9998453520686051, 0.0022077125736069047, 0.01744700409810311, -0.002168251336591839, 0.999995049287394, -0.00228036989186367, -0.01745195212428273, 0.0022421877474218433, 0.9998451889973552]
    data.D = [-0.03460961458058812, -0.001751389429465262, 0.00043587560582904254, 0.002749400793757686, 0.0]
    data.P =[254.79454822788438, 0.0, 318.9628486633301, 28.43647592955959, 0.0, 254.79454822788438, 178.73063278198242, 0.0, 0.0, 0.0, 1.0, 0.0]
    info_l = data
    pub_im.publish(img)
    pub_iml.publish(img_l)
    pubinf.publish(info)
    pubinfl.publish(info_l)
    # print(img)
    

if __name__ == '__main__':
    rospy.init_node("camera_info")
 
    pub_im = rospy.Publisher('/right/image_raw', Image, queue_size=10)
    pub_iml = rospy.Publisher('/left/image_raw', Image, queue_size=10)
    pubinf = rospy.Publisher('/right/camera_info', CameraInfo, queue_size=10)
    pubinfl = rospy.Publisher('/left/camera_info', CameraInfo, queue_size=10)

    rospy.Subscriber("/zed2i/zed_node/right_raw/image_raw_color", Image, callback=main)
    rospy.Subscriber("/zed2i/zed_node/left_raw/image_raw_color", Image, callback=main_a)
    rospy.Subscriber("/zed2i/zed_node/left_raw/camera_info", CameraInfo, callback=main_b)
    rospy.Subscriber("/zed2i/zed_node/right_raw/camera_info", CameraInfo, callback=main_c)
    rospy.spin()