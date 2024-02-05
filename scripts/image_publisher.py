#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
import numpy as np
import socket
import time

def main():
    rospy.init_node('image_publisher')
    pub = rospy.Publisher('/right/image_raw', Image, queue_size=10)
    pub_r = rospy.Publisher('/left/image_raw', Image, queue_size=10)
    pub_in = rospy.Publisher('/right/camera_info', CameraInfo, queue_size=10)
    pub_in_r = rospy.Publisher('/left/camera_info', CameraInfo, queue_size=10)
    camera_info_r  = CameraInfo()
    camera_info_r.width = 720
    camera_info_r.height = 480
    
    camera_info_r.K = [903.2833424489683, 0.0, 355.597617808124, 0.0, 451.4561079203834, 236.80024491597652, 0.0, 0.0, 1.0]
    camera_info_r.distortion_model = 'plumb_bob'
    camera_info_r.D = [-0.5157698534507824, 0.3382203621218827, 0.002221360001684626, 0.0113171654220934, 0.0]
    camera_info_r.R = [0.9961024363941907, 0.010318565910589664, -0.08759830710182695, -0.009726097698249862, 0.9999268680335709, 0.00718760100018136, 0.08766606660003215, -0.006307597175036321, 0.9961299488444046]
    camera_info_r.P = [955.0066303298178, 0.0, 401.62639236450195, 0.0, 0.0, 955.0066303298178, 239.90731811523438, 0.0, 0.0, 0.0, 1.0, 0.0]




    # camera_info_r.width = 480
    # camera_info_r.height = 320
    
    # camera_info_r.K = [627.6029864112687, 0.0, 263.1007874539984, 0.0, 313.77571554942216, 156.89613486921158, 0.0, 0.0, 1.0]
    # camera_info_r.distortion_model = 'plumb_bob'
    # camera_info_r.D = [-0.5090017601968297, 0.287519549405079, -0.00030348556078139337, -0.004383122309850391, 0.0]
    # camera_info_r.R = [0.9999477690735148, -0.008511214427106064, 0.005658476285763898, 0.008411869997624655, 0.9998140112222695, 0.017354636463453355, -0.005805132904920603, -0.01730613164781158, 0.999833385239433]
    # camera_info_r.P = [611.1266777269319, 0.0, 248.998685836792, 0.0, 0.0, 611.1266777269319, 160.05079078674316, 0.0, 0.0, 0.0, 1.0, 0.0]

    
    

    camera_info  = CameraInfo()
    camera_info.header.frame_id = "base_link"
    camera_info.width = 720
    camera_info.height = 480
    camera_info.K = [908.0941644955262, 0.0, 306.90475834321825, 0.0, 454.3393389975465, 240.9876389279201, 0.0, 0.0, 1.0]
    camera_info.R = [0.9951886517491079, -0.004470309105157225, -0.09787524593224427, 0.0038083429281338963, 0.9999686024746445, -0.0069491430441642335, 0.09790323770915182, 0.006542965796248057, 0.9951744297582482]
    camera_info.distortion_model = 'plumb_bob'
    camera_info.D = [-0.4940329725921402, 0.27135719595076957, 0.0003134243142443668, 0.012435315025056166, 0.0]
    camera_info.P = [955.0066303298178, 0.0, 401.62639236450195, -75.3413158764854, 0.0, 955.0066303298178, 239.90731811523438, 0.0, 0.0, 0.0, 1.0, 0.0]
    
    Trans = [-0.07548987959077738, 0.0012255936056641257, 0.0016636267757271068]
    Rot = [0.9995771640859589, 0.006947836564230706, 0.02823509527075794, -0.007918318065378233, 0.9993765593362589, 0.03440629140486592, -0.02797844307475924, -0.034615317654151094, 0.9990090022151025]


    # camera_info.width = 480
    # camera_info.height = 320
    # camera_info.K = [628.3185725385997, 0.0, 240.11896587417755, 0.0, 314.05692677563303, 163.40068050348316, 0.0, 0.0, 1.0]
    # camera_info.R = [0.9996255883347984, -0.016229125490228355, -0.022029494595440194, 0.015844867315420657, 0.9997211873287167, -0.017506792562250354, 0.022307472426629185, 0.017151183396004237, 0.9996040283942694]
    # camera_info.distortion_model = 'plumb_bob'
    # camera_info.D = [-0.4988350997319012, 0.25482774803000263, -0.0015023543298646822, -0.003509780721549894, 0.0]
    # camera_info.P =[611.1266777269319, 0.0, 248.998685836792, -46.15115884855338, 0.0, 611.1266777269319, 160.05079078674316, 0.0, 0.0, 0.0, 1.0, 0.0]
    # Trans = [-0.07548987959077738, 0.0012255936056641257, 0.0016636267757271068]
    # Rot = [0.9995771640859589, 0.006947836564230706, 0.02823509527075794, -0.007918318065378233, 0.9993765593362589, 0.03440629140486592, -0.02797844307475924, -0.034615317654151094, 0.9990090022151025]


    rate = rospy.Rate(50) # 10hz
    bridge = CvBridge()
    UDP_IP = "0.0.0.0"  # Listen on all available network interfaces
    UDP_PORT = 5005
    MAX_DGRAM_SIZE = 65507

    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((UDP_IP, UDP_PORT))

# Set socket timeout
    server.settimeout(30.0)

# Start video stream
    data = b''
    while True:
        #print("Waiting for data...")
        try:
            chunk, addr = server.recvfrom(MAX_DGRAM_SIZE)
        except socket.timeout:
            print("Socket timeout.")
            continue
        data += chunk
        if len(chunk) < MAX_DGRAM_SIZE:
            try:
                img_a = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                img = img_a
                # print(f"image shape is {img.shape[0], img.shape[1], img.shape[2]}")
                a = int((img.shape[1]/2))
                img_r = img[:, 0:a, :]
                img_r = cv2.rotate(img_r, cv2.ROTATE_90_COUNTERCLOCKWISE)
                img_l = img[:, a:int(img.shape[1] ), :]
                img_l = cv2.rotate(img_l, cv2.ROTATE_90_COUNTERCLOCKWISE)
                # print(f"img_l shape is {img_l.shape} and img_r shape is {img_r.shape} and overal is {img.shape}")
                actual_time = rospy.Time.now()
                camera_info.header.stamp = actual_time
                camera_info_r.header.stamp = actual_time

               
                

                img_msg = bridge.cv2_to_imgmsg(img_l, encoding="bgr8")
                #img_msg = cv2.rotate(img_msg, cv2.ROTATE_90_COUNTERCLOCKWISE)
                img_msg.header.stamp = actual_time
                
                img_msg_r = bridge.cv2_to_imgmsg(img_r, encoding="bgr8")
                img_msg_r.header.frame_id = "base_link"
                img_msg.header.frame_id = "base_link"
                img_msg_r.header.stamp = actual_time
                
                pub.publish(img_msg)
                pub_in.publish(camera_info)
                pub_r.publish(img_msg_r)
                pub_in_r.publish(camera_info_r)

                rate.sleep()
                #cv2.imshow("Received image", img)
                #cv2.waitKey(1)
            except:
                print("could not recieve")
            
            # print("Waiting for data...")
            data = b""

        # if cv2.waitKey(1) & 0xFF == ord("q"):
        #     break


        # img = cv2.imread('/home/hami/MicrosoftTeams-image.png', 1) # Replace with your image path
        # #print(img.type)
        # img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        # pub.publish(img_msg)
        # rate.sleep()

if __name__ == '__main__':
    main()