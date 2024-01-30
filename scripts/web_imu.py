#!/usr/bin/env python3
import socket
import struct
import rospy
from sensor_msgs.msg import Imu

rospy.init_node('image_publisher')
# Define UDP server IP address and port number
UDP_IP = "0.0.0.0" # Listen on all available network interfaces
UDP_PORT = 6006    # Same port number as the UDP client
pub = rospy.Publisher('/camera/imu', Imu, queue_size=10)
imu = Imu()
imu.header.frame_id = "base_link"

# Create a UDP socket and bind it to the specified IP address and port number
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
rate = rospy.Rate(100)

# Receive and decode incoming UDP packets
while True:
    # Receive a UDP packet and get the sender's IP address and port number
    packet, addr = sock.recvfrom(1024) # 1024 is the maximum packet size
    
    # Unpack the binary packet into the IMU data
    ax, ay, az, gx, gy, gz = struct.unpack('ffffff', packet)
    imu.header.stamp = rospy.Time.now()
    imu.orientation.x = 0.0
    imu.orientation.y = 0.0
    imu.orientation.z = 0.0
    imu.orientation.w = 0.0
    imu.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    imu.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    imu.linear_acceleration.x = ax
    imu.linear_acceleration.y = ay
    imu.linear_acceleration.z = az
    imu.angular_velocity.x = gx
    imu.angular_velocity.y = gy
    imu.angular_velocity.z = gz
    imu.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    pub.publish(imu)
    rate.sleep()
    # Print the received IMU data
    print(f"Acceleration: ({ax:.2f}, {ay:.2f}, {az:.2f}) | Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f})")
