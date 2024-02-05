#! /usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation
from xsens_mvn_ros_msgs.msg import LinkStateArray


import tf
from tf.transformations import euler_from_matrix, euler_from_quaternion, quaternion_matrix

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation


# global tf_listener



global first
# tf_listener = tf.TransformListener()

first = True

def quaternion_2_euler(q):
    q_x = q[0]
    q_y = q[1]
    q_z = q[2]
    q_w = q[3]
    roll = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
    pitch = math.asin(2 * (q_w * q_y - q_z * q_x))
    yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
    return np.array([roll, pitch, yaw])


def euler_to_rotation_matrix(euler_angles):
        roll, pitch, yaw = euler_angles
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R_roll = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        R_pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_yaw = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        
        R = np.dot(R_yaw, np.dot(R_pitch, R_roll))
        return R

def callback(msg):
    
    global first
    global eu_c
    global M_T
    Link = LinkStateArray()

    desired_frame_id = 'right_hand'
    # for link_state in msg.link_states:
         
    #      if link_state.header.frame_id == desired_frame_id:
    #         # print("Frame ID:", link_state.header.frame_id)
    #         print("Position:", link_state.pose)
    #         print("Orientation:", link_state.pose.orientation)

    # print(msg.states[15].pose.orientation)
    if first:
        q_a = msg.states[15].pose.orientation

        Q = msg.states[15]

        # print(Q)
        q_c = [q_a.x, q_a.y, q_a.z, q_a.w]
        M_T = quaternion_matrix(q_c)

        eu_c = quaternion_2_euler(q_c)
        eu_c = eu_c - [0.567232, -1.5033226033712, -0.3039785346839]
        print(f"shape is {eu_c.shape}")
        first = False
        print(f"Basis is in {np.rad2deg(eu_c)}")
        # time.sleep(3)




    
        
    q_a = msg.states[15].pose.orientation

    Q = msg.states[15]

    # print(Q)
    q = [q_a.x, q_a.y, q_a.z, q_a.w]
    M_a = quaternion_matrix(q)
    M_a = np.linalg.inv(M_a)

    T_F = np.dot(M_T, M_a)

    T_F = np.linalg.inv(T_F)
    eu_m = euler_from_matrix(T_F)
    print(f"ANGS are {np.rad2deg(eu_m)}")

    eu = quaternion_2_euler(q)
    eu = eu - [0.567232, -1.5033226033712, -0.3039785346839]
    # eu = euler_from_quaternion(q_a)
    ang = eu_c - eu

    
    # print(f"X = {a[0]} ")
    # print(np.rad2deg(ang))
    parent_frame = '/world'
    child_frame = '/skeleton_right_hand'
    (trans, rot) = tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
    eu_a = euler_from_quaternion(rot)
    # a = np.rad2deg(eu[0])

    # if a>178: 
    #     a = 179



    # print(f"the angle is {np.rad2deg(eu_a[0])}")


    # print(f"angles in y are {np.rad2deg(msg.states[15].pose.position.y)}")

    # print(f"angles in z are {np.rad2deg(msg.states[15].pose.position.z)}")

def main():
    # rospy.init_node('tf_listener', anonymous=True)
    global tf_listener
    global first 
    global trans_p
    global rot_p   
    global An
    # NNNN = tf_msg
    PP = PoseStamped()
    
    # first = True

    parent_frame = '/world'
    child_frame = '/skeleton_right_hand'
    rate = rospy.Rate(100)

    

    # tf_listener = tf.TransformListener()

    # tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(4.0))
    
    if first:
        (trans_p, rot_p) = tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        An = np.array(trans_p)
        first = False

    while True:

        (trans, rot) = tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        PP.pose.position.x = trans[0] - trans_p[0]
        # print(f"THe number is ssssssssssssssssssssssssssss{trans[0] - trans_p[0]}")
        PP.pose.position.y = trans[1] - trans_p[1]
        PP.pose.position.z = trans[2] - trans_p[2]


        eu = euler_from_quaternion(rot)
        print(f"the angle is {np.rad2deg(eu)}")



        # print(f"x_b = {abs(trans[0] - trans_p[0]) + abs(trans[1] - trans_p[1]) + abs(trans[2] - trans_p[2])}")
        # print(f"X is {trans[0]}")
        # print(f"Transform from {parent_frame} to {child_frame}:")
        # print("X:", An)
        # print("Rotation:", rot)
        q_a = rot
        eu_ca_ref = quaternion_2_euler(q_a)
        # PP.header = tf_msg.header
        # print(PP)
        # pub.publish(PP)
        R_main = euler_to_rotation_matrix(eu_ca_ref)
        rate.sleep()
        # print(PP)

        # print("eu_ca_ref:", np.rad2deg(eu_ca_ref))

    # except:
    #     rospy.logwarn("Transform lookup failed.")


    # NNNN = tf_msg
        




global parent_frame
global child_frame

if __name__ == '__main__':
    
    rospy.init_node("TF_LISTENER")
    
    
    tf_listener = tf.TransformListener()
    parent_frame = '/world'
    child_frame = '/skeleton_right_hand'
    tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(4.0))
    # tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(4.0))
    rospy.Subscriber('/xsens/link_states', LinkStateArray, callback)
    rospy.spin()
    # rospy.Subscriber("/xsens/joint_states", JointState, main)
    # main()

    # pub = rospy.Publisher('/Pose', PoseStamped, queue_size=10)

    # main()
    # rospy.spin()


# def callback():
#     global tf_listener
#     rospy.init_node('tf_listener', anonymous=True)
#     parent_frame = '/world'
#     child_frame = '/skeleton_right_hand'
    

#     try:
#         (trans, rot) = tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

#         print(f"Transform from {parent_frame} to {child_frame}:")
#         print("Translation:", trans)
#         print("Rotation:", rot)

#     except:
#         rospy.logwarn("Transform lookup failed.")

