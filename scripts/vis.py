#! /usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from hrii_robot_msgs.msg import RobotState

from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
# global listener
# listener = tf.TransformListener()


def quaternion_to_euler(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
    pitch = math.asin(2 * (q.w * q.y - q.z * q.x))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
    return np.array([roll, pitch, yaw])

def quaternion_from_euler(Ang):
    roll, pitch, yaw = Ang
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return np.array([qx, qy, qz, qw])

#listener = tf.TransformListener()
global broadcaster

broadcaster = tf.TransformBroadcaster()
first = True
def connect_tf_tree():
    global broadcaster
    frame_tree1 = 'map'
    frame_tree2 = 'franka_a_franka_base_link'
    identity_rot = [0, 0, 0, 1]
    identity_trans = (0, 0, 0)
    # broadcaster = tf.TransformBroadcaster()
    broadcaster.sendTransform(identity_trans, identity_rot, rospy.Time.now(), frame_tree2, frame_tree1)

    # Create a TF broadcaster to publish the connection identity transform
    # broadcaster = tf.TransformBroadcaster()

def callback_a(odom_msg):
    global latest_pose_stamped
    # global listener
    global odom_pr
    global first
    global j
    global robot_rf
    global camera_rf
    global eu_ro_ref
    global eu_ca_ref

    connect_tf_tree()

    PP = Pose()

    if first:
        Li = ["start"]
        color = (0, 255, 0)
        j=0
        #setting reference


          #gripper reference location
        first = False

    if j == 0:
        q_b = PP.orientation
        eu_ro_ref = quaternion_to_euler(q_b)
        q_a = odom_msg.pose.pose.orientation
        eu_ca_ref = quaternion_to_euler(q_a)
        
        camera_rf = odom_msg
        robot_rf = latest_pose_stamped  # robot reference location

    # Get the odometry data from the message
    #global x
    #global y
    #global z
    if j>2:
        # PP.pose.position.x = robot_rf.O_T_EE.position.x-2*(odom_msg.pose.pose.position.y - camera_rf.pose.pose.position.y)
        # PP.pose.position.y = robot_rf.O_T_EE.position.y+2*(odom_msg.pose.pose.position.x - camera_rf.pose.pose.position.x)
        # PP.pose.position.z = robot_rf.O_T_EE.position.z+2*(odom_msg.pose.pose.position.z - camera_rf.pose.pose.position.z)

        PP.position.x = robot_rf.position.x+1*(odom_msg.pose.pose.position.x - camera_rf.pose.pose.position.x)
        PP.position.y = robot_rf.position.y+1*(odom_msg.pose.pose.position.y - camera_rf.pose.pose.position.y)
        PP.position.z = robot_rf.position.z+1*(odom_msg.pose.pose.position.z - camera_rf.pose.pose.position.z)

        #(trans,rot) = listener.lookupTransform('franka_a_franka_base_link', 'franka_a_franka_flange', rospy.Time(0))
        # PP.pose.position.x = robot_rf.O_T_EE.position.x
        # PP.pose.position.y = robot_rf.O_T_EE.position.y
        # PP.pose.position.z = robot_rf.O_T_EE.position.z

        # q = PP.pose.orientation
        # eu = quaternion_to_euler(q)
        q_c = odom_msg.pose.pose.orientation
        eu_c = quaternion_to_euler(q_c)
        ang_a = eu_ca_ref - eu_c
        ang_a = ang_a + np.array([-3.095, -0.225, -0.001])
        Ang = eu_ro_ref - ang_a
        # print(f"ang_a is {ang_a}")
        # print(f"Ang is {Ang}")
        # print(f"reference is {eu_ro_ref}")
        



        q_t = quaternion_from_euler(Ang)
        #print(q_t)

        
        









        # PP.pose.orientation.x = robot_rf.O_T_EE.orientation.x + 1*(odom_msg.pose.pose.orientation.x - camera_rf.pose.pose.orientation.x)
        # PP.pose.orientation.y = robot_rf.O_T_EE.orientation.y + 1*(odom_msg.pose.pose.orientation.y - camera_rf.pose.pose.orientation.y)
        # PP.pose.orientation.z = robot_rf.O_T_EE.orientation.z + 1*(odom_msg.pose.pose.orientation.y - camera_rf.pose.pose.orientation.y)
        # PP.pose.orientation.w = robot_rf.O_T_EE.orientation.w + 1*(odom_msg.pose.pose.orientation.w - camera_rf.pose.pose.orientation.w)

        # PP.pose.orientation.x = robot_rf.O_T_EE.orientation.x + q_t[0]
        # PP.pose.orientation.y = robot_rf.O_T_EE.orientation.y + q_t[1]
        # PP.pose.orientation.z = robot_rf.O_T_EE.orientation.z + q_t[2]
        # PP.pose.orientation.w = robot_rf.O_T_EE.orientation.w + q_t[3]

        PP.orientation.x = q_t[0]
        PP.orientation.y = q_t[1]
        PP.orientation.z = q_t[2]
        PP.orientation.w = q_t[3]



        # PP.header = latest_pose_stamped.header
        pub.publish(PP)
        connect_tf_tree()
        # print()




    j+=1


    # Do something with the odometry data, such as printing it or storing it in a data structure
    #print("Received odometry: x={}, y={}, z={}".format(x, y, z))
    #print(f"print pose:{latest_pose_stamped}")
def callback_b(pose):
    # Get the pose data from the message
    global latest_pose_stamped
    latest_pose_stamped = pose
    print("HI")
    print(latest_pose_stamped.orientation.x)
    


    # Do something with the pose data, such as printing it or storing it in a data structure
    #print("Received pose: x={}, y={}, z={}".format(x, y, z))




if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("pose_stamped_subscriber")

    global latest_pose_stamped
    # Subscribe to the pose_stamped topic
    rospy.Subscriber("/franka_a/cartesian_pose_example_controller/O_T_EE", Pose, callback_b)
    rospy.Subscriber("/rtabmap/odom", Odometry, callback_a)
    
    # pub = rospy.Publisher('/pose_stamped', PoseStamped, queue_size=10)
    # Keep the node running until it is shut down

    pub = rospy.Publisher('/franka_a/cartesian_impedance_controller/equilibrium_pose', Pose, queue_size=10)
    rospy.spin()
