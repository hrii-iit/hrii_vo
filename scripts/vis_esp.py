#! /usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseStamped
from hrii_robot_msgs.msg import RobotState
from nav_msgs.msg import Odometry
first = True

first_c = True

def callback_a(odom_msg):
    global latest_pose_stamped
    global odom_pr
    global first
    global j
    global robot_rf
    global camera_rf


    PP = PoseStamped()

    if first:
        Li = ["start"]
        color = (0, 255, 0)
        j=0
        camera_rf = odom_msg
        

        #setting reference


          #gripper reference location
        first = False

    #if j%100 == 0:
    #    camera_rf = odom_msg
    #    robot_rf = latest_pose_stamped  # robot reference location

    # Get the odometry data from the message
    #global x
    #global y
    #global z
    if j>0:
        PP.pose.position.x = -(odom_msg.pose.pose.position.y - camera_rf.pose.pose.position.y)
        PP.pose.position.y = +(odom_msg.pose.pose.position.x - camera_rf.pose.pose.position.x)
        PP.pose.position.z = +(odom_msg.pose.pose.position.z - camera_rf.pose.pose.position.z)
        PP.pose.orientation.x = -(odom_msg.pose.pose.orientation.x - camera_rf.pose.pose.orientation.x)
        PP.pose.orientation.y = +(odom_msg.pose.pose.orientation.y - camera_rf.pose.pose.orientation.y)
        PP.pose.orientation.z = +(odom_msg.pose.pose.orientation.z - camera_rf.pose.pose.orientation.z)
        PP.pose.orientation.w = +(odom_msg.pose.pose.orientation.w - camera_rf.pose.pose.orientation.w)


        PP.header = odom_msg.header
        
        pub.publish(PP)




    j+=1


    # Do something with the odometry data, such as printing it or storing it in a data structure
    #print("Received odometry: x={}, y={}, z={}".format(x, y, z))
    #print(f"print pose:{latest_pose_stamped}")
def callback_b(pose):
    # Get the pose data from the message
    global latest_pose_stamped
    latest_pose_stamped = pose
    #print(latest_pose_stamped)


    # Do something with the pose data, such as printing it or storing it in a data structure
    #print("Received pose: x={}, y={}, z={}".format(x, y, z))

def callback_c(odom_msg):
    global latest_pose_stamped
    global odom_pr
    global first_c
    global i
    #global robot_rf
    global camera_rf_c


    PP = PoseStamped()

    if first_c:
        Li = ["start"]
        color = (0, 255, 0)
        i=0
        camera_rf_c = odom_msg
        

        #setting reference


          #gripper reference location
        first_c = False

    #if j%100 == 0:
    #    camera_rf = odom_msg
    #    robot_rf = latest_pose_stamped  # robot reference location

    # Get the odometry data from the message
    #global x
    #global y
    #global z
    if i>0:
        PP.pose.position.x = +(odom_msg.pose.pose.position.y - camera_rf_c.pose.pose.position.y)
        PP.pose.position.z = +(odom_msg.pose.pose.position.x - camera_rf_c.pose.pose.position.x)
        PP.pose.position.y = -(odom_msg.pose.pose.position.z - camera_rf_c.pose.pose.position.z)
        PP.pose.orientation.x = -(odom_msg.pose.pose.orientation.x - camera_rf_c.pose.pose.orientation.x)
        PP.pose.orientation.y = +(odom_msg.pose.pose.orientation.y - camera_rf_c.pose.pose.orientation.y)
        PP.pose.orientation.z = +(odom_msg.pose.pose.orientation.z - camera_rf_c.pose.pose.orientation.z)
        PP.pose.orientation.w = +(odom_msg.pose.pose.orientation.w - camera_rf_c.pose.pose.orientation.w)


        PP.header = odom_msg.header
        
        pub_c.publish(PP)




    i+=1
    


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("visual_connector")
    global latest_pose_stamped
    # Subscribe to the pose_stamped topic
    #rospy.Subscriber("/franka_a/robot_state", RobotState, callback_b)
    rospy.Subscriber("/rtabmap/odom", Odometry, callback_a)
    rospy.Subscriber("/rtabmap/odom", Odometry, callback_c)
    # pub = rospy.Publisher('/pose_stamped', PoseStamped, queue_size=10)
    # Keep the node running until it is shut down

    pub = rospy.Publisher('/rgbd_odom', PoseStamped, queue_size=10)
    pub_c = rospy.Publisher('/zed2i_odom', PoseStamped, queue_size=10)
    rospy.spin()
