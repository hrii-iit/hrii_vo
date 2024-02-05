#! /usr/bin/env python3


import rospy
import actionlib
import actionlib_tutorials.msg
import time
import dynamic_reconfigure.client
from xsens_mvn_ros_msgs.msg import LinkStateArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import WrenchStamped

from std_srvs.srv import Trigger, TriggerResponse, Empty, SetBoolResponse, SetBool
from geometry_msgs.msg import Pose
from quaternion import from_rotation_matrix
from hrii_robot_msgs.msg import RobotState
from tf.transformations import euler_from_matrix, euler_from_quaternion, quaternion_matrix
from scipy.linalg import block_diag, inv
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

from scipy import signal

# global listener
# listener = tf.TransformListener()
global start_loco
global ready
global start
global init
global x_hat
global state
global C_O
global C
global Q
global Q_o
global H
global R
global UU
global A
global ang_hat
global ns
global P
global reset_xsens

reset_xsens = False

state = 0
ready = 0

start_loco = False

init = False
start = False
global transform
global Locomotion
Locomotion = False
global j
ns = rospy.get_namespace()
ns = ns[:-1]
print(f"ns is {ns}")


x_O = np.array([0.0, 0.0, 0.0])
P_O = np.diag([0.04, 0.04, 0.04])
C_O = np.diag([0.04, 0.04, 0.04])


A = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])

Q = np.diag([0.002, 0.002, 0.002])
Q_o = np.diag([0.001, 0.001, 0.001])
H = np.eye(3)
R = np.diag([2.5, 2.5, 2.5])
UU = np.diag([0.5, 0.5, 0.5])
x_hat = x_O
ang_hat = x_O
C = C_O
P = P_O

j = 0

global response_c
response_c = False

def force_read(F_msg):
    print(F_msg)

# def calibration(x, y, z):
#     A = [x, y, z]


    
def Teleop_loco(req):
    # global start
    global start_loco
    global state
    global MODE_TELEOP_CAM
    

    
     
    # start = False
    start_loco = req.data 
    
    if MODE_TELEOP_CAM:
        odom_reset()
    # client = dynamic_reconfigure.client.Client("/moca_red/dynamic_reconfigure_compliance_param_node")
    # params = {'translational_stiffness' : 500.0, 'rotational_stiffness' : 30.0}
    # config = client.update_configuration(params)
    response = SetBoolResponse()
    response.success = True
    response.message = f"locomotion set on {start_loco}"
    

    j = 0
    return response





def Teleop_start(req):
    global j
    global transform
    
    # vis_indicator_proxy = rospy.ServiceProxy('/vo_tele', Trigger)
    #j=0
    global start
    global ready
    global state
    global MODE_TELEOP_CAM
    global T_z

    ready = 0
    
    state = 1
    # global rot_J_matrix
    if MODE_TELEOP_CAM:
        odom_reset()

    else:
        reset_xsens = True
    
    
    # vis_indicator_proxy()
    client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
    params = {'translational_stiffness' : 50.0, 'rotational_stiffness' : 25.0}
    config = client.update_configuration(params)
    response = TriggerResponse()
    response.success = True
    start = True
    transform = False
    response.message = "Teleop Started"
    j = 0
    # listener.waitForTransform('/moca_red_odom', '/moca_red_franka_base_link', rospy.Time(), rospy.Duration(1.0))
    # (trans_J, rot_J) = listener.lookupTransform('/moca_red_odom','/moca_red_franka_base_link', rospy.Time(0))
    # rot_J_matrix = tf.transformations.quaternion_matrix(rot_J)
    # T_z = np.eye(4)
    
    # T_z[:3, :3] = rot_J_matrix[:3, :3]
    # T_z[:3, 3] = trans_J
    # T_z[:3, :3] = rot_J_matrix[:3, :3]
    # T_z[:3, 3] = np.array([0, 0, 0])
    return response

def Teleop_end(req):
    global state
    global j
    global start
    global ready

    ready = 0

    state = 2

    response_a = TriggerResponse()
    response_a.success = True
    # odom_reset()
    start = False
    response_a.message = "Teleop Ended"
    return response_a


def loco(req):
    global start
    global response_c
    global state
    global Locomotion
    global admit_checker
    A = admit_checker()
    
    
    response_c = TriggerResponse()
    if start:
        response_c.success = True
        response_c.message = f"visual_odom started {start}: ready to do locomotion"
        # Locomotion = True
    else:
        response_c.success = False
        response_c.message = f"visual odom started {start}"
    print(A)
    if A.success:
        Locomotion = True
    else:
        Locomotion = False
    # response_c.message = f"Loco_checker is {start}"
    return response_c

def Teleop_homing(req):
    global j 

    global state
    global ready

    state = 3
    ready = 0
    #j = 0
    global latest_pose_stamped
    global start
    client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
    params = {'translational_stiffness' : 45.0, 'rotational_stiffness' : 15.0}
    config = client.update_configuration(params)
    listener.waitForTransform(f'{ns}_odom', f'{ns}_franka_base_link', rospy.Time(), rospy.Duration(1.0))
    (trans_H, rot_H) = listener.lookupTransform(f'{ns}_odom',f'{ns}_franka_base_link', rospy.Time(0))
    rot_matrix = tf.transformations.quaternion_matrix(rot_H)
    T_a = np.eye(4)
    
    T_a[:3, :3] = rot_matrix[:3, :3]
    T_a[:3, 3] = trans_H

    T_b = np.array([[0.9717, 0.0486, 0.2313, 0.316],[0.0388, -0.9982, 0.0468, 0.0], [0.2331, -0.0365, -0.9718, 0.658], [0.0, 0.0, 0.0, 1.0]])

    T_c = np.dot(T_a, T_b)
    q = from_rotation_matrix(T_c[:3, :3])
    response_b = TriggerResponse()
    response_b.success = True
    # odom_reset()
    start = False
    response_b.message = "Homing has been initiated restart the teleoperating"
    PP = PoseStamped()
    # PP.pose.position.x = 0.316
    # PP.pose.position.y = 0.00
    # PP.pose.position.z = 0.718
    # q_x = 0.993 
    # q_y = 0.003 
    # q_z = 0.112
    # q_w = -0.023
    PP.pose.position.x = T_c[0, 3]
    PP.pose.position.y = T_c[1, 3]
    PP.pose.position.z = T_c[2, 3]
   



    q_t = [q.x, q.y, q.z, q.w]
    PP.pose.orientation.x = q_t[0]
    PP.pose.orientation.y = q_t[1]
    PP.pose.orientation.z = q_t[2]
    PP.pose.orientation.w = q_t[3]



    PP.header = latest_pose_stamped.header
    pub.publish(PP)
    time.sleep(4)
    client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
    params = {'translational_stiffness' : 150.0, 'rotational_stiffness' : 25.0}
    config = client.update_configuration(params)
    rospy.sleep(2)
    client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
    params = {'translational_stiffness' : 600.0, 'rotational_stiffness' : 25.0}
    config = client.update_configuration(params)
    rospy.sleep(1)

    return response_b
    




    
def quaternion_2_euler(q):
    q_x = q[0]
    q_y = q[1]
    q_z = q[2]
    q_w = q[3]
    roll = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
    pitch = math.asin(2 * (q_w * q_y - q_z * q_x))
    yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
    return np.array([roll, pitch, yaw])

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


global listener
#listener = tf.TransformListener()
global broadcaster

broadcaster = tf.TransformBroadcaster()


# listener.waitForTransform('/franka_a_franka_EE', '/franka_a_franka_base_link', rospy.Time())
first = True
def connect_tf_tree():
    global broadcaster
    frame_tree1 = 'map'
    frame_tree2 = 'moca_red_odom'
    identity_rot = [0, 0, 0, 1]
    identity_trans = (0, 0, 0)
    # broadcaster = tf.TransformBroadcaster()
    broadcaster.sendTransform(identity_trans, identity_rot, rospy.Time.now(), frame_tree2, frame_tree1)


def connect_tf_tree_b():
    global broadcaster
    frame_tree1 = 'world'
    frame_tree2 = 'moca_red_odom'
    identity_rot = [0, 0, 0, 1]
    identity_trans = (0, 0, 0)
    # broadcaster = tf.TransformBroadcaster()
    broadcaster.sendTransform(identity_trans, identity_rot, rospy.Time.now(), frame_tree2, frame_tree1)





    
    # Create a TF broadcaster to publish the connection identity transform
    # broadcaster = tf.TransformBroadcaster()




def callback_c(tf_msg):
    global listener
    global latest_pose_stamped
    global first
    global j
    global i
    global R_main
    global robot_rf
    global ang_hat
    global camera_rf
    global Pos_fil
    global ang_fil
    global x_hat
    global Q
    global Q_o
    global H
    global R
    global UU
    global A
    global trans_s
    global rot_s
    global x
    global y
    global z
    global x_a
    global y_a
    global z_a
    global x_hat_minus
    global ang_hat_minus
    global C
    global C_minus
    
    global P
    global P_minus
    global K
    global reset_xsens

    global eu_ro_ref
    global eu_ca_ref
    global transform
    global Pos_array
    global start_loco
    global ready
    global init
    global state
    global Locomotion
    global ang_array
    global T_z

    NNNNN = tf_msg

    state_callforward()
    
    if start:
        listener.waitForTransform(f'{ns}_odom', f'{ns}_franka_base_link', rospy.Time(), rospy.Duration(1.0))
        listener.waitForTransform('/skeleton_right_hand', '/world', rospy.Time(), rospy.Duration(1.0))
        
        
        (trans_J, rot_J) = listener.lookupTransform(f'{ns}_odom',f'{ns}_franka_base_link', rospy.Time(0))
        rot_J_matrix = tf.transformations.quaternion_matrix(rot_J)
        T_z = np.eye(4)
        
        T_z[:3, :3] = rot_J_matrix[:3, :3]
        T_z[:3, 3] = trans_J
        T_z[:3, :3] = rot_J_matrix[:3, :3]
        T_z[:3, 3] = np.array([0, 0, 0])

    connect_tf_tree_b()

    PP = PoseStamped()
    if j>2 and start:
        if reset_xsens:
            # (trans_s, rot_s) = listener.lookupTransform('/world', '/skeleton_right_hand', rospy.Time(0))
            # q_a = rot_s
            q = tf_msg.states[15].pose.orientation
            (trans, rot) = listener.lookupTransform(f'{ns}_odom', f'{ns}_franka_EE', rospy.Time(0))
            eu_ro_ref = quaternion_matrix(rot)
            q_a = [q.x, q.y, q.z, q.w]
            eu_ca_ref = quaternion_matrix(q_a)
            
            trans_s = [tf_msg.states[15].pose.position.x, tf_msg.states[15].pose.position.y, tf_msg.states[15].pose.position.z]
            reset_xsens = False
        if transform==False:
            try:
                i = 0
                q_b = latest_pose_stamped.O_T_EE.orientation
                # eu_ro_ref = quaternion_matrix(q_b)
                trans_s = [tf_msg.states[15].pose.position.x, tf_msg.states[15].pose.position.y, tf_msg.states[15].pose.position.z]
                q = tf_msg.states[15].pose.orientation
                q_a = [q.x, q.y, q.z, q.w]
                eu_ca_ref = quaternion_matrix(q_a)
                # (trans_s, rot_s) = listener.lookupTransform('/world', '/skeleton_right_hand', rospy.Time(0))
                # q_a = rot_s
                # print(f"q_a is {q_a}")
                # eu_ca_ref = quaternion_2_euler(q_a)
                # q_a = rot_s
                
                
                
                listener.waitForTransform(f'{ns}_odom', f'{ns}_franka_EE', rospy.Time(), rospy.Duration(1.0))
                (trans, rot) = listener.lookupTransform(f'{ns}_odom', f'{ns}_franka_EE', rospy.Time(0))
                connect_tf_tree_b()
                eu_ro_ref = quaternion_matrix(rot)
                e_m = quaternion_2_euler(rot)
                transform = True
                R_main = euler_to_rotation_matrix(e_m)

                robot_rf = latest_pose_stamped 
                x = robot_rf.O_T_EE.position.x
                y = robot_rf.O_T_EE.position.y
                z = robot_rf.O_T_EE.position.z
                x_a = x
                y_a = y
                z_a = z
                first = False
                i+=1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Failed to get the TF transform")
                transform = False
        else:
            try: 
                i +=1
            except:
                rospy.logerr("Failed to get the TF transform") 
    if start and not first:
        if transform: 
            trans_c = [tf_msg.states[15].pose.position.x, tf_msg.states[15].pose.position.y, tf_msg.states[15].pose.position.z]
            # (trans_c, rot_c) = listener.lookupTransform('/world', '/skeleton_right_hand', rospy.Time(0))
            x_b = trans_c[0] - trans_s[0]
            y_b = trans_c[1] - trans_s[1]
            z_b = trans_c[2] - trans_s[2]
            XX = np.array([[x_b], [y_b], [z_b], [1]])
            TF = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
            XY = np.dot(TF, XX)

            # print(f"x_b is {x_b}, y_b is {y_b}, and z_b is {z_b}")
            x_b = XY[2, 0]
            y_b = -XY[0, 0]
            z_b = -XY[1, 0]
            
            X = np.eye(4)
            X[:3, 3] = np.array([x_b, y_b, z_b])
            Y = np.dot(T_z, X)

            x = robot_rf.O_T_EE.position.x+1.5*Y[0, 3] - 0.6*(x_a - x)
            y = robot_rf.O_T_EE.position.y+1.5*Y[1, 3] - 0.6*(y_a - y)
            z = robot_rf.O_T_EE.position.z+1.5*Y[2, 3] - 0.6*(z_a - z)
            # ///////////////////////////
            # REVOME IT IF YOU ARE USING THE TRANSLATION
            # //////////////////////

            # x = robot_rf.O_T_EE.position.x
            # y = robot_rf.O_T_EE.position.y
            # z = robot_rf.O_T_EE.position.z

            # ///////////////////////////
            # REVOME IT IF YOU ARE USING THE TRANSLATION
            # //////////////////////

            # (trans_e, rot_e) = listener.lookupTransform('/world', '/skeleton_right_hand', rospy.Time(0))
            # q_c = rot_e
            q = tf_msg.states[15].pose.orientation
            q_c = [q.x, q.y, q.z, q.w]
            # print(f"q_c is {q_c}")

            eu_c = quaternion_matrix(q_c)
            eu_c = np.linalg.inv(eu_c)

            # print(f"q_c is {q_c} and eu_c is {np.rad2deg(eu_c)}")

            T_F = np.dot(eu_ca_ref, eu_c)
            
            T_F = np.linalg.inv(T_F)

            Q_com = np.dot(T_F, eu_ro_ref)
            # angle_c = 1.0

            # ang_o = eu_c.copy()
            # eu_c[0] = -1*ang_o[2]
            # eu_c[2] = -1*ang_o[1]
            # eu_c[1] = -1*ang_o[0]

            # ang_z = eu_ca_ref
            # eu_ca_ref[0] = -1*ang_z[2]
            # eu_ca_ref[2] = -1*ang_z[1]
            # eu_ca_ref[1] = -1*ang_z[0]

            # ang_a = eu_ca_ref - eu_c



            # print(f"ang is {ang_a}")

            # rot_a = euler_to_rotation_matrix(ang_a)
            # TF_a = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
            # # TF_a = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            # rot_a = np.dot(TF_a, rot_a)
            # q_b = from_rotation_matrix(rot_a)
            # ang_a = quaternion_to_euler(q_b)
            # ang_a = angle_c*ang_a

            # ang_a = ang_a.reshape((1, 3))

            # ang_a = ang_a.reshape((3, 1))
            # ang_hat_minus = np.dot(A, ang_hat)
            # C_minus = np.dot(np.dot(A, C), A.T) + Q_o
            # L = np.dot(np.dot(C_minus, H.T), inv(np.dot(np.dot(H, C_minus), H.T) + UU))
            # ang_hat = ang_hat_minus + np.dot(L, ang_a - np.dot(H, ang_hat_minus))
            # ang_a_hat = np.average(ang_hat, axis=1)
            # ang_a_hat = ang_a_hat.reshape((1, 3))
            # C = C_minus - np.dot(np.dot(L, H), C_minus)

            Pos = np.array([x, y, z])
            Pos = Pos.reshape((3, 1))
            x_hat_minus = np.dot(A, x_hat)
            P_minus = np.dot(np.dot(A, P), A.T) + Q
            K = np.dot(np.dot(P_minus, H.T), inv(np.dot(np.dot(H, P_minus), H.T) + R))
            x_hat = x_hat_minus + np.dot(K, Pos - np.dot(H, x_hat_minus))
            Pos_hat = np.average(x_hat, axis=1)
            Pos_hat = Pos_hat.reshape((1, 3))
            P = P_minus - np.dot(np.dot(K, H), P_minus)
            

            if init==False:
                # ang_array = ang_a
                Pos_array = Pos
                Pos_fil = Pos_hat
                # ang_fil = ang_a_hat
                # j = 3
                init = True

            
            else:
                # ang_array = np.concatenate((ang_array, ang_a), axis=0)
                # ang_fil = np.concatenate((ang_fil, ang_a_hat), axis=0)

                Pos_array = np.concatenate((Pos_array, Pos), axis=0)
                Pos_fil = np.concatenate((Pos_fil, Pos_hat), axis=0)

            if j==100 and not start_loco:
                client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
                params = {'translational_stiffness' : 150.0, 'rotational_stiffness' : 25.0}
                config = client.update_configuration(params) 
                ready = 0
            if j==100 and start_loco:
                state = 4

            if j==110 and not start_loco:
                client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
                params = {'translational_stiffness' : 325.0, 'rotational_stiffness' : 25.0}
                config = client.update_configuration(params)
            if j==120 and not start_loco:
                client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
                params = {'translational_stiffness' : 625.0, 'rotational_stiffness' : 25.0}
                config = client.update_configuration(params)
            
            if j==130 and not start_loco:
                client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
                params = {'translational_stiffness' : 1000.0, 'rotational_stiffness' : 25.0}
                config = client.update_configuration(params)
                ready = 1

            if j==130 and start_loco:
                client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
                params = {'translational_stiffness' : 500.0, 'rotational_stiffness' : 30.0}
                config = client.update_configuration(params)
                ready = 1
            
            if start and j>2 and i>100:
                # print(f"Pos fil shape is {Pos_fil.shape}")

                x_a, y_a, z_a = Pos_fil[-1, :]
                # print(f"Pos fil is made of {Pos_fil[-1, :]}")

                PP.pose.position.x = x_a
                PP.pose.position.y = y_a
                PP.pose.position.z = z_a
                
                # ang_a = ang_fil[-1, :]

                # R_i = euler_to_rotation_matrix(ang_a)
                # R_i = np.dot(TF_a, R_i)

                # R_a = euler_to_rotation_matrix([0, 0.0, 0]) ##rotation around x
                R_b = euler_to_rotation_matrix([0, 0, 0]) ##rotation around y
                R_a = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
                # R_i = np.dot(R_a, R_i)
                # R_t = np.dot(R_main, R_i)###Important to be editeds
                
                
                # R_t = np.dot(TF_a, R_t)

                # R_t = np.dot(R_a, R_t)


                q = from_rotation_matrix(Q_com)

                        
                q_t = [q.x, q.y, q.z, q.w]

                PP.pose.orientation.x = q_t[0]
                PP.pose.orientation.y = q_t[1]
                PP.pose.orientation.z = q_t[2]
                PP.pose.orientation.w = q_t[3]

                # PP.pose.orientation.x = robot_rf.O_T_EE.orientation.x
                # PP.pose.orientation.y = robot_rf.O_T_EE.orientation.y
                # PP.pose.orientation.z = robot_rf.O_T_EE.orientation.z
                # PP.pose.orientation.w = robot_rf.O_T_EE.orientation.w

                PP.header = latest_pose_stamped.header
                
                if Locomotion:
                    pub_a.publish(PP)
                else:
                    pub.publish(PP)


    j+=1
    # print("An")




def callback_a(odom_msg):
    global listener
    global latest_pose_stamped
    # global listener
    global odom_pr
    global first
    global j
    global i
    global R_main
    global robot_rf
    global camera_rf
    global Pos_fil
    global x_hat
    global Q
    global H
    global R
    global A
    global x
    global y
    global z
    global x_a
    global y_a
    global z_a
    global x_hat_minus
    global P
    global P_minus
    global K

    global eu_ro_ref
    global eu_ca_ref
    global transform
    global Pos_array
    global start_loco
    global ready
    global init
    global state
    global Locomotion
    global ang_array
    global T_z

    state_callforward()




    

    if start:
        listener.waitForTransform(f'{ns}_odom', f'{ns}_franka_base_link', rospy.Time(), rospy.Duration(1.0))
        (trans_J, rot_J) = listener.lookupTransform(f'{ns}_odom',f'{ns}_franka_base_link', rospy.Time(0))
        rot_J_matrix = tf.transformations.quaternion_matrix(rot_J)
        T_z = np.eye(4)
        
        T_z[:3, :3] = rot_J_matrix[:3, :3]
        T_z[:3, 3] = trans_J
        T_z[:3, :3] = rot_J_matrix[:3, :3]
        T_z[:3, 3] = np.array([0, 0, 0])


    connect_tf_tree()

    PP = PoseStamped()

 

    # if first:
    #     Li = ["start"]
    #     color = (0, 255, 0)
    #     j=0
    #     #setting reference
    #     try:
    #         q_b = latest_pose_stamped.O_T_EE.orientation
    #         eu_ro_ref = quaternion_to_euler(q_b)
    #         q_a = odom_msg.pose.pose.orientation
    #         eu_ca_ref = quaternion_to_euler(q_a)
    #         listener.waitForTransform('/franka_a_franka_base_link', '/franka_a_franka_EE', rospy.Time(), rospy.Duration(1.0))
    #         (trans, rot) = listener.lookupTransform('/franka_a_franka_base_link', '/franka_a_franka_EE', rospy.Time(0))
    #         # print("Translation: ", trans)
    #         # print("Rotation: ", rot)
    #         e_m = quaternion_2_euler(rot)

    #         transform = True
    #         R_main = euler_to_rotation_matrix(e_m)

    #         camera_rf = odom_msg
    #         robot_rf = latest_pose_stamped 
    #         first = False
    #         # s = input("please start as soon as map is ready")


    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         rospy.logerr("Failed to get the TF transform")
    #         transform = False
            
          #gripper reference location
        

    # if j == 0:
    #     # q_b = PP.pose.orientation
    #     q_b = latest_pose_stamped.O_T_EE.orientation
    #     eu_ro_ref = quaternion_to_euler(q_b)
    #     q_a = odom_msg.pose.pose.orientation
    #     eu_ca_ref = quaternion_to_euler(q_a)
    #     try:
    #         listener.waitForTransform('/franka_a_franka_base_link', '/franka_a_franka_EE', rospy.Time(), rospy.Duration(1.0))
    #         (trans, rot) = listener.lookupTransform('/franka_a_franka_base_link', '/franka_a_franka_EE', rospy.Time(0))
    #         # print("Translation: ", trans)
    #         # print("Rotation: ", rot)
    #         e_m = quaternion_2_euler(rot)

    #         transform = True
    #         R_main = euler_to_rotation_matrix(e_m)

    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         rospy.logerr("Failed to get the TF transform")
    #         transform = False
        
    #     camera_rf = odom_msg
    #     robot_rf = latest_pose_stamped  # robot reference location

    # Get the odometry data from the message
    #global x
    #global y
    #global z


    # if j>2 and start:
    

    if j>2 and start:
        if transform==False:
            try:
                i = 0
                q_b = latest_pose_stamped.O_T_EE.orientation
                eu_ro_ref = quaternion_to_euler(q_b)
                q_a = odom_msg.pose.pose.orientation
                eu_ca_ref = quaternion_to_euler(q_a)
                listener.waitForTransform(f'{ns}_odom', f'{ns}_franka_EE', rospy.Time(), rospy.Duration(1.0))
                (trans, rot) = listener.lookupTransform(f'{ns}_odom', f'{ns}_franka_EE', rospy.Time(0))
                connect_tf_tree()
                # print("Translation: ", trans)
                # print("Rotation: ", rot)
                e_m = quaternion_2_euler(rot)

                transform = True
                R_main = euler_to_rotation_matrix(e_m)

                camera_rf = odom_msg
                robot_rf = latest_pose_stamped 
                x = robot_rf.O_T_EE.position.x
                y = robot_rf.O_T_EE.position.y
                z = robot_rf.O_T_EE.position.z
                x_a = x
                y_a = y
                z_a = z
                first = False
                i+=1
                # s = input("please start as soon as map is ready")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Failed to get the TF transform")
                transform = False

        else:
            try: 
                i +=1
            except:
                rospy.logerr("Failed to get the TF transform")
    # window_size = 129  
    # print(j) 
    # PP.pose.position.x = robot_rf.O_T_EE.position.x-2*(odom_msg.pose.pose.position.y - camera_rf.pose.pose.position.y)
    # PP.pose.position.y = robot_rf.O_T_EE.position.y+2*(odom_msg.pose.pose.position.x - camera_rf.pose.pose.position.x)
    # PP.pose.position.z = robot_rf.O_T_EE.position.z+2*(odom_msg.pose.pose.position.z - camera_rf.pose.pose.position.z)
    if start and not first:
        if transform: 
            x_b = odom_msg.pose.pose.position.z - camera_rf.pose.pose.position.z
            y_b = -(odom_msg.pose.pose.position.x - camera_rf.pose.pose.position.x)
            z_b = -(odom_msg.pose.pose.position.y - camera_rf.pose.pose.position.y)

            X = np.eye(4)
            X[:3, 3] = np.array([x_b, y_b, z_b])
            Y = np.dot(T_z, X)

            x = robot_rf.O_T_EE.position.x+1.5*Y[0, 3] - 0.6*(x_a - x)
            y = robot_rf.O_T_EE.position.y+1.5*Y[1, 3] - 0.6*(y_a - y)
            z = robot_rf.O_T_EE.position.z+1.5*Y[2, 3] - 0.6*(z_a - z)
            
            
            q_c = odom_msg.pose.pose.orientation
            eu_c = quaternion_to_euler(q_c)
            
            eu_car = eu_ca_ref
            # eu_co = eu_c
            # eu_c[0] = eu_co[2]
            # eu_ca_ref[0] = eu_car[2]
            # eu_c[1] = eu_co[0]
            # eu_ca_ref[1] = eu_car[0]
            # eu_c[2] = eu_co[1]
            angle_c = 1.0
            # eu_ca_ref[2] = eu_car[1]
            ang_o = eu_c.copy()
            eu_c[0] = -1*ang_o[2]
            eu_c[2] = -1*ang_o[1]
            eu_c[1] = -1*ang_o[0]

            ang_z = eu_ca_ref
            eu_ca_ref[0] = -1*ang_z[2]
            eu_ca_ref[2] = -1*ang_z[1]
            eu_ca_ref[1] = -1*ang_z[0]



            
            # print(f"it is {eu_c}")
            
            ang_a = eu_ca_ref - eu_c


            ang_a = angle_c*ang_a

            ang_a = ang_a.reshape((1, 3))
            

            Pos = np.array([x, y, z])
            Pos = Pos.reshape((3, 1))
            x_hat_minus = np.dot(A, x_hat)
            P_minus = np.dot(np.dot(A, P), A.T) + Q
            K = np.dot(np.dot(P_minus, H.T), inv(np.dot(np.dot(H, P_minus), H.T) + R))
            x_hat = x_hat_minus + np.dot(K, Pos - np.dot(H, x_hat_minus))
            Pos_hat = np.average(x_hat, axis=1)
            Pos_hat = Pos_hat.reshape((1, 3))
            P = P_minus - np.dot(np.dot(K, H), P_minus)

            if init==False:
                ang_array = ang_a
                Pos_array = Pos
                Pos_fil = Pos_hat
                # j = 3
                init = True

            
            else:
                ang_array = np.concatenate((ang_array, ang_a), axis=0)

                Pos_array = np.concatenate((Pos_array, Pos), axis=0)
                Pos_fil = np.concatenate((Pos_fil, Pos_hat), axis=0)

            
            # print(f"the shape of pos_array is {Pos} and the ang_array is {ang_a}")
            #
            # if j>200:
            #     Pos_fil = signal.savgol_filter(Pos_array.T, window_length=window_size , polyorder=1, mode="interp")
                
            # else:
            #     Pos_fil = signal.savgol_filter(Pos_array.T, window_length=window_size , polyorder=1, mode="nearest")
            #     # Ang_fil = signal.savgol_filter(ang_array.T, window_length=69, polyorder=1, mode="nearest")

            if j==100 and not start_loco:
                client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
                params = {'translational_stiffness' : 150.0, 'rotational_stiffness' : 25.0}
                config = client.update_configuration(params) 
                ready = 0
            if j==100 and start_loco:
                state = 4

            if j==110 and not start_loco:
                client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
                params = {'translational_stiffness' : 325.0, 'rotational_stiffness' : 25.0}
                config = client.update_configuration(params)
            if j==120 and not start_loco:
                client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
                params = {'translational_stiffness' : 625.0, 'rotational_stiffness' : 25.0}
                config = client.update_configuration(params)
            
            if j==130 and not start_loco:
                client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
                params = {'translational_stiffness' : 1000.0, 'rotational_stiffness' : 25.0}
                config = client.update_configuration(params)
                ready = 1

            if j==130 and start_loco:
                client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
                params = {'translational_stiffness' : 500.0, 'rotational_stiffness' : 30.0}
                config = client.update_configuration(params)
                ready = 1

            if start and j>2 and i>100:
                # print(f"Pos fil shape is {Pos_fil.shape}")

                x_a, y_a, z_a = Pos_fil[-1, :]
                # print(f"Pos fil is made of {Pos_fil[-1, :]}")

                #a, b, c = Ang_fil.T[-1, :]


                # print(f"real number vs filtered are x : {x}&{x_a}, y : {y}&{y_a}, z : {z}&{z_a}")

                PP.pose.position.x = x_a
                PP.pose.position.y = y_a
                PP.pose.position.z = z_a
                
                ang_a = ang_a[0, :]
                
                # ang_a[0] = a
                # ang_a[1] = b
                # ang_a[2] = c

                

                


                
                #(trans,rot) = listener.lookupTransform('franka_a_franka_base_link', 'franka_a_franka_flange', rospy.Time(0))
                # PP.pose.position.x = robot_rf.O_T_EE.position.x
                # PP.pose.position.y = robot_rf.O_T_EE.position.y
                # PP.pose.position.z = robot_rf.O_T_EE.position.z
                # if transform==False:
                #     break
                # q = PP.pose.orientation
                # eu = quaternion_to_euler(q)

                # ang_a = ang_a + np.array([-3.095, -0.225, -0.001])
                # Ang = eu_ro_ref - ang_a

                


                # print(f"angles are around_x {np.rad2deg(ang_a[0])}, around y {np.rad2deg(ang_a[1])}, around z {np.rad2deg(ang_a[2])}")
                


                # ang_o = ang_a.copy()
                # ang_a[0] = -angle_c*ang_o[2]
                # ang_a[2] = angle_c*ang_o[1]
                # ang_a[1] = angle_c*ang_o[0]
                

                # ang_a = ang_a[0, :]
                R_i = euler_to_rotation_matrix(ang_a)

                R_a = euler_to_rotation_matrix([0, 0.0, 0]) ##rotation around x
                R_b = euler_to_rotation_matrix([0, 0, 0]) ##rotation around y

                # Ang = eu_ro_ref
                # print(f"ang_a is {ang_a}")
                # print(f"Ang is {Ang}")
                
                # print(f"reference is {eu_ro_ref}")

                # R_m = np.dot(R_a, R_b)
                # R_t = np.dot(R_i, R_m)
                R_t = np.dot(R_main, R_i)###Important to be edited
                

                q = from_rotation_matrix(R_t)        
                q_t = [q.x, q.y, q.z, q.w]


                # q_t = quaternion_from_euler(ang_a)
                #print(q_t)

                
                









                # PP.pose.orientation.x = robot_rf.O_T_EE.orientation.x + 1*(odom_msg.pose.pose.orientation.x - camera_rf.pose.pose.orientation.x)
                # PP.pose.orientation.y = robot_rf.O_T_EE.orientation.y + 1*(odom_msg.pose.pose.orientation.y - camera_rf.pose.pose.orientation.y)
                # PP.pose.orientation.z = robot_rf.O_T_EE.orientation.z + 1*(odom_msg.pose.pose.orientation.y - camera_rf.pose.pose.orientation.y)
                # PP.pose.orientation.w = robot_rf.O_T_EE.orientation.w + 1*(odom_msg.pose.pose.orientation.w - camera_rf.pose.pose.orientation.w)


                PP.pose.orientation.x = q_t[0]
                PP.pose.orientation.y = q_t[1]
                PP.pose.orientation.z = q_t[2]
                PP.pose.orientation.w = q_t[3]



                PP.header = latest_pose_stamped.header
                # print(f"locomotion is {Locomotion}")
                if Locomotion:
                    pub_a.publish(PP)
                else:
                    pub.publish(PP)
                
                
            # print()




    j+=1


    # Do something with the odometry data, such as printing it or storing it in a data structure
    #print("Received odometry: x={}, y={}, z={}".format(x, y, z))
    #print(f"print pose:{latest_pose_stamped}")
def callback_b(pose):
    # Get the pose data from the message
    # print("hey")
    global latest_pose_stamped
    latest_pose_stamped = pose
    # print("HI")
    # print(latest_pose_stamped.O_T_EE.orientation.x)
    


    # Do something with the pose data, such as printing it or storing it in a data structure
    #print("Received pose: x={}, y={}, z={}".format(x, y, z))
def state_callforward():
    global ready
    global pub_state
    global state
    array_info = Int8MultiArray()
    array_info.data = [ready, state]
    pub_state.publish(array_info)
    

def main():
    global listener
    global pub
    global pub_a
    global odom_reset
    global MODE_TELEOP_CAM
    global admit_checker
    global pub_state
    camera_use = rospy.get_param('/camera_use')

    # camera_use = (camera_use_str.lower() =='true')
    if camera_use:
        MODE_TELEOP_CAM = True
    else:
        MODE_TELEOP_CAM = False

    

    rospy.init_node("pose_stamped_subscriber")
    listener = tf.TransformListener()

    global latest_pose_stamped
    # rospy.wait_for_service("/rtabmap/reset_odom")
    # odom_reset = rospy.ServiceProxy("/rtabmap/reset_odom",Empty)
    # Subscribe to the pose_stamped topic
    rospy.Subscriber("robot_state", RobotState, callback_b)
    if MODE_TELEOP_CAM:
        rospy.wait_for_service("/rtabmap/reset_odom")
        odom_reset = rospy.ServiceProxy("/rtabmap/reset_odom",Empty)
        rospy.Subscriber("/rtabmap/odom", Odometry, callback_a)

    else:
        rospy.Subscriber("/xsens/link_states", LinkStateArray, callback_c)

    # rospy.Subscriber("/moca_red/ATI45_ft_handler/wrench",WrenchStamped, force_read)
    # pub = rospy.Publisher('/pose_stamped', PoseStamped, queue_size=10)
    # Keep the node running until it is shut down
    pub_state = rospy.Publisher('/teleop_state', Int8MultiArray,  queue_size=10)
    pub = rospy.Publisher('cartesian_impedance_controller/equilibrium_pose', PoseStamped, queue_size=10)
    service = rospy.Service('/start_teleop', Trigger, Teleop_start)
    service_a = rospy.Service('/end_teleop', Trigger, Teleop_end)
    service_b = rospy.Service('/homing_teleop', Trigger, Teleop_homing)
    service_c = rospy.Service('/loco_check', Trigger, loco)
    service_d = rospy.Service('/loco_mode', SetBool, Teleop_loco)



    pub_a = rospy.Publisher('pos_to_force', PoseStamped, queue_size=10)
    # service_c = rospy.Service('/status_teleop', Trigger, Tele_stat)
    
    rospy.wait_for_service('variable_admittance_controller/admittance_status')
    admit_checker = rospy.ServiceProxy('variable_admittance_controller/admittance_status', Trigger)
    rospy.spin()


if __name__ == "__main__":
    main()

