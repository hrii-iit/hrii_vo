#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse, SetBool
from hrii_utils.srv import SetData
import dynamic_reconfigure.client
import math
import numpy as np
import time

#will be read by topic or service
global Loco_tele
Loco_tele = False
global init
init = False
global once
once = False
global horiz
horiz = False
global vert
vert = False

global tele_vo
tele_vo = False


def quaternion_to_euler(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
    pitch = math.asin(2 * (q.w * q.y - q.z * q.x))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
    return np.array([roll, pitch, yaw])

def Vis_indicator(req):
    global Loco_tele
    global tele_vo
    print(Loco_tele)
    print("AN")
    respose = TriggerResponse()

    #Different Homing for Locomotion must be set 
    # admit_cont = admitance_cont(False)
    # home = homing_srv()
    
    admit_cont = admitance_cont(True)
    # admit = admit_checker()
    vo_enb = vo_active()
    vo = vo_checker()
    
    locom = loco_mode(True)

    prio = priority(1) # Turn Locomotion ON
    # motion = motion_mode(0) # Turn RotroTranslation On
    client = dynamic_reconfigure.client.Client("dynamic_reconfigure_compliance_param_node")
    params = {'translational_stiffness' : 325.0, 'rotational_stiffness' : 25.0}
    config = client.update_configuration(params)

    
    
    if vo:
        tele_vo = True
        Loco_tele = True

    respose.success = True
    respose.message = f"locomotion is activated {Loco_tele}"
    # print(f"admittance status is {res}")
    return respose
    # return tele_vo

def vis_indic(req):
    global Loco_tele
    global tele_vo

    res = TriggerResponse()
    admit_cont = admitance_cont(False)
    # home = homing_srv()
    prio = priority(0)
    vo_enb = vo_active()
    vo = vo_checker()
    Loco_tele = False
    
    locom = loco_mode(False)
    # motion = motion_mode(0)
    # if vo.success:
    #     Loco_tele = True
    # else:
    #     print(f"visual odometry is {vo}")
    # # admit_cont = admitance_cont("False")

    res.success = True
    res.message = f"Manipulation is activated {Loco_tele}"
    return res


    

def Vis_off(req):
    global tele_vo
    tele_vo = False
    # print


def admittance_check(req):
    global admit
    admit = True


def force_read(F_msg):
    global Loco_tele
    Force = WrenchStamped()
    global L
    global first
    L = F_msg
    
    
    # if tele_vo and admit:
    #     Force   
    # trigger_service = rospy.ServiceProxy('/moca_red/variable_admittance_controller/admittance_status ', Trigger)
    # print(Force.wrench)
    if Loco_tele==False:
        first = True
        Force = F_msg
        pub.publish(Force)
    
  
 

def pos_to_force(X_msg):
    global Loco_tele
    Force = WrenchStamped()
    global L
    global first
    global horiz
    global vert
    global x_i
    global y_i
    global t_z_i
    global once
    global A
    global init


    # elif admit:

    Force.header = L.header
    # print(Force)

    if Loco_tele:
        
        if first:
            x_i = X_msg.pose.position.x
            y_i = X_msg.pose.position.y
            t_x_i, t_y_i, t_z_i = quaternion_to_euler(X_msg.pose.orientation)
        x_c = X_msg.pose.position.x 
        y_c = X_msg.pose.position.y
        z = 0
        t_x, t_y, t_z_c = quaternion_to_euler(X_msg.pose.orientation)
        t_x = 0
        t_y = 0
        x = x_c - x_i
        y = y_c - y_i
        t_z = t_z_c - t_z_i
        # if abs(x)>0.1:
        #     Force.wrench.force.x = 3.5*(x/abs(x))
        # if abs(y)>0.1:
        #     Force.wrench.force.y = 3.5*(y/abs(y))
        if abs(t_z)>0.35:
            Force.wrench.torque.z = 1.8*(t_z/abs(t_z))
        if np.sqrt(x**2 + y**2)>0.10 or abs(t_z)>0.34:
            
            message = String()
            if once==False:
                message.data = 'base'
                pub_mode.publish(message)

            if abs(x)>abs(y):
                Force.wrench.force.x = 6.0*4.7*(abs(x))*(x/abs(x))
                # vert = True
            elif abs(x)<abs(y) or abs(x)==abs(y):
                Force.wrench.force.y = 6.0*4.7*(abs(y))*(y/abs(y))
                # horiz = True
            once = True
        else:
            if np.sqrt(x**2 + y**2)<0.099999999 and abs(t_z)<0.34 and not init and once:
                A = time.time()
                init = True
            
        if init:
            if time.time()-A>0.5:
                # admit_cont = admitance_cont(False)
                message = String()
                message.data = 'base_stop'
                pub_mode.publish(message)

                #stop policy
                once = False
                init = False
                # horiz = False
                # vert = False
                x_i = X_msg.pose.position.x
                y_i = X_msg.pose.position.y
                t_x_i, t_y_i, t_z_i = quaternion_to_euler(X_msg.pose.orientation)
                time.sleep(0.5)
                # admit_cont = admitance_cont(True)
        # trans_const = 20
        # ang_const = 15
        # Force.wrench.force.x = x*trans_const
        # Force.wrench.force.y = y*ang_const
        Force.wrench.force.z = z
        Force.wrench.torque.x = t_x
        Force.wrench.torque.y = t_y
        # Force.wrench.torque.z = t_z

        first = False
        
        pub.publish(Force)

        


def main():
    global pub
    global service_n
    global service_m
    global pub_mode
    # global admit_checker
    global motion_mode
    global vo_checker
    global loco_mode
    global priority
    global vo_active
    global admitance_cont
    global homing_srv

    rospy.init_node("force_mapper")
    pub = rospy.Publisher("ATI45_ft_handler/wrench_a", WrenchStamped, queue_size=10)
    pub_mode = rospy.Publisher('cartesian_impedance_controller/weighted_jacobian', String, queue_size=10)
    # if Loco_tele==False:
    rospy.Subscriber("ATI45_ft_handler/wrench", WrenchStamped, force_read)
    # if Loco_tele:
    rospy.Subscriber("pos_to_force", PoseStamped, pos_to_force)
    # rospy.wait_for_service('/moca_red/variable_admittance_controller/admittance_status')
    rospy.wait_for_service('variable_admittance_controller/activate')
    
    rospy.wait_for_service('super_man/motion_mode')
    rospy.wait_for_service('super_man/priority_mode')
    # admit_checker = rospy.ServiceProxy('/moca_red/variable_admittance_controller/admittance_status', Trigger)
    admitance_cont = rospy.ServiceProxy('variable_admittance_controller/activate', SetBool)
    motion_mode = rospy.ServiceProxy('super_man/motion_mode', SetData)
    priority = rospy.ServiceProxy('super_man/priority_mode', SetData)
    service_n = rospy.Service('/vo_loco', Trigger, Vis_indicator)
    service_m = rospy.Service('/vo_mani', Trigger, vis_indic)
    # if tele_vo:
    #    rospy.Subscriber('/moca_red/cartesian_impedance_controller/equilibrium_pose', PoseStamped, callback_a)

    
    print("Hi")
    rospy.wait_for_service('/loco_mode')
    loco_mode = rospy.ServiceProxy('/loco_mode', SetBool)  
    rospy.wait_for_service('/loco_check')
    vo_checker = rospy.ServiceProxy('/loco_check', Trigger)
    rospy.wait_for_service('/start_teleop')
    vo_active = rospy.ServiceProxy('/start_teleop', Trigger)
    rospy.wait_for_service('/homing_teleop')
    homing_srv = rospy.ServiceProxy('/homing_teleop', Trigger)

    rospy.Subscriber('cartesian_impedance_controller/equilibrium_pose', PoseStamped)
    rospy.spin()
















if __name__ == "__main__":
    main()
    # rospy.init_node("force_mapper")
    # rospy.Subscriber("/moca_red/ATI45_ft_handler/wrench", WrenchStamped, force_read)
    # rospy.Subscriber("/rtabmap/odom", Odometry, callback_a)
    # pub = rospy.Publisher("/moca_red/ATI45_ft_handler/wrench_a", WrenchStamped, queue_size=10)

    # rospy.spin()
