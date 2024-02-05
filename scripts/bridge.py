#! /usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int8MultiArray
from std_srvs.srv import Trigger


def button_callback(msg):
    global D
    global homing
    D = msg.data
    global P
    global S
    try: 
        A = P
    except:
        P = D
    
    print(f"the given msg is {D[1]}")

    if D[4]!=P[4]:
        if S[1] == 1 or S[1] == 2:
            homing_srv()
            homing = True
        if S[1] == 3 and not homing:
            loco()
        if S[1] == 4 and not homing:
            mani()
        if homing:
            homing = False
    
            
        
    print("Here")
    if D[5]!=P[5]:
        print("HIA")
        if S[1] == 1:
            print("HI")
            vo_deactive()
        else:
            vo_active()

        

    







    P = D
def state_callback(msg):
    global S
    S = msg.data
    



def main():
    global homing_srv
    global vo_deactive
    global vo_active
    global loco
    global mani
    rospy.init_node("bridge")
    rospy.Subscriber("/moca_red/touch_buttons", Int8MultiArray ,button_callback)
    rospy.wait_for_service('/homing_teleop')
    homing_srv = rospy.ServiceProxy('/homing_teleop', Trigger)
    rospy.wait_for_service('/start_teleop')
    vo_active = rospy.ServiceProxy('/start_teleop', Trigger)
    rospy.wait_for_service('/end_teleop')
    vo_deactive = rospy.ServiceProxy('/end_teleop', Trigger)

    rospy.wait_for_service('/vo_loco')
    loco = rospy.ServiceProxy('/vo_loco', Trigger)
    rospy.wait_for_service('/vo_mani')
    mani = rospy.ServiceProxy('/vo_mani', Trigger)
    rospy.Subscriber("/teleop_state", Int8MultiArray, state_callback)

    rospy.spin()






if __name__ == "__main__":
    main()
