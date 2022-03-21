#!/usr/bin/env python

from __future__ import print_function
from pickommerce_msgs.srv import Grippers_switcher
import sys
import rospy

# 1 - vaccum , 3 - fingers , 2 -stick

def load_gripper(g):
    rospy.wait_for_service('gripper_switcher')
    try:
        gripper_service = rospy.ServiceProxy('gripper_switcher', Grippers_switcher)
        resp = gripper_service(g)
        print(resp.ans)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def mission():

    load_gripper(1)

    load_gripper(2)

    load_gripper(3)

    load_gripper(1)    

   
    


    




if __name__ == "__main__":
    
    mission()