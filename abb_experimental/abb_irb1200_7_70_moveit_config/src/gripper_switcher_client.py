#!/usr/bin/env python

from __future__ import print_function
from services.srv import Grippers_switcher
import sys
import rospy

def load_gripper(g):
    rospy.wait_for_service('gripper_switcher')
    try:
        gripper_service = rospy.ServiceProxy('gripper_switcher', Grippers_switcher)
        resp = gripper_service(g)
        print(resp.ans)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def mission():
    
    # 1 - vaccum , 2 - fingers , 3 -stick

    load_gripper(1)

    load_gripper(3)

    load_gripper(2)


if __name__ == "__main__":
    
    mission()