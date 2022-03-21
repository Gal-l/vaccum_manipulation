#!/usr/bin/env python

from __future__ import print_function
from pickommerce_msgs.srv import ClusterList

import sys
import rospy

def load_gripper():
    rospy.wait_for_service('get_cluster')
    try:
        service = rospy.ServiceProxy('get_cluster', ClusterList)
        resp = service()
        print(resp.data.__len__())
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def mission():
    
    # 1 - vaccum , 3 - fingers , 2 -stick

    load_gripper()


if __name__ == "__main__":
    
    mission()
