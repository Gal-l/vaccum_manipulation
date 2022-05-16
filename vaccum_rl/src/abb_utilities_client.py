#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list
from vaccum_msgs.srv import HomeCommand, HomeCommandResponse
from std_msgs.msg import Int8

# !/usr/bin/env python

import rospy
from vaccum_msgs.srv import HomeCommand


def mission(str_msg):
    if str_msg == "exit":
        return 0

    rospy.wait_for_service('home_robot')

    try:
        home_robot_service = rospy.ServiceProxy('home_robot', HomeCommand)
        resp = home_robot_service(str_msg)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    str_msg = True
    print("home - send the arm to home position \nv  - close the main value \nt - release the gripper \n"
          "t1 - connect the gripper \npose - print the end effector pose \nexit - exit the client \ng - return the gripper and go home")
    while str_msg != "exit":
        str_msg = raw_input("Enter command: ")
        mission(str_msg)
