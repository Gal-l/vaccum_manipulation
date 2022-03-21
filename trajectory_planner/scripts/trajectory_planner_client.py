#!/usr/bin/env python

from __future__ import print_function
from pickommerce_msgs.srv import Grippers_switcher , PoseGoal
import sys
from geometry_msgs.msg import PoseArray , Pose
import rospy
import copy

# 1 - vaccum , 3 - fingers , 2 -stick

posearray = PoseArray()

pos1  = Pose() 

pos1.position.x , pos1.position.y , pos1.position.z = 0.0215, -0.561,  0.529

pos1.orientation.x , pos1.orientation.y , pos1.orientation.z , pos1.orientation.w = -0.399, -0.916, -0.0284,  0.0076

z_offset , y_offset = 0.03 , 0.16


pos2= copy.deepcopy(pos1)

pos3 = copy.deepcopy(pos1) 

pos2.position.z+=z_offset

pos3.position.y+=y_offset

posearray.poses.extend([pos1,pos2,pos3])

posearray.header.frame_id = "base_link"



def SendWayPoints(pose_array):
    rospy.wait_for_service('trajectory_planner')
 
    try:
        trajectory_planner_service = rospy.ServiceProxy('trajectory_planner', PoseGoal)
        resp = trajectory_planner_service(pose_array)
        print(resp.ans)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def mission():
    
    SendWayPoints(posearray)

  



    




if __name__ == "__main__":
    
    mission()