#!/usr/bin/env python
import numpy as np
import sys
import copy
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import ros_numpy
from moveit_msgs.msg import RobotState ,RobotTrajectory
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import pickle
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from std_msgs.msg import Float32MultiArray
import open3d as o3d

def all_close(goal, actual, tolerance):
    """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        self.pcd_array = None
        self.theta = None

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('load_env', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        # define ABB ROBOT - ABB - irb1200_7_70
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

                # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        # print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        # print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # print "============ Available Planning Groups:", robot.get_group_names()

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.add_cage_to_scene()

    def add_cage_to_scene(self):

        cage_pose = PoseStamped()

        cage_pose.header.frame_id = "base_link"

        cage_pose.pose.orientation.x, cage_pose.pose.orientation.y, cage_pose.pose.orientation.z, cage_pose.pose.orientation.w = 0.0005629, -0.707388, -0.706825, 0.0005633

        cage_pose.pose.position.x, cage_pose.pose.position.y, cage_pose.pose.position.z = 1.02, -1.71, -0.74

        cage_name = "cage"

        pkg_path = rospkg.RosPack().get_path('initial_path')

        rospy.sleep(0.5)

        self.scene.add_mesh(cage_name, cage_pose, '{}/meshes/cage.stl'.format(pkg_path), (0.001, 0.001, 0.001))

def main():
    try:

        tutorial = MoveGroupPythonIntefaceTutorial()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    while True:
        main()
