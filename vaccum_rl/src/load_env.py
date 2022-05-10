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
from visualization_msgs.msg import Marker

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
        self.cage_marker_publisher = rospy.Publisher('Cage_Marker', Marker, queue_size=1)

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


        self.add_gripper_to_scene()
        rospy.sleep(0.5)
        self.attach_vaccum_gripper_mesh()
        rospy.sleep(0.5)
        self.add_marker_cage()


    def add_cage_to_scene(self):

        cage_pose = PoseStamped()

        cage_pose.header.frame_id = "base_link"

        cage_pose.pose.orientation.x, cage_pose.pose.orientation.y, cage_pose.pose.orientation.z, cage_pose.pose.orientation.w = 0.0005629, -0.707388, -0.706825, 0.0005633

        cage_pose.pose.position.x, cage_pose.pose.position.y, cage_pose.pose.position.z = 1.02, -1.71, -0.74

        cage_name = "cage"

        pkg_path = rospkg.RosPack().get_path('vaccum_rl')

        rospy.sleep(0.5)

        self.scene.add_mesh(cage_name, cage_pose, '{}/meshes/cage.stl'.format(pkg_path), (0.001, 0.001, 0.001))

    def add_marker_cage(self):

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.MESH_RESOURCE
        marker.action = marker.ADD

        scale = 0.001 # m to mm
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = "package://vaccum_rl/meshes/cage.stl"

        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = 0.0005629, -0.707388, -0.706825, 0.0005633

        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = 1.02, -1.71, -0.74

        marker.id = 0

        print ("Publish Marker")

        self.cage_marker_publisher.publish(marker)

        rospy.sleep(0.5)

    def add_gripper_to_scene(self):

        vaccum_gripper_mesh_pose = PoseStamped()

        vaccum_gripper_mesh_pose.header.frame_id = "base_link"

        vaccum_gripper_mesh_pose.pose.orientation.x, vaccum_gripper_mesh_pose.pose.orientation.y, vaccum_gripper_mesh_pose.pose.orientation.z, \
        vaccum_gripper_mesh_pose.pose.orientation.w = 0.5535217, -0.5586397, 0.438775, 0.4347551 # 0.5584857, -0.5580411, 0.433811, 0.4341565

        vaccum_gripper_mesh_pose.pose.position.x, vaccum_gripper_mesh_pose.pose.position.y,\
        vaccum_gripper_mesh_pose.pose.position.z = 0.83 + 0.02, 0.10, 0.76 # 0.75, 0.13, 0.76



        vaccum_name = "vaccum_gripper"

        pkg_path = rospkg.RosPack().get_path('vaccum_rl')

        rospy.sleep(0.5)

        self.scene.add_mesh(vaccum_name,vaccum_gripper_mesh_pose,'{}/meshes/vaccum_box.stl'.format(pkg_path),(0.001,0.001,0.001))

    def attach_vaccum_gripper_mesh(self):

        # grasping_group = self.group_names
        #
        # touch_links = self.robot.get_link_names(group=grasping_group)

        touch_links = 'tool0'

        self.scene.attach_mesh('tool0', "vaccum_gripper", touch_links=touch_links)

        rospy.sleep(0.5)

def main():
    try:

        tutorial = MoveGroupPythonIntefaceTutorial()
        print "Done upload the environment"
        exit()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
    exit()
