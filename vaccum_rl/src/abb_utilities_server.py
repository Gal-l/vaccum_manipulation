#!/usr/bin/env python
import sys
import copy
import rospy
import numpy as np
import moveit_commander
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list
from vaccum_msgs.srv import HomeCommand, HomeCommandResponse
from std_msgs.msg import Int8
from config import V_Params


def all_close(goal, actual, tolerance):
    # type: (object, object, object) -> object
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


class Homming_robot:

    def __init__(self):

        self.service = rospy.Service('home_robot', HomeCommand, self.server_callback)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "manipulator"

        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.pub_tool_changer = rospy.Publisher('tool_changer', Int8, queue_size=20)

        self.pub_main_valve = rospy.Publisher('main_valve', Int8, queue_size=20)

        self.v_params = V_Params()

    def server_callback(self, req):
        print (req)
        if req.command == "home":
            self.go_to_home_state()
        if req.command == "zero":
            self.go_to_zero_state()
        if req.command == "v":
            self.pub_main_valve.publish(0)
        if req.command == "t":
            self.pub_tool_changer.publish(0)
            print ("remove all tools")
            self.scene.remove_attached_object('tool0', 'glue_gripper')
            self.scene.remove_attached_object('tool0', 'vaccum_gripper')
            self.scene.remove_attached_object('tool0', 'fingers_gripper')
        if req.command == "t1":
            self.pub_tool_changer.publish(1)
        if req.command == "pose":
            print(self.move_group.get_current_pose().pose)
        return HomeCommandResponse("Your Robot is Home and Ready")

    def go_to_home_state(self):

        print("new new")

        move_group = self.move_group
        group = self.move_group

        # We can get the joint values from the group and adjust some of the values:
        joint_goal = self.v_params.home_pose_joints
        move_group.go(joint_goal, wait=True)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.move_group.get_current_joint_values()

        print ("Your Robot is Home and Ready")

        return all_close(joint_goal, current_joints, 0.01)

    def go_to_zero_state(self):

        print("new new")

        group = self.move_group

        # We can get the joint values from the group and adjust some of the values:
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        # joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.move_group.get_current_joint_values()

        print ("Your Robot is Home and Ready")

        return all_close(joint_goal, current_joints, 0.01)


def main():
    rospy.init_node('home_robot', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    control = Homming_robot()

    rate = rospy.Rate(1)  # 10hz

    while not rospy.is_shutdown():
        # print("wait for orders")

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass