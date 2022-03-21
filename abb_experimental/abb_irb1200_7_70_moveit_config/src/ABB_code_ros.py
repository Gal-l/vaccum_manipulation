#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
import keyboard


class ABB_Controll:



  def __init__(self):

    self.robot = moveit_commander.RobotCommander()

    self.scene = moveit_commander.PlanningSceneInterface()

    self.group_name = "manipulator"

    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    self.pose_goal_sub = rospy.Subscriber('pose_goal', Pose, self.pose_goal_callback)

    self.pose_goal=Pose()


  def pose_goal_callback(self,data):

    self.pose_goal=data

    self.planning()


  def display_trajectory(self,plan):

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = self.robot.get_current_state()

    display_trajectory.trajectory.append(self.plan)

    self.display_trajectory_publisher.publish(display_trajectory)


  def planning(self):

    self.move_group.set_pose_target(self.pose_goal)

    self.plan = self.move_group.plan()

    self.display_trajectory(self.plan)



  def show_plan(self):

    self.display_trajectory(self.plan)



  def execute(self):
    

    self.move_group.execute(self.plan,wait=True)

    self.move_group.stop()

    self.move_group.clear_pose_targets()







    







def main():
    
    rospy.init_node('ABB_controll', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    

    control=ABB_Controll()

    rate = rospy.Rate(10) # 10hz
    
    
    while not rospy.is_shutdown():

      if((keyboard.is_pressed('ctrl') and keyboard.is_pressed('l'))):

        control.show_plan()

      if((keyboard.is_pressed('ctrl') and keyboard.is_pressed('e'))):

        control.execute()



        

      rate.sleep()






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass