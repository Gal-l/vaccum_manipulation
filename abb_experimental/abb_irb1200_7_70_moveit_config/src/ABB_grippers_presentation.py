#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import Int8
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose , Point
import keyboard
from tf.transformations import quaternion_matrix
import tf
import numpy as np

class ABB_Controll:



  def __init__(self):

    self.robot = moveit_commander.RobotCommander()

    self.scene = moveit_commander.PlanningSceneInterface()

    self.group_name = "manipulator"

    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)



    self.pub_change_gripper = rospy.Publisher('gripper_change',Int8,queue_size=20)
        
    self.tf_listener = tf.TransformListener()

    self.pose_goal_sub = rospy.Subscriber('pose_goal', Pose, self.pose_goal_callback)

    self.tcp_movement_sub = rospy.Subscriber('tcp_movement', Point, self.tcp_movement_callback)

    self.trans , self.rot = [0,0,0] , [0,0,0,0]

    self.pose_goal=Pose()

    self.home_mode , self.state_mode = 1 , 1

    self.ohm_pose , self.vaccum_gripper_in_pose , self.vaccum_gripper_prep_pose , self.vaccum_gripper_out_pose , self.vaccum_gripper_outside_pose  = Pose() , Pose() , Pose() , Pose()  , Pose()

    self.fingers_gripper_in_pose , self.fingers_gripper_prep_pose , self.fingers_gripper_out_pose , self.fingers_gripper_outside_pose  = Pose() , Pose() , Pose() , Pose()

    self.stick_gripper_in_pose , self.stick_gripper_prep_pose , self.stick_gripper_out_pose , self.stick_gripper_outside_pose  = Pose() , Pose() , Pose() , Pose()



    self.vaccum_gripper_in_pose.position.x , self.vaccum_gripper_in_pose.position.y , self.vaccum_gripper_in_pose.position.z = -0.20234 , -0.569 , 0.5319

    self.vaccum_gripper_in_pose.orientation.x , self.vaccum_gripper_in_pose.orientation.y , self.vaccum_gripper_in_pose.orientation.z , self.vaccum_gripper_in_pose.orientation.w = 0.7816 , 0.6236, 0.0036 , 0.00562

    self.vaccum_gripper_prep_pose.position.x , self.vaccum_gripper_prep_pose.position.y , self.vaccum_gripper_prep_pose.position.z = -0.20234 , -0.569 , 0.565

    self.vaccum_gripper_prep_pose.orientation.x , self.vaccum_gripper_prep_pose.orientation.y , self.vaccum_gripper_prep_pose.orientation.z , self.vaccum_gripper_prep_pose.orientation.w = 0.7816 , 0.6236, 0.0036 , 0.00562

    self.vaccum_gripper_out_pose.position.x , self.vaccum_gripper_out_pose.position.y , self.vaccum_gripper_out_pose.position.z = -0.20234 , -0.5 , 0.5319

    self.vaccum_gripper_out_pose.orientation.x , self.vaccum_gripper_out_pose.orientation.y , self.vaccum_gripper_out_pose.orientation.z , self.vaccum_gripper_out_pose.orientation.w = 0.7816 , 0.6236, 0.0036 , 0.00562

    self.vaccum_gripper_outside_pose.position.x , self.vaccum_gripper_outside_pose.position.y , self.vaccum_gripper_outside_pose.position.z = -0.20234 , -0.3 , 0.5319

    self.vaccum_gripper_outside_pose.orientation.x , self.vaccum_gripper_outside_pose.orientation.y , self.vaccum_gripper_outside_pose.orientation.z , self.vaccum_gripper_outside_pose.orientation.w = 0.7816 , 0.6236, 0.0036 , 0.00562
    

    
    self.ohm_pose.position.x , self.ohm_pose.position.y , self.ohm_pose.position.z = 0.026 , -0.342 , 0.6288

    self.ohm_pose.orientation.x , self.ohm_pose.orientation.y , self.ohm_pose.orientation.z , self.ohm_pose.orientation.w = 0.8163 , 0.577 , -0.01436 , 0.0133


    self.fingers_gripper_in_pose.position.x , self.fingers_gripper_in_pose.position.y , self.fingers_gripper_in_pose.position.z = 0.02465 , -0.5647 , 0.528

    self.fingers_gripper_in_pose.orientation.x , self.fingers_gripper_in_pose.orientation.y , self.fingers_gripper_in_pose.orientation.z , self.fingers_gripper_in_pose.orientation.w = 0.783 , 0.622 , 0.00347 , -0.00561

    self.fingers_gripper_prep_pose.position.x , self.fingers_gripper_prep_pose.position.y , self.fingers_gripper_prep_pose.position.z = 0.02465 , -0.5647 , 0.565

    self.fingers_gripper_prep_pose.orientation.x , self.fingers_gripper_prep_pose.orientation.y , self.fingers_gripper_prep_pose.orientation.z , self.fingers_gripper_prep_pose.orientation.w = 0.783 , 0.622 , 0.00347 , -0.00561

    self.fingers_gripper_out_pose.position.x , self.fingers_gripper_out_pose.position.y , self.fingers_gripper_out_pose.position.z = 0.0247, -0.466, 0.532

    self.fingers_gripper_out_pose.orientation.x , self.fingers_gripper_out_pose.orientation.y , self.fingers_gripper_out_pose.orientation.z , self.fingers_gripper_out_pose.orientation.w = 0.783, 0.621, 0.00379, -0.0054

    self.fingers_gripper_outside_pose.position.x , self.fingers_gripper_outside_pose.position.y , self.fingers_gripper_outside_pose.position.z = 0.02492 , -0.3 , 0.5311

    self.fingers_gripper_outside_pose.orientation.x , self.fingers_gripper_outside_pose.orientation.y , self.fingers_gripper_outside_pose.orientation.z , self.fingers_gripper_outside_pose.orientation.w = 0.788 , 0.615 , 0.00512 , 0.00274

    self.stick_gripper_in_pose.position.x , self.stick_gripper_in_pose.position.y , self.stick_gripper_in_pose.position.z = 0.2697 , -0.56117 , 0.5268

    self.stick_gripper_in_pose.orientation.x , self.stick_gripper_in_pose.orientation.y , self.stick_gripper_in_pose.orientation.z , self.stick_gripper_in_pose.orientation.w = 0.7829 , 0.622 , 0.00334 , -0.005168

    self.stick_gripper_prep_pose.position.x , self.stick_gripper_prep_pose.position.y , self.stick_gripper_prep_pose.position.z = 0.2697 , -0.56117 , 0.565

    self.stick_gripper_prep_pose.orientation.x , self.stick_gripper_prep_pose.orientation.y , self.stick_gripper_prep_pose.orientation.z , self.stick_gripper_prep_pose.orientation.w = 0.7829 , 0.622 , 0.00334 , -0.005168

    self.stick_gripper_out_pose.position.x , self.stick_gripper_out_pose.position.y , self.stick_gripper_out_pose.position.z = 0.2697 , -0.5 , 0.5268

    self.stick_gripper_out_pose.orientation.x , self.stick_gripper_out_pose.orientation.y , self.stick_gripper_out_pose.orientation.z , self.stick_gripper_out_pose.orientation.w = 0.7829 , 0.622 , 0.00334 , -0.005168

    self.stick_gripper_outside_pose.position.x , self.stick_gripper_outside_pose.position.y , self.stick_gripper_outside_pose.position.z = 0.2697 , -0.3 , 0.5268

    self.stick_gripper_outside_pose.orientation.x , self.stick_gripper_outside_pose.orientation.y , self.stick_gripper_outside_pose.orientation.z , self.stick_gripper_outside_pose.orientation.w = 0.7829 , 0.622 , 0.00334 , -0.005168




    





  def tcp_movement_callback(self,data):

    self.tcp_movement=[data.x,data.y,data.z,1]

    self.plan_movement_to_gripper(self.tcp_movement)


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


  def pose_to_matrix(self,p):

    """geometry_msgs.msg.Pose to 4x4 matrix"""

    t_matrix = tf.transformations.translation_matrix([p.position.x, p.position.y, p.position.z])

    r_matrix = tf.transformations.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    return np.dot(t_matrix, r_matrix) 




    

  def plan_movement_to_gripper(self,tcp_movement):

    tf_pose = Pose()

    tf_pose.position.x ,tf_pose.position.y ,tf_pose.position.z = self.trans[0] ,self.trans[1] ,self.trans[2]

    tf_pose.orientation.x ,tf_pose.orientation.y ,tf_pose.orientation.z ,tf_pose.orientation.w = self.rot[0] ,self.rot[1] ,self.rot[2] ,self.rot[3]

    tf_matrix_from_gripper_to_base=self.pose_to_matrix(tf_pose)

    point_to_move_in_base_axes=np.dot(tf_matrix_from_gripper_to_base,tcp_movement)

    self.pose_goal.position.x ,self.pose_goal.position.y, self.pose_goal.position.z = point_to_move_in_base_axes[0] ,point_to_move_in_base_axes[1] ,point_to_move_in_base_axes[2]

    self.pose_goal.orientation.x , self.pose_goal.orientation.y , self.pose_goal.orientation.z , self.pose_goal.orientation.w = self.rot[0] ,self.rot[1] ,self.rot[2] ,self.rot[3]

    print(self.pose_goal)

    self.planning()





  def fingers_gripper(self):

  

    if(self.state_mode == 1):

      self.pose_goal = self.fingers_gripper_prep_pose

    elif(self.state_mode == 2):

      self.pose_goal = self.fingers_gripper_in_pose

    elif(self.state_mode == 3):

      self.pose_goal = self.fingers_gripper_out_pose

    elif(self.state_mode == 3):

      self.pose_goal = self.fingers_gripper_outside_pose

    print(self.pose_goal)





    




  def move_to_home_pose(self):

    self.pose_goal = self.ohm_pose







  def presentation_script(self):

    # if(self.home_mode):

    #   self.move_to_ohm_pose()

    #   self.home_mode = 0

    # else:


    self.fingers_gripper()


















def main():
    
    rospy.init_node('ABB_controll', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    

    controll=ABB_Controll()

    rate = rospy.Rate(10) # 10hz
    

    
    
    while not rospy.is_shutdown():

    

      #(control.trans,control.rot) = control.tf_listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))

      if((keyboard.is_pressed('ctrl') and keyboard.is_pressed('a'))):

        print(" trans  ",control.trans,"    rot   ",control.rot) 

      if((keyboard.is_pressed('ctrl') and keyboard.is_pressed('l'))):

        print(controll.pose_goal)

        controll.planning()

        controll.show_plan()

      if((keyboard.is_pressed('ctrl') and keyboard.is_pressed('e'))):

        controll.execute()

        controll.state_mode += 1

      controll.presentation_script()

      rate.sleep()






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass