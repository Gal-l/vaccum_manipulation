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
import time
from pickommerce_msgs.srv import Grippers_switcher,Grippers_switcherResponse

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

    self.pub_air_open = rospy.Publisher('air_open',Int8,queue_size=20)

    self.pub_air_close = rospy.Publisher('air_close',Int8,queue_size=20)

    self.pub_24v = rospy.Publisher('v24',Int8,queue_size=20)
        
    self.pose_goal=Pose()

    



  def display_trajectory(self,plan):

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = self.robot.get_current_state()

    display_trajectory.trajectory.append(self.plan)

    self.display_trajectory_publisher.publish(display_trajectory)




  def show_plan(self):

    self.display_trajectory(self.plan)



  def execute(self):

    self.move_group.execute(self.plan,wait=True)

    self.move_group.stop()

    self.move_group.clear_pose_targets()



  



  def plan_cartesian_path(self,goal_pose):

    waypoints = []

    wpose = self.move_group.get_current_pose().pose

    wpose.position.x += goal_pose.position.x - wpose.position.x

    wpose.position.y += goal_pose.position.y - wpose.position.y

    wpose.position.z += goal_pose.position.z - wpose.position.z

    wpose.orientation = goal_pose.orientation

    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0) 

    return plan


  def open_finger_gripper(self):

    self.pub_air_open.publish(1)

    self.pub_air_close.publish(0)

  def close_finger_gripper(self):

    self.pub_air_open.publish(0)

    self.pub_air_close.publish(1)

  def fingers_gripper_default(self):

    self.pub_air_open.publish(0)

    self.pub_air_close.publish(0)


  def vaccum_gripper_close(self):

    self.pub_air_open.publish(1)

  def vaccum_gripper_open(self):

    self.pub_air_open.publish(0)

  def stick_gripper_open(self):

    self.pub_air_open.publish(0)

    self.pub_air_close.publish(1)

  def stick_gripper_close(self):

    self.pub_air_open.publish(1)

    self.pub_air_close.publish(0)

  def stick_gripper_refill_glue(self):

    self.pub_24v.publish(0)

    time.sleep(2.5)

    self.pub_24v.publish(1)

  def stick_gripper_default(self):

    self.pub_air_close.publish(0)

    self.pub_air_open.publish(0)



    




class Gripper_switcher:

  def __init__(self,controll):

    self.controll=controll

    self.server = rospy.Service('gripper_switcher', Grippers_switcher, self.load_gripper)

    self.ohm_pose , self.vaccum_gripper_in_pose , self.vaccum_gripper_prep_pose , self.vaccum_gripper_out_pose , self.vaccum_gripper_to_object_pose , self.vaccum_gripper_grab_object_pose  = Pose() , Pose() , Pose() , Pose()  , Pose() , Pose()

    self.fingers_gripper_in_pose , self.fingers_gripper_prep_pose , self.fingers_gripper_out_pose , self.fingers_gripper_to_object_pose , self.fingers_gripper_grab_object_pose  = Pose() , Pose() , Pose() , Pose(), Pose()

    self.stick_gripper_in_pose , self.stick_gripper_prep_pose , self.stick_gripper_out_pose , self.stick_gripper_to_object_pose , self.stick_gripper_grab_object_pose = Pose() , Pose() , Pose() , Pose(), Pose()



    self.vaccum_gripper_in_pose.position.x , self.vaccum_gripper_in_pose.position.y , self.vaccum_gripper_in_pose.position.z = 0.0311 , -0.56 , 0.528

    self.vaccum_gripper_in_pose.orientation.x , self.vaccum_gripper_in_pose.orientation.y , self.vaccum_gripper_in_pose.orientation.z , self.vaccum_gripper_in_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117

    self.vaccum_gripper_prep_pose.position.x , self.vaccum_gripper_prep_pose.position.y , self.vaccum_gripper_prep_pose.position.z = 0.0311 , -0.56 , 0.607

    self.vaccum_gripper_prep_pose.orientation.x , self.vaccum_gripper_prep_pose.orientation.y , self.vaccum_gripper_prep_pose.orientation.z , self.vaccum_gripper_prep_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117

    self.vaccum_gripper_out_pose.position.x , self.vaccum_gripper_out_pose.position.y , self.vaccum_gripper_out_pose.position.z = 0.0311 , -0.4 , 0.528

    self.vaccum_gripper_out_pose.orientation.x , self.vaccum_gripper_out_pose.orientation.y , self.vaccum_gripper_out_pose.orientation.z , self.vaccum_gripper_out_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117

    self.vaccum_gripper_to_object_pose.position.x , self.vaccum_gripper_to_object_pose.position.y , self.vaccum_gripper_to_object_pose.position.z = -0.455 , -0.4156 , 0.6

    self.vaccum_gripper_to_object_pose.orientation.x , self.vaccum_gripper_to_object_pose.orientation.y , self.vaccum_gripper_to_object_pose.orientation.z , self.vaccum_gripper_to_object_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117
    
    self.vaccum_gripper_grab_object_pose.position.x , self.vaccum_gripper_grab_object_pose.position.y , self.vaccum_gripper_grab_object_pose.position.z = -0.455 , -0.4156 , 0.35

    self.vaccum_gripper_grab_object_pose.orientation.x , self.vaccum_gripper_grab_object_pose.orientation.y , self.vaccum_gripper_grab_object_pose.orientation.z , self.vaccum_gripper_grab_object_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117


    
    self.ohm_pose.position.x , self.ohm_pose.position.y , self.ohm_pose.position.z = 0.0037 , -0.361 , 0.628

    self.ohm_pose.orientation.x , self.ohm_pose.orientation.y , self.ohm_pose.orientation.z , self.ohm_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117


    self.fingers_gripper_in_pose.position.x , self.fingers_gripper_in_pose.position.y , self.fingers_gripper_in_pose.position.z = -0.1965 , -0.564 , 0.532

    self.fingers_gripper_in_pose.orientation.x , self.fingers_gripper_in_pose.orientation.y , self.fingers_gripper_in_pose.orientation.z , self.fingers_gripper_in_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117

    self.fingers_gripper_prep_pose.position.x , self.fingers_gripper_prep_pose.position.y , self.fingers_gripper_prep_pose.position.z = -0.196 , -0.564 , 0.592

    self.fingers_gripper_prep_pose.orientation.x , self.fingers_gripper_prep_pose.orientation.y , self.fingers_gripper_prep_pose.orientation.z , self.fingers_gripper_prep_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117

    self.fingers_gripper_out_pose.position.x , self.fingers_gripper_out_pose.position.y , self.fingers_gripper_out_pose.position.z = -0.1965 , -0.42 , 0.532

    self.fingers_gripper_out_pose.orientation.x , self.fingers_gripper_out_pose.orientation.y , self.fingers_gripper_out_pose.orientation.z , self.fingers_gripper_out_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117

    self.fingers_gripper_to_object_pose.position.x , self.fingers_gripper_to_object_pose.position.y , self.fingers_gripper_to_object_pose.position.z = -0.415 , -0.239 , 0.6

    self.fingers_gripper_to_object_pose.orientation.x , self.fingers_gripper_to_object_pose.orientation.y , self.fingers_gripper_to_object_pose.orientation.z , self.fingers_gripper_to_object_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117

    self.fingers_gripper_grab_object_pose.position.x , self.fingers_gripper_grab_object_pose.position.y , self.fingers_gripper_grab_object_pose.position.z = -0.415 , -0.239 , 0.408

    self.fingers_gripper_grab_object_pose.orientation.x , self.fingers_gripper_grab_object_pose.orientation.y , self.fingers_gripper_grab_object_pose.orientation.z , self.fingers_gripper_grab_object_pose.orientation.w = -0.708 , -0.705 , -0.0204 , 0.0117


    self.stick_gripper_in_pose.position.x , self.stick_gripper_in_pose.position.y , self.stick_gripper_in_pose.position.z = 0.274 , -0.5586 , 0.5278

    self.stick_gripper_in_pose.orientation.x , self.stick_gripper_in_pose.orientation.y , self.stick_gripper_in_pose.orientation.z , self.stick_gripper_in_pose.orientation.w = -0.708 , -0.705 , -0.02199 , 0.0325

    self.stick_gripper_prep_pose.position.x , self.stick_gripper_prep_pose.position.y , self.stick_gripper_prep_pose.position.z = 0.274 , -0.5585 , 0.5997

    self.stick_gripper_prep_pose.orientation.x , self.stick_gripper_prep_pose.orientation.y , self.stick_gripper_prep_pose.orientation.z , self.stick_gripper_prep_pose.orientation.w = -0.708 , -0.705 , -0.02199 , 0.0325

    self.stick_gripper_out_pose.position.x , self.stick_gripper_out_pose.position.y , self.stick_gripper_out_pose.position.z = 0.274 , -0.42 , 0.5278

    self.stick_gripper_out_pose.orientation.x , self.stick_gripper_out_pose.orientation.y , self.stick_gripper_out_pose.orientation.z , self.stick_gripper_out_pose.orientation.w = -0.708 , -0.705 , -0.02199 , 0.0325

    self.stick_gripper_to_object_pose.position.x , self.stick_gripper_to_object_pose.position.y , self.stick_gripper_to_object_pose.position.z = -0.2391 , -0.341 , 0.6855

    self.stick_gripper_to_object_pose.orientation.x , self.stick_gripper_to_object_pose.orientation.y , self.stick_gripper_to_object_pose.orientation.z , self.stick_gripper_to_object_pose.orientation.w = -0.708 , -0.705 , -0.02199 , 0.0325

    self.stick_gripper_grab_object_pose.position.x , self.stick_gripper_grab_object_pose.position.y , self.stick_gripper_grab_object_pose.position.z =  -0.2391 , -0.341 , 0.5332

    self.stick_gripper_grab_object_pose.orientation.x , self.stick_gripper_grab_object_pose.orientation.y , self.stick_gripper_grab_object_pose.orientation.z , self.stick_gripper_grab_object_pose.orientation.w = -0.708 , -0.705 , -0.02199 , 0.0325


    self.box_pose , self.box_pose_prep = Pose() , Pose()

    self.box_pose.position.x , self.box_pose.position.y , self.box_pose.position.z = 0.263 , 0.4389 , 0.65

    self.box_pose.orientation.x , self.box_pose.orientation.y ,self.box_pose.orientation.z ,self.box_pose.orientation.w = -0.623 , 0.7812 , -0.0178 , 0.00012

    self.box_pose_prep.position.x , self.box_pose_prep.position.y , self.box_pose_prep.position.z = 0.2 , -0.239 , 0.6

    self.box_pose_prep.orientation.x , self.box_pose_prep.orientation.y ,self.box_pose_prep.orientation.z ,self.box_pose_prep.orientation.w = -0.623 , 0.7812 , -0.0178 , 0.00012


    self.box_stick , self.box_stick_prep = Pose() , Pose()

    self.box_stick.position.x , self.box_stick.position.y ,self.box_stick.position.z  = 0.333 , 0.334 , 0.8009

    self.box_stick.orientation.x , self.box_stick.orientation.y ,self.box_stick.orientation.z ,self.box_stick.orientation.w = -0.708 , -0.705 , -0.02199 , 0.0325

    self.box_stick_prep.position.x , self.box_stick_prep.position.y ,self.box_stick_prep.position.z  = 0.333 ,  -0.3348 , 0.7113

    self.box_stick_prep.orientation.x , self.box_stick_prep.orientation.y ,self.box_stick_prep.orientation.z ,self.box_stick_prep.orientation.w = -0.708 , -0.705 , -0.02199 , 0.0325

    self.Gripper_type = 0 # 1 - vaccum , 2 - fingers , 3 -stick
  


  def load_gripper(self,req):

    self.return_gripper()

    self.get_gripper(req)

    return Grippers_switcherResponse('finished loading gripper')


  def return_gripper(self):

    if(self.Gripper_type==1):

      self.return_vaccum_gripper()
    
    elif(self.Gripper_type==2):

      self.return_fingers_gripper()

    elif(self.Gripper_type==3):

      self.return_stick_gripper()


  def get_gripper(self,req):

    if(req.gripper_type==1):

      print('loading vaccum gripper')

      self.load_vaccum_gripper()

      self.Gripper_type = 1

    elif(req.gripper_type==2):

      print('loading fingers gripper')

      self.load_fingers_gripper()

      self.Gripper_type = 2


    elif(req.gripper_type==3):

      print('loading sticker gripper')

      self.load_stick_gripper()

      self.Gripper_type = 3


  
  def move_to_goal_pose(self,goal_pose):

    self.controll.pose_goal = goal_pose

    self.controll.plan = self.controll.plan_cartesian_path(self.controll.pose_goal)

    keyboard.wait('space')

    self.controll.move_group.execute(self.controll.plan, wait=True)

    




  def move_to_home_pose(self):

    self.controll.pose_goal = self.ohm_pose

    self.controll.plan = self.controll.plan_cartesian_path(self.ohm_pose)

    self.controll.show_plan()

    keyboard.wait('space')

    self.controll.move_group.execute(self.controll.plan, wait=True) 

    #return plan, fraction




  def load_stick_gripper(self):

    self.move_to_home_pose()

    ###### go to preparation pose to connect to the gripper ####

    self.move_to_goal_pose(self.stick_gripper_prep_pose)

    ####### go into the gripper stand ####

    self.move_to_goal_pose(self.stick_gripper_in_pose)

    ##### connect to the gripper ######

    keyboard.wait('space')

    self.controll.pub_change_gripper.publish(1)

    ###### get out of the gripper stand ######

    self.move_to_goal_pose(self.stick_gripper_out_pose)

    ### refill glue ###

    self.controll.stick_gripper_refill_glue()


  def return_stick_gripper(self):

    self.move_to_home_pose()

    ###### prepare to bring back the gripper ###

    self.move_to_goal_pose(self.box_stick_prep)

    ###### prepare to bring back the gripper stand ###

    self.move_to_goal_pose(self.stick_gripper_out_pose)

    #### go back to the gripper stand #####

    self.move_to_goal_pose(self.stick_gripper_in_pose)

    #### release the gripper ####

    keyboard.wait('space')

    self.controll.pub_change_gripper.publish(0)

    keyboard.wait('space')

    #### turn off volt ####

    self.controll.pub_24v.publish(1)

    #### get out of the gripper stand ###

    self.move_to_goal_pose(self.stick_gripper_prep_pose)






  def load_fingers_gripper(self):

    self.move_to_home_pose()

    ###### go to preparation pose to connect to the gripper ####

    self.move_to_goal_pose(self.fingers_gripper_prep_pose)

    ####### go into the gripper stand ####

    self.move_to_goal_pose(self.fingers_gripper_in_pose)

    ##### connect to the gripper ######

    keyboard.wait('space')

    self.controll.pub_change_gripper.publish(1)

    keyboard.wait('space')

    ###### get out of the gripper stand ######

    self.move_to_goal_pose(self.fingers_gripper_out_pose)



  def return_fingers_gripper(self):

    self.move_to_home_pose()

    ###### prepare to bring back the gripper ###

    self.move_to_goal_pose(self.fingers_gripper_out_pose)

    #### go back to the gripper stand #####

    self.move_to_goal_pose(self.fingers_gripper_in_pose)

    ### default the gripper ####

    self.controll.fingers_gripper_default()

    #### release the gripper ####

    keyboard.wait('space')

    self.controll.pub_change_gripper.publish(0)

    keyboard.wait('space')

    #### get out of the gripper stand ###

    self.move_to_goal_pose(self.fingers_gripper_prep_pose)







  def load_vaccum_gripper(self):

    self.move_to_home_pose()

    #### connect_to_gripper ###

    self.controll.pub_change_gripper.publish(0)

    ###### go to preparation pose to connect to the gripper ####

    self.move_to_goal_pose(self.vaccum_gripper_prep_pose)

    ####### go into the gripper stand ####

    self.move_to_goal_pose(self.vaccum_gripper_in_pose)


    ##### connect to the gripper ######

    keyboard.wait('space')

    self.controll.pub_change_gripper.publish(1)

    keyboard.wait('space')

    self.controll.pub_24v.publish(0)

    keyboard.wait('space')


    ###### get out of the gripper stand ######

    self.move_to_goal_pose(self.vaccum_gripper_out_pose)


  def return_vaccum_gripper(self):

    self.move_to_home_pose()

    ###### prepare to bring back the gripper stand ###

    self.move_to_goal_pose(self.vaccum_gripper_out_pose)

    #### go back to the gripper stand #####

    self.move_to_goal_pose(self.vaccum_gripper_in_pose)

    #### release the gripper ####

    keyboard.wait('space')

    self.controll.pub_change_gripper.publish(0)

    keyboard.wait('space')

    #### turn off volt ####

    self.controll.pub_24v.publish(1)

    #### get out of the gripper stand ###

    self.move_to_goal_pose(self.vaccum_gripper_prep_pose)






  





def main():
    
    rospy.init_node('gripper_switcher_server', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    controll=ABB_Controll()

    serv = Gripper_switcher(controll)

    print("Ready to load gripper")

    rospy.spin()
    
    
    





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass