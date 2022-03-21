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
        
    self.tf_listener = tf.TransformListener()

    self.pose_goal_sub = rospy.Subscriber('pose_goal', Pose, self.pose_goal_callback)

    self.tcp_movement_sub = rospy.Subscriber('tcp_movement', Point, self.tcp_movement_callback)

    self.trans , self.rot = [0,0,0] , [0,0,0,0]

    self.pose_goal=Pose()

    


    





  def tcp_movement_callback(self,data):

    self.tcp_movement=[data.x,data.y,data.z,1]

    self.plan_movement_to_gripper(self.tcp_movement)


  def pose_goal_callback(self,data):

    self.pose_goal=data

    #self.planning()

    #self.plan = self.plan_cartesian_path(self.pose_goal)


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



    




class ABB_presentation:

  def __init__(self,controll):

    self.controll=controll

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




  

  def stick_gripper(self):

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

    ### move to the bananna position ###

    self.move_to_goal_pose(self.stick_gripper_to_object_pose)

    ### go down to grab the bananna ####

    self.move_to_goal_pose(self.stick_gripper_grab_object_pose)

    ### close the gripper ###

    keyboard.wait('space')

    self.controll.stick_gripper_close()

    keyboard.wait('space')

    #### go up with the bananna ####

    self.move_to_goal_pose(self.stick_gripper_to_object_pose)

    ### go to prep box position ####

    self.move_to_goal_pose(self.box_stick_prep)

    #### go to the box position ####

    self.move_to_goal_pose(self.box_stick)

    ##### release bananna ######

    keyboard.wait('space')

    self.controll.stick_gripper_open()

    keyboard.wait('space')

    ###### prepare to bring back the gripper ###

    self.move_to_goal_pose(self.box_stick_prep)

    ###### prepare to bring back the gripper stand ###

    self.move_to_goal_pose(self.stick_gripper_out_pose)

    #### go back to the gripper stand #####

    self.move_to_goal_pose(self.stick_gripper_in_pose)

    ### default the gripper ####

    #self.controll.stick_gripper_default()

    #### release the gripper ####

    keyboard.wait('space')

    self.controll.pub_change_gripper.publish(0)

    keyboard.wait('space')

    #### turn off volt ####

    self.controll.pub_24v.publish(1)

    #### get out of the gripper stand ###

    self.move_to_goal_pose(self.stick_gripper_prep_pose)




  def vaccum_gripper(self):

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

    ### move to the bowl position ###

    self.move_to_goal_pose(self.vaccum_gripper_to_object_pose)

    ### go down to grab the bowl ####

    self.move_to_goal_pose(self.vaccum_gripper_grab_object_pose)

    ### close the gripper ###

    keyboard.wait('space')

    self.controll.vaccum_gripper_close()

    keyboard.wait('space')

    #### go up with the bowl ####

    self.move_to_goal_pose(self.vaccum_gripper_to_object_pose)

    ### go to prep box position ####

    self.move_to_goal_pose(self.box_pose_prep)

    #### go to the box position ####

    self.move_to_goal_pose(self.box_pose)

    ##### release bowl ######

    keyboard.wait('space')

    self.controll.vaccum_gripper_open()

    keyboard.wait('space')

    ###### prepare to bring back the gripper ###

    self.move_to_goal_pose(self.box_pose_prep)

    ###### prepare to bring back the gripper stand ###

    self.move_to_goal_pose(self.vaccum_gripper_out_pose)

    #### go back to the gripper stand #####

    self.move_to_goal_pose(self.vaccum_gripper_in_pose)

    ### default the gripper ####

    #self.controll.vaccum_gripper_default()

    #### release the gripper ####

    keyboard.wait('space')

    self.controll.pub_change_gripper.publish(0)

    keyboard.wait('space')

    #### turn off volt ####

    self.controll.pub_24v.publish(1)

    #### get out of the gripper stand ###

    self.move_to_goal_pose(self.vaccum_gripper_prep_pose)




  def fingers_gripper(self):

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

    ### move to the bottle position ###

    self.move_to_goal_pose(self.fingers_gripper_to_object_pose)

    #### open the gripper #####

    keyboard.wait('space')

    self.controll.open_finger_gripper()

    keyboard.wait('space')

    ### go down to grab the bottle ####

    self.move_to_goal_pose(self.fingers_gripper_grab_object_pose)

    ### close the gripper ###

    keyboard.wait('space')

    self.controll.close_finger_gripper()

    keyboard.wait('space')

    #### go up with the bottle ####

    self.move_to_goal_pose(self.fingers_gripper_to_object_pose)

    ### got to prep box position ####

    self.move_to_goal_pose(self.box_pose_prep)

    #### go to the box position ####

    self.move_to_goal_pose(self.box_pose)

    ##### release bottle ######

    keyboard.wait('space')

    self.controll.open_finger_gripper()

    keyboard.wait('space')

    ###### prepare to bring back the gripper ###

    self.move_to_goal_pose(self.box_pose_prep)
    
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




    



  def presentation_script(self):

    self.controll.stick_gripper_default()

    self.controll.pub_change_gripper.publish(0)

    self.move_to_home_pose()

    self.controll.fingers_gripper_default()

    keyboard.wait('space')

    self.fingers_gripper()

    keyboard.wait('space')

    self.move_to_home_pose()

    keyboard.wait('space')

    self.vaccum_gripper()

    keyboard.wait('space')

    self.move_to_home_pose()

    keyboard.wait('space')

    self.stick_gripper()

    keyboard.wait('space')

    self.move_to_home_pose()





  def vaccum_home_ready(self):

    #self.controll.pub_change_gripper.publish(0)

    self.move_to_home_pose()

    keyboard.wait('space')

    self.load_vaccum_gripper()

    keyboard.wait('space')

    self.move_to_home_pose()



    
















def main():
    
    rospy.init_node('ABB_controll', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    

    controll=ABB_Controll()

    pres = ABB_presentation(controll)

    rate = rospy.Rate(10) # 10hz
    

    
    
    while not rospy.is_shutdown():

    

      #(control.trans,control.rot) = control.tf_listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))

      if((keyboard.is_pressed('ctrl') and keyboard.is_pressed('a'))):

        #print(" trans  ",control.trans,"    rot   ",control.rot) 

        print(controll.move_group.get_current_pose().pose)

      if((keyboard.is_pressed('ctrl') and keyboard.is_pressed('l'))):

        controll.plan = controll.plan_cartesian_path(controll.pose_goal)

        controll.show_plan()

      if((keyboard.is_pressed('ctrl') and keyboard.is_pressed('e'))):

        #controll.execute()

        controll.move_group.execute(controll.plan, wait=True)

      if((keyboard.is_pressed('ctrl') and keyboard.is_pressed('d'))):

        pres.presentation_script()
        #pres.vaccum_home_ready()

      rate.sleep()






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass