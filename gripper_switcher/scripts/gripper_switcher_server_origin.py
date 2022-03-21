#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import Int8 , Float32,Int16
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose , Point , PoseStamped
from tf.transformations import quaternion_matrix
import tf
import numpy as np
import time
from pickommerce_msgs.srv import Grippers_switcher,Grippers_switcherResponse
import copy
import rospkg


class ABB_Controll:



  def __init__(self):

    self.robot = moveit_commander.RobotCommander()

    self.scene = moveit_commander.PlanningSceneInterface()

    self.group_name = "manipulator"

    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)



    self.pub_tool_changer = rospy.Publisher('tool_changer',Int8,queue_size=20)

    self.pub_grasp_object = rospy.Publisher('grasp_object',Int16,queue_size=20)

    self.pub_glue_spin = rospy.Publisher('glue_spin',Int16,queue_size=20)

    self.pub_main_valve = rospy.Publisher('main_valve',Int8,queue_size=20)
        
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

  
  def default_mode(self):

    self.pub_grasp_object.publish(162)


  def open_finger_gripper(self):

    self.pub_grasp_object.publish(0)


  def close_finger_gripper(self):

    self.pub_grasp_object.publish(255)



  def vaccum_gripper_grasp(self):

    self.pub_grasp_object.publish(0)

  def vaccum_gripper_release(self):

    self.pub_grasp_object.publish(255)

    self.pub_main_valve.publish(0)

  def glue_gripper_release(self):

    self.pub_grasp_object.publish(255)

  def glue_gripper_grasp(self):

    self.pub_grasp_object.publish(0)

  def glue_gripper_refill_glue(self):

    self.pub_glue_spin.publish(0)

    time.sleep(2.5)

    self.pub_glue_spin.publish(90)




    




class Gripper_switcher:

  def __init__(self,controll):

    self.controll=controll

    self.gripper_len_pub = rospy.Publisher('gripper_len',Float32,queue_size=20)

    self.server = rospy.Service('gripper_switcher', Grippers_switcher, self.load_gripper)

    self.home_pose , self.vaccum_gripper_in_pose , self.vaccum_gripper_prep_pose , self.vaccum_gripper_out_pose , self.vaccum_gripper_to_object_pose , self.vaccum_gripper_grab_object_pose  = Pose() , Pose() , Pose() , Pose()  , Pose() , Pose()

    self.fingers_gripper_in_pose , self.fingers_gripper_prep_pose , self.fingers_gripper_out_pose , self.fingers_gripper_to_object_pose , self.fingers_gripper_grab_object_pose  = Pose() , Pose() , Pose() , Pose(), Pose()

    self.glue_gripper_in_pose , self.glue_gripper_prep_pose , self.glue_gripper_out_pose , self.glue_gripper_to_object_pose , self.glue_gripper_grab_object_pose = Pose() , Pose() , Pose() , Pose(), Pose()

    self.home_pose.position.x , self.home_pose.position.y , self.home_pose.position.z = 0.0174 , -0.359 , 0.697

    self.home_pose.orientation.x , self.home_pose.orientation.y , self.home_pose.orientation.z , self.home_pose.orientation.w = -0.403753228723, 0.914825177059, 0.00154021652319, 0.00870938852969

    self.z_offset , self.y_offset = 0.03 , 0.16


    self.vaccum_gripper_in_pose.position.x , self.vaccum_gripper_in_pose.position.y , self.vaccum_gripper_in_pose.position.z = -0.173114143777, -0.564528217963, 0.524880428068

    self.vaccum_gripper_in_pose.orientation.x , self.vaccum_gripper_in_pose.orientation.y , self.vaccum_gripper_in_pose.orientation.z , self.vaccum_gripper_in_pose.orientation.w =  0.399016714448, -0.916685838385, 0.0108709614368, 0.018829697123


    self.fingers_gripper_in_pose.position.x , self.fingers_gripper_in_pose.position.y , self.fingers_gripper_in_pose.position.z = 0.276366233193, -0.562610306725, 0.528786985421

    self.fingers_gripper_in_pose.orientation.x , self.fingers_gripper_in_pose.orientation.y , self.fingers_gripper_in_pose.orientation.z , self.fingers_gripper_in_pose.orientation.w = -0.403753228723, 0.914825177059, 0.00154021652319, 0.00870938852969



    self.glue_gripper_in_pose.position.x , self.glue_gripper_in_pose.position.y , self.glue_gripper_in_pose.position.z = 0.0289233109078, -0.565701195365, 0.527782888559

    self.glue_gripper_in_pose.orientation.x , self.glue_gripper_in_pose.orientation.y , self.glue_gripper_in_pose.orientation.z , self.glue_gripper_in_pose.orientation.w = -0.337486455283, -0.941081157239, 0.0207132887098, 0.00633306096747


  



    self.vaccum_gripper_prep_pose= copy.deepcopy(self.vaccum_gripper_in_pose)

    self.vaccum_gripper_out_pose = copy.deepcopy(self.vaccum_gripper_in_pose) 

    self.vaccum_gripper_prep_pose.position.z+=self.z_offset

    self.vaccum_gripper_out_pose.position.y+=self.y_offset
   

    self.glue_gripper_prep_pose= copy.deepcopy(self.glue_gripper_in_pose)

    self.glue_gripper_out_pose = copy.deepcopy(self.glue_gripper_in_pose )

    self.glue_gripper_prep_pose.position.z+=self.z_offset

    self.glue_gripper_out_pose.position.y+=self.y_offset


    self.fingers_gripper_prep_pose = copy.deepcopy(self.fingers_gripper_in_pose )

    self.fingers_gripper_out_pose = copy.deepcopy(self.fingers_gripper_in_pose )

    self.fingers_gripper_prep_pose.position.z+=self.z_offset

    self.fingers_gripper_out_pose.position.y+=self.y_offset

    self.Gripper_type = 0 # 1 - vaccum , 3 - fingers , 2 -glue 

    self.add_grippers_to_scene()

    self.add_cage_to_scene()

    self.fingers_gripper_len , self.vaccum_gripper_len , self.glue_gripper_len = 0.17 ,0.34 ,0.28

    self.gripper_len_dict = {3: self.fingers_gripper_len, 2: self.glue_gripper_len, 1: self.vaccum_gripper_len , 0:0}   
  
 


  def load_gripper(self,req):

    if(req.gripper_type!=self.Gripper_type):

      self.return_gripper()

      self.get_gripper(req)

      print('Finished to load the gripper')

      return Grippers_switcherResponse('Finished to load the gripper')
    return Grippers_switcherResponse('Gripper is already loaded')


  def return_gripper(self):

    if(self.Gripper_type==1):

      self.return_vaccum_gripper()
    
    elif(self.Gripper_type==3):

      self.return_fingers_gripper()

    elif(self.Gripper_type==2):

      self.return_glue_gripper()


  def get_gripper(self,req):

    if(req.gripper_type==1):

      print('loading vaccum gripper')

      self.load_vaccum_gripper()

      self.Gripper_type = 1

    elif(req.gripper_type==3):

      print('loading fingers gripper')

      self.load_fingers_gripper()

      self.Gripper_type = 3


    elif(req.gripper_type==2):

      print('loading glueer gripper')

      self.load_glue_gripper()

      self.Gripper_type = 2


  
  def move_to_goal_pose(self,goal_pose):

    self.controll.pose_goal = goal_pose

    self.controll.plan = self.controll.plan_cartesian_path(self.controll.pose_goal)

    raw_input(" enter to continue ")

    self.controll.move_group.execute(self.controll.plan, wait=True)

    




  def move_to_home_pose(self):

    self.controll.pose_goal = self.home_pose

    self.controll.plan = self.controll.plan_cartesian_path(self.home_pose)

    self.controll.show_plan()

    raw_input(" enter to continue ")

    self.controll.move_group.execute(self.controll.plan, wait=True) 

    #return plan, fraction




  def load_glue_gripper(self):

    self.move_to_home_pose()

    ###### go to preparation pose to connect to the gripper ####

    self.move_to_goal_pose(self.glue_gripper_prep_pose)

    self.controll.pub_tool_changer.publish(0)

    self.controll.pub_main_valve.publish(0)

    ####### go into the gripper stand ####

    self.move_to_goal_pose(self.glue_gripper_in_pose)

    ##### connect to the gripper ######

    self.controll.pub_main_valve.publish(0)

    raw_input(" enter to continue ")

    self.controll.pub_tool_changer.publish(1)

    self.attach_glue_gripper_mesh()

    ###### get out of the gripper stand ######

    self.move_to_goal_pose(self.glue_gripper_out_pose)

    ### refill glue ###

    raw_input(" enter to continue ")

    self.controll.glue_gripper_release()
    
    time.sleep(1)

    self.controll.glue_gripper_grasp()


  def return_glue_gripper(self):

    self.move_to_home_pose()

    ###### prepare to bring back the gripper stand ###

    self.move_to_goal_pose(self.glue_gripper_out_pose)

    #### go back to the gripper stand #####

    self.move_to_goal_pose(self.glue_gripper_in_pose)

    #### release the gripper ####

    self.controll.pub_main_valve.publish(0)

    raw_input(" enter to continue ")

    self.controll.pub_tool_changer.publish(0)

    self.detach_glue_gripper_mesh()

    self.controll.default_mode()

    raw_input(" enter to continue ")

    #### turn off volt ####

   

    #### get out of the gripper stand ###

    self.move_to_goal_pose(self.glue_gripper_prep_pose)






  def load_fingers_gripper(self):

    self.move_to_home_pose()

    ###### go to preparation pose to connect to the gripper ####

    self.move_to_goal_pose(self.fingers_gripper_prep_pose)

    self.controll.pub_tool_changer.publish(0)

    ####### go into the gripper stand ####

    self.move_to_goal_pose(self.fingers_gripper_in_pose)

    ##### connect to the gripper ######

    raw_input(" enter to continue ")

    self.controll.pub_tool_changer.publish(1)

    #raw_input(" enter to continue ")

    self.attach_fingers_gripper_mesh()

    ###### get out of the gripper stand ######

    self.controll.pub_main_valve.publish(1)

    self.move_to_goal_pose(self.fingers_gripper_out_pose)



  def return_fingers_gripper(self):

    print('return fingers')

    self.move_to_home_pose()

    ###### prepare to bring back the gripper ###

    self.move_to_goal_pose(self.fingers_gripper_out_pose)

    #### go back to the gripper stand #####

    self.move_to_goal_pose(self.fingers_gripper_in_pose)



    #### release the gripper ####

    raw_input(" enter to continue ")

    self.controll.pub_tool_changer.publish(0)

   

    self.detach_fingers_gripper_mesh()

    self.controll.default_mode()

    raw_input(" enter to continue ")

    #### get out of the gripper stand ###

    self.move_to_goal_pose(self.fingers_gripper_prep_pose)

    # self.pub_main_valve.publish(0)







  def load_vaccum_gripper(self):

    self.move_to_home_pose()

    #### connect_to_gripper ###

  
    self.controll.pub_tool_changer.publish(0)

    ###### go to preparation pose to connect to the gripper ####

    self.move_to_goal_pose(self.vaccum_gripper_prep_pose)

    ####### go into the gripper stand ####

    self.move_to_goal_pose(self.vaccum_gripper_in_pose)




    ##### connect to the gripper ######

    raw_input(" enter to continue ")

 

    self.controll.pub_tool_changer.publish(1)

    self.controll.vaccum_gripper_release()

    self.attach_vaccum_gripper_mesh()

    raw_input(" enter to continue ")

    ###### get out of the gripper stand ######



    self.move_to_goal_pose(self.vaccum_gripper_out_pose)


  def return_vaccum_gripper(self):

    self.move_to_home_pose()

    ###### prepare to bring back the gripper stand ###

    self.move_to_goal_pose(self.vaccum_gripper_out_pose)

    #### go back to the gripper stand #####

    self.move_to_goal_pose(self.vaccum_gripper_in_pose)

    #### release the gripper ####

    raw_input(" enter to continue ")

    self.controll.pub_tool_changer.publish(0)

    self.detach_vaccum_gripper_mesh()

    self.controll.default_mode()

    raw_input(" enter to continue ")

    #### get out of the gripper stand ###

    self.move_to_goal_pose(self.vaccum_gripper_prep_pose)




  def attach_fingers_gripper_mesh(self):

    grasping_group = self.controll.group_name

    #touch_links = self.controll.robot.get_link_names(group=grasping_group)

    touch_links = 'tool0'

    self.controll.scene.attach_mesh('tool0', "fingers_gripper", touch_links=touch_links)

    rospy.sleep(1)

    attached_objects = self.controll.scene.get_attached_objects(['fingers_gripper'])

  
  def detach_fingers_gripper_mesh(self):

    print('detach fingers')

    self.controll.scene.remove_attached_object('tool0', 'fingers_gripper')


  def attach_vaccum_gripper_mesh(self):

    grasping_group = self.controll.group_name

    touch_links = 'tool0'

    self.controll.scene.attach_mesh('tool0', "vaccum_gripper", touch_links=touch_links)

    rospy.sleep(0.5)

    attached_objects = self.controll.scene.get_attached_objects(['vaccum_gripper'])

  
  def detach_vaccum_gripper_mesh(self):

    print('detach vaccum')

    self.controll.scene.remove_attached_object('tool0', 'vaccum_gripper')


  def detach_glue_gripper_mesh(self):

    print('detach glue')

    self.controll.scene.remove_attached_object('tool0', 'glue_gripper')



  def attach_glue_gripper_mesh(self):

    grasping_group = self.controll.group_name

    #touch_links = self.controll.robot.get_link_names(group=grasping_group)

    touch_links = 'tool0'

    self.controll.scene.attach_mesh('tool0', "glue_gripper", touch_links=touch_links)

    rospy.sleep(0.5)

    attached_objects = self.controll.scene.get_attached_objects(['glue_gripper'])


  def add_grippers_to_scene(self):


    fingers_gripper_mesh_pose,vaccum_gripper_mesh_pose,glue_gripper_mesh_pose = PoseStamped(),PoseStamped(),PoseStamped()

    fingers_gripper_mesh_pose.header.frame_id,vaccum_gripper_mesh_pose.header.frame_id,glue_gripper_mesh_pose.header.frame_id = "base_link","base_link","base_link"

    fingers_gripper_mesh_pose.pose.orientation.x,fingers_gripper_mesh_pose.pose.orientation.y,fingers_gripper_mesh_pose.pose.orientation.z,fingers_gripper_mesh_pose.pose.orientation.w =  0.7277256, 0, 0, 0.6858684 

    fingers_gripper_mesh_pose.pose.position.x ,fingers_gripper_mesh_pose.pose.position.y ,fingers_gripper_mesh_pose.pose.position.z   = 0.21,-0.48,0.35

    vaccum_gripper_mesh_pose.pose.orientation.x,vaccum_gripper_mesh_pose.pose.orientation.y,vaccum_gripper_mesh_pose.pose.orientation.z,vaccum_gripper_mesh_pose.pose.orientation.w =  0.7208306, 0, 0, 0.6931112 

    vaccum_gripper_mesh_pose.pose.position.x ,vaccum_gripper_mesh_pose.pose.position.y ,vaccum_gripper_mesh_pose.pose.position.z   = -0.24,-0.44,0.21

    glue_gripper_mesh_pose.pose.orientation.x,glue_gripper_mesh_pose.pose.orientation.y,glue_gripper_mesh_pose.pose.orientation.z,glue_gripper_mesh_pose.pose.orientation.w =  0.5219682, -0.4769834, -0.4766037, 0.522384 

    glue_gripper_mesh_pose.pose.position.x ,glue_gripper_mesh_pose.pose.position.y ,glue_gripper_mesh_pose.pose.position.z   = 0.08,-0.49,0.31

    fingers_name = "fingers_gripper"

    vaccum_name = "vaccum_gripper"

    glue_name = "glue_gripper"

    pkg_path = rospkg.RosPack().get_path('gripper_switcher')

    rospy.sleep(0.5)

    self.controll.scene.add_mesh(fingers_name,fingers_gripper_mesh_pose,'{}/meshes/Finger.stl'.format(pkg_path),(0.001,0.001,0.001))

    self.controll.scene.add_mesh(vaccum_name,vaccum_gripper_mesh_pose,'{}/meshes/Vaccum.stl'.format(pkg_path),(0.001,0.001,0.001))

    self.controll.scene.add_mesh(glue_name,glue_gripper_mesh_pose,'{}/meshes/Glue.stl'.format(pkg_path),(0.001,0.001,0.001))


  def add_cage_to_scene(self):

    cage_pose = PoseStamped()

    cage_pose.header.frame_id = "base_link"

    cage_pose.pose.orientation.x,cage_pose.pose.orientation.y,cage_pose.pose.orientation.z,cage_pose.pose.orientation.w = 0.0005629, -0.707388, -0.706825, 0.0005633 

    cage_pose.pose.position.x ,cage_pose.pose.position.y ,cage_pose.pose.position.z   = 1.02,-1.71,-0.74

    cage_name = "cage"

    pkg_path = rospkg.RosPack().get_path('gripper_switcher')

    rospy.sleep(0.5)

    self.controll.scene.add_mesh(cage_name,cage_pose,'{}/meshes/cage.stl'.format(pkg_path),(0.001,0.001,0.001))




def main():
    
    rospy.init_node('gripper_switcher_server', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    controll=ABB_Controll()

    serv = Gripper_switcher(controll)

    print("Ready to load gripper")

    # Main loop
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

      serv.gripper_len_pub.publish(serv.gripper_len_dict[serv.Gripper_type])
    
      rate.sleep()

 
    
    
    





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass