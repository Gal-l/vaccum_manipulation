#!/usr/bin/env python

from __future__ import print_function
from pickommerce_msgs.srv import Grippers_switcher , PoseGoal
import sys
import rospy
from geometry_msgs.msg import Pose , Point , PoseStamped
from std_msgs.msg import Int8 , Float32,Int16
import time

class ABB_demo:

    def __init__(self):

        rospy.init_node('Demo1', anonymous=True)

        self.pub_pose_goal= rospy.Publisher('pose_goal',PoseStamped,queue_size=20)

        self.pub_tool_changer = rospy.Publisher('tool_changer',Int8,queue_size=20)

        self.pub_grasp_object = rospy.Publisher('grasp_object',Int16,queue_size=20)

        self.pub_glue_spin = rospy.Publisher('glue_spin',Int16,queue_size=20)

        self.pub_main_valve = rospy.Publisher('main_valve',Int8,queue_size=20)

        self.pub_pid_mode = rospy.Publisher('pid_finger_mode',Int8,queue_size=20)

        self.pub_pid_target = rospy.Publisher('fingers_gripper_target',Int16,queue_size=20)

        self.pub_fingers_force = rospy.Publisher('fingers_force',Int8,queue_size=20)

        self.Box_position , self.Box_orientation = [0.4 ,0.3 , 0.35 ] , [-0.983 , 0.176 ,  0.00227,  0.0277]

        self.presentation_position , self.presentation_orientation = [ 0.551 , -0.222 , 0.371  ] , [-0.944 , 0.328 ,   -0.0242, 0.0185]

        #self.presentation_position_glue , self.presentation_orientation_glue = [ 0.31029352816 , 0.297452218692 ,  0.798889582555 - 0.34  ] , [-0.80351348438 , -0.594690959132 ,   0.0252424733974, 0.00845937839763]

        self.presentation_position_glue , self.presentation_orientation_glue = [ 0.397984620307 , -0.235031940587 ,  0.687163318405 - 0.28   ] , [-0.918352015036 , 0.394751995005 ,   -0.0163985241172, 0.0230548763965]

        self.presentation_position_fingers , self.presentation_orientation_fingers =[ 0.397984620307 , -0.235031940587 ,  0.687163318405 - 0.28   ] , [-0.918352015036 , 0.394751995005 ,   -0.0163985241172, 0.0230548763965]




    





    def turn_on_pid(self):

        self.pub_pid_mode.publish(1)


    def turn_off_pid(self):

        self.pub_pid_mode.publish(0)


    def default_mode(self):

        self.pub_grasp_object.publish(162)


    def open_finger_gripper(self):

        self.pub_pid_mode.publish(0)

        self.pub_main_valve.publish(1)

        self.pub_grasp_object.publish(0)


    def close_finger_gripper(self):

        self.pub_pid_mode.publish(0)

        self.pub_main_valve.publish(1)

        self.pub_grasp_object.publish(255)



    def vaccum_gripper_grasp(self):

        self.pub_grasp_object.publish(255)

        self.pub_main_valve.publish(1)

    def vaccum_gripper_release(self):

        #self.pub_grasp_object.publish(255)

        self.pub_main_valve.publish(0)

    def glue_gripper_release(self):

        self.pub_main_valve.publish(1)

        for i in range(200-162):

            self.pub_grasp_object.publish(162 + i)

            time.sleep(0.1)

        self.pub_main_valve.publish(0)

    def glue_gripper_grasp(self):

        self.pub_main_valve.publish(1)

        self.pub_grasp_object.publish(0)

    def glue_gripper_refill_glue(self):

        self.pub_main_valve.publish(0)

        self.pub_glue_spin.publish(0)

        time.sleep(1.5)

        self.pub_glue_spin.publish(90)

    
    def convert_to_pose_msg(self, translation, rotation):
        pose_msg = PoseStamped()
        
        pose_msg.header.seq = 1
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]

        pose_msg.pose.orientation.x = rotation[0]
        pose_msg.pose.orientation.y = rotation[1]
        pose_msg.pose.orientation.z = rotation[2]
        pose_msg.pose.orientation.w = rotation[3]
        return pose_msg

 



    def load_gripper(self,g):
        rospy.wait_for_service('gripper_switcher')
        try:
            gripper_service = rospy.ServiceProxy('gripper_switcher', Grippers_switcher)
            resp = gripper_service(g)
            print(resp.ans)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def SendPoseGoal(self,pose_stamp):
   
        rospy.wait_for_service('trajectory_planner')
       
        try:
        
            ABB_ros_service = rospy.ServiceProxy('trajectory_planner', PoseGoal)
            resp = ABB_ros_service(pose_stamp)
            print(resp.ans)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def move_to_pose(self,position,orientation):

        pose_stamp = self.convert_to_pose_msg(position,orientation)

        self.SendPoseGoal(pose_stamp)



    def mission(self):

        # 1 - vaccum , 3 - fingers , 2 -glue


        # load vaccum gripper

        self.load_gripper(1)

        # move to presentation Position

        raw_input(" move to presentation Position chooce action in ABB_ros terminal " )

        self.move_to_pose(self.presentation_position, self.presentation_orientation)

        # vaccum gripper grasp

        raw_input(" vaccum gripper grasp ")

        self.vaccum_gripper_grasp()

        # vaccum gripper release

        raw_input(" vaccum gripper release ")

        self.vaccum_gripper_release()

        # move to box position

        raw_input(" move to Box Position chooce action in ABB_ros terminal ")

        self.move_to_pose(self.Box_position, self.Box_orientation)

        # vaccum gripper grasp 2 times

        for i in range(2):

            raw_input(" vaccum gripper grasp ")

            self.vaccum_gripper_grasp()

            # vaccum gripper release

            raw_input(" vaccum gripper release ")

            self.vaccum_gripper_release()

        # move to presentation Position

        raw_input(" move to presentation Position chooce action in ABB_ros terminal " )

        self.move_to_pose(self.presentation_position, self.presentation_orientation)







        # load glue gripper

        self.load_gripper(2)

        # go to presentation pose

        raw_input(" move to presentation Position chooce action in ABB_ros terminal ")

        self.move_to_pose(self.presentation_position_glue, self.presentation_orientation_glue)

        # refill glue

        raw_input(" refill glue ")

        self.glue_gripper_refill_glue()

        # move to box position

        raw_input(" move to Box Position chooce action in ABB_ros terminal ")

        self.move_to_pose(self.Box_position, self.Box_orientation)

        # grasp object with glue gripper

        raw_input(" grasp object ")

        self.glue_gripper_grasp()

        # release object with glue gripper

        raw_input(" release object ")

        self.glue_gripper_release()

        # refill glue

        raw_input(" refill glue ")

        self.glue_gripper_refill_glue()

        # grasp object with glue gripper

        raw_input(" grasp object ")

        self.glue_gripper_grasp()

        # release object with glue gripper

        raw_input(" release object ")

        self.glue_gripper_release()

        # go to presentation pose

        raw_input(" move to presentation Position chooce action in ABB_ros terminal ")

        self.move_to_pose(self.presentation_position_glue, self.presentation_orientation_glue)








        # load fingers gripper

        self.load_gripper(3)

        # go to presentation pose

        raw_input(" move to presentation Position chooce action in ABB_ros terminal ")

        self.move_to_pose(self.presentation_position_fingers, self.presentation_orientation_fingers)

        #move to box position 

        raw_input(" move to box position chooce action in ABB_ros terminal ")

        self.move_to_pose(self.Box_position, self.Box_orientation)

        # send to force 



        raw_input(" open_fingers ")

        self.open_finger_gripper()

        raw_input(" send  force ")

        self.close_finger_gripper()

        raw_input(" open_fingers ")

        self.open_finger_gripper()

        raw_input(" send  force ")

        self.close_finger_gripper()

        raw_input(" open_fingers ")

        # self.open_finger_gripper()


        for i in range(20):

            self.pub_grasp_object.publish(255-i*10)

            time.sleep(0.3)

        raw_input(" close the fingers ")

        self.close_finger_gripper()


        # go to presentation pose

        raw_input(" move to presentation Position chooce action in ABB_ros terminal ")

        self.move_to_pose(self.presentation_position_fingers, self.presentation_orientation_fingers)



        self.load_gripper(1)










if __name__ == "__main__":

    Demo = ABB_demo()
    
    Demo.mission()