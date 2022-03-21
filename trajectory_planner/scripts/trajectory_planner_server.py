#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import Float32
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose , Point , PoseStamped
from tf.transformations import quaternion_matrix
import tf
import numpy as np
import copy
from moveit_msgs.msg import PositionIKRequest 
from moveit_msgs.srv import GetPositionIK
from pickommerce_msgs.srv import PoseGoal , PoseGoalResponse

class TrajectoryPlanner:



  def __init__(self):

    self.service = rospy.Service('trajectory_planner', PoseGoal, self.recieve_waypoints)

    self.robot = moveit_commander.RobotCommander()

    self.scene = moveit_commander.PlanningSceneInterface()

    self.group_name = "manipulator"

    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    self.end_effector_pose_publisher = rospy.Publisher('/end_effector_pose',Pose,queue_size=20)

    self.pub_pose_goal = rospy.Publisher('/pose_goal',PoseStamped,queue_size=20)

    self.debug_pose_publisher = rospy.Publisher('/debug_pose_goal',PoseStamped,queue_size=20)

    self.tf_listener = tf.TransformListener()

    #self.pose_goal_sub = rospy.Subscriber('pose_goal', PoseStamped, self.pose_goal_callback)

    self.gripper_len_sub = rospy.Subscriber('gripper_len', Float32, self.gripper_len_callback)

    self.tcp_movement_sub = rospy.Subscriber('tcp_movement', Point, self.tcp_movement_callback)

    self.trans , self.rot = [0,0,0] , [0,0,0,0]

    self.pose_goal , self.end_arm_pose = Pose() , Pose()

    self.gripper_len = 0.0


    


  def gripper_len_callback(self,data):

    self.gripper_len=data.data


  def tcp_movement_callback(self,data):

    self.tcp_movement=[data.x,data.y,data.z,1]

    self.plan_movement_to_gripper(self.tcp_movement)



  def recieve_waypoints(self,req):

    posearray = req.pose_array

    self.send_waypoints(posearray.poses)

    return PoseGoalResponse(' finished planning mission ')



  def send_waypoints(self,waypoints):

    print("Planning cartesian trajectory...")

    self.plan = self.plan_cartesian_path(waypoints)

    self.execute()

    



    
  def convert_to_pose_msg(self, pose):
    pose_msg = PoseStamped()
        
    pose_msg.header.seq = 1
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "base_link"
    pose_msg.pose = pose
  
    return pose_msg
   


  def display_trajectory(self,plan):

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = self.robot.get_current_state()

    display_trajectory.trajectory.append(self.plan)

    self.display_trajectory_publisher.publish(display_trajectory)


  def end_effector_planning(self):

    base_vec=[0,0,1]

    RMatrix = self.quaternion2matrix(self.pose_goal)

    dir_vec = np.dot(RMatrix,base_vec)

    end_eff_pose = copy.deepcopy(self.pose_goal)

    end_eff_pose.position.x -= self.gripper_len*dir_vec[0]

    end_eff_pose.position.y -= self.gripper_len*dir_vec[1]

    end_eff_pose.position.z -= self.gripper_len*dir_vec[2]

    #self.pose_goal = end_eff_pose

    IK_resp = self.check_IK(end_eff_pose)

    if(IK_resp.val == -31):

      print(" Cant Compute IK Or there is A Collision ")

    pose_to_debug = self.convert_to_pose_msg(end_eff_pose)

    self.debug_pose_publisher.publish(pose_to_debug)

    self.move_group.set_pose_target(end_eff_pose)

    self.plan = self.move_group.plan()

    

  

  def plan_cartesian_path(self,waypoints_array):

    waypoints = self.prepare_waypoints_array_for_cartesian_path(waypoints_array)

    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 20) 

    return plan
    


  def prepare_waypoints_array_for_cartesian_path(self,waypoints_array):

    waypoints = []

    base_vec=[0,0,1]

    wpose = self.move_group.get_current_pose().pose

    for waypoint in waypoints_array:

      RMatrix = self.quaternion2matrix(waypoint)

      dir_vec = np.dot(RMatrix,base_vec)

      end_eff_pose = copy.deepcopy(waypoint)

      end_eff_pose.position.x -= self.gripper_len*dir_vec[0]

      end_eff_pose.position.y -= self.gripper_len*dir_vec[1]

      end_eff_pose.position.z -= self.gripper_len*dir_vec[2]

      wpose.position.x += end_eff_pose.position.x - wpose.position.x

      wpose.position.y += end_eff_pose.position.y - wpose.position.y

      wpose.position.z += end_eff_pose.position.z - wpose.position.z

      wpose.orientation = end_eff_pose.orientation

      waypoints.append(copy.deepcopy(wpose))

    return waypoints



  



    
    

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


  def quaternion2matrix(self,p):

    r_matrix = tf.transformations.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    r_matrix = [ele[:-1] for ele in r_matrix]

    return r_matrix

    

  def plan_movement_to_gripper(self,tcp_movement):

    tf_pose = Pose()

    tf_pose.position.x ,tf_pose.position.y ,tf_pose.position.z = self.trans[0] ,self.trans[1] ,self.trans[2]

    tf_pose.orientation.x ,tf_pose.orientation.y ,tf_pose.orientation.z ,tf_pose.orientation.w = self.rot[0] ,self.rot[1] ,self.rot[2] ,self.rot[3]

    tf_matrix_from_gripper_to_base=self.pose_to_matrix(tf_pose)

    point_to_move_in_base_axes=np.dot(tf_matrix_from_gripper_to_base,tcp_movement)

    self.pose_goal.position.x ,self.pose_goal.position.y, self.pose_goal.position.z = point_to_move_in_base_axes[0] ,point_to_move_in_base_axes[1] ,point_to_move_in_base_axes[2]

    self.pose_goal.orientation.x , self.pose_goal.orientation.y , self.pose_goal.orientation.z , self.pose_goal.orientation.w = self.rot[0] ,self.rot[1] ,self.rot[2] ,self.rot[3]

    self.planning()



  def read_end_effector_pose(self):

    base_vec=[0,0,1]

    self.end_arm_pose = self.move_group.get_current_pose().pose

    RMatrix = self.quaternion2matrix(self.end_arm_pose)

    dir_vec = np.dot(RMatrix,base_vec)

    end_eff_pose = copy.deepcopy(self.end_arm_pose)

    end_eff_pose.position.x += self.gripper_len*dir_vec[0]

    end_eff_pose.position.y += self.gripper_len*dir_vec[1]

    end_eff_pose.position.z += self.gripper_len*dir_vec[2]

    #self.end_effector_pose_publisher.publish(end_eff_pose)

    self.end_effector_pose_publisher.publish(self.move_group.get_current_pose().pose)


  def check_IK(self , pose):

    positionikreq = PositionIKRequest()

    positionikreq.group_name=self.group_name

    positionikreq.robot_state = self.robot.get_current_state()

    positionikreq.pose_stamped = self.convert_to_pose_msg(pose)

    positionikreq.avoid_collisions = True

    rospy.wait_for_service('compute_ik')

    try:
        ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)

        resp = ik_service(ik_request = positionikreq)
        
        return resp.error_code

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



  

    





def main():
    
    rospy.init_node('Trajectory_Planner', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    

    control=TrajectoryPlanner()

    rate = rospy.Rate(10) # 10hz
    
    
    
    
    while not rospy.is_shutdown():

      try:

        (control.trans,control.rot) = control.tf_listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))

        control.read_end_effector_pose()
        

      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

      rate.sleep()

      






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass