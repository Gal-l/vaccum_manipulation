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
from moveit_msgs.msg import RobotState, RobotTrajectory
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import pickle
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import Point, PoseStamped, PointStamped, Pose, PoseArray
from std_msgs.msg import Float32MultiArray
import open3d as o3d
from config import V_Params


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
        self.command = None
        self.v_params = V_Params()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('arm_controller', anonymous=True)

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

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.pose_publisher = rospy.Publisher("/grasp_point", PoseStamped, queue_size=1)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.trajectory_publisher = rospy.Publisher("/trajectory_publisher", PoseArray, queue_size=1)
        rospy.Subscriber('initial_path_pcl', PointCloud2, self.callback)
        rospy.Subscriber('theta_array', Float32MultiArray, self.callback_theta)
        rospy.Subscriber('/RL_agent/RL_agent_command', String, self.callback_command)
        self.pub_tool_changer = rospy.Publisher('tool_changer', Int8, queue_size=20)
        self.pub_main_valve = rospy.Publisher('main_valve', Int8, queue_size=20)
        self.pub_vaccum_rate = rospy.Publisher('grasp_object', Int16, queue_size=20)


        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # print "============ Available Planning Groups:", robot.get_group_names()

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def vaccum_on(self):
        self.pub_main_valve.publish(1)
        self.pub_vaccum_rate.publish(255)

    def vaccum_off(self):
        self.pub_main_valve.publish(0)

    def callback(self, points):
        self.pointcloud2_to_pcd(points)

    def callback_theta(self, theta):
        self.theta = np.asarray(theta.data)

    def callback_command(self, command):
        self.command = command.data
        print "The command is :", command.data

    def pointcloud2_to_pcd(self, pcl):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(
            ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcl))
        self.pcd_array = np.asarray(pcd.points)

    def restart_arm(self):
        print "Restart The Arm Position ..."
        move_group = self.move_group

        # Joint restart
        joint_goal = self.v_params.home_pose_joints
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
        print "Arm Position Has Been Restarted !"

    def go_home(self):
        print "Restart The Arm Position ..."
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose
        # Get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
        print "Arm Position Has Been Restarted !"

    def restart_cartezian(self):

        waypoints = []
        wpose = self.v_params.home_pos
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        self.display_trajectory(plan)

        self.execute_plan(plan)

        print "Arm Position Has Been Restarted !"

    def go_linear(self, x=0, y=0, z=0):

        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        wpose.position.x += x
        wpose.position.y += y
        wpose.position.z += z
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        self.display_trajectory(plan)

        self.execute_plan(plan)

    def show_plan(self):
        # print "Show The trajectory"
        trajectory_2d = np.copy(self.pcd_array)
        theta = np.copy(self.theta)
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.orientation = self.v_params.home_pos.orientation
        wpose_publish = move_group.get_current_pose()
        self.pose_publisher.publish(wpose_publish)
        theta -= 180
        phi = R.from_quat([wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w])
        phi_m = phi.as_dcm()  # as_matrix for older version

        trajectory_rviz = PoseArray()
        trajectory_rviz.header.frame_id = "base_link"
        trajectory_rviz.header.seq = 1
        trajectory_rviz.header.stamp = rospy.Time.now()

        for count, line in enumerate(trajectory_2d):
            wpose.position.x = line[0]
            wpose.position.y = line[1]
            wpose.position.z = line[2]

            r = R.from_euler('y', np.deg2rad(theta[count]-90), degrees=False).as_dcm()
            rviz_matrix = R.from_dcm(np.dot(phi_m, r))
            quat = rviz_matrix.as_quat()
            wpose.orientation.x = quat[0]
            wpose.orientation.y = quat[1]
            wpose.orientation.z = quat[2]
            wpose.orientation.w = quat[3]
            trajectory_rviz.poses.append(copy.deepcopy(wpose))

        self.trajectory_publisher.publish(trajectory_rviz)

    def calculate_trajectory(self, scale=1):

        print "Calculate The trajectory"
        trajectory_2d = np.copy(self.pcd_array)
        theta = np.copy(self.theta)
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose_publish = move_group.get_current_pose()
        # wpose.orientation = self.v_params.home_pos.orientation
        self.pose_publisher.publish(wpose_publish)
        theta -= 180
        phi = R.from_quat([wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w])
        phi_m = phi.as_dcm()  # as_matrix for older version

        for count, line in enumerate(trajectory_2d):
            wpose.position.x = line[0]
            wpose.position.y = line[1]
            wpose.position.z = line[2]

            r = R.from_euler('y', np.deg2rad(theta[count]), degrees=False).as_dcm()
            rotate_matrix = R.from_dcm(np.dot(phi_m, r))
            quat = rotate_matrix.as_quat()
            wpose.orientation.x = quat[0]
            wpose.orientation.y = quat[1]
            wpose.orientation.z = quat[2]
            wpose.orientation.w = quat[3]
            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.1,  # eef_step
            20.0)  # jump_threshold

        #    https://github.com/ros-planning/moveit/issues/24
        for i, trajectory_2d in enumerate(plan.joint_trajectory.points):
            plan.joint_trajectory.points[i].velocities = tuple([0, 0, 0, 0, 0, 0])
            plan.joint_trajectory.points[i].accelerations = tuple([0, 0, 0, 0, 0, 0])
            plan.joint_trajectory.points[i].time_from_start.nsecs = 0
            plan.joint_trajectory.points[i].time_from_start.secs = 0

        new_plan = self.move_group.retime_trajectory(self.robot.get_current_state(),
                                                     plan,
                                                     velocity_scaling_factor=1.0 * scale,
                                                     acceleration_scaling_factor=1.0 * scale,
                                                     algorithm="iterative_spline_parameterization")

        vel_traj = np.array([i.velocities for i in new_plan.joint_trajectory.points])
        return new_plan, fraction

        print "Launch The trajectory"

    def display_trajectory(self, plan):
        print "Display The trajectory"
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):
        print "Execute The trajectory"
        move_group = self.move_group

        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


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


def main(ABB_arm):
    try:

        if ABB_arm.pcd_array is not None and ABB_arm.theta is not None:
            ABB_arm.show_plan()

        if ABB_arm.command == "home":
            ABB_arm.go_home()
            ABB_arm.command = None

        if ABB_arm.command == "restart":
            ABB_arm.restart_arm()
            rospy.sleep(0.5)
            ABB_arm.vaccum_on()
            ABB_arm.go_linear(y=ABB_arm.v_params.in_offset)
            rospy.sleep(0.5)
            ABB_arm.go_linear(y= - ABB_arm.v_params.out_offset)
            ABB_arm.vaccum_off()
            ABB_arm.command = None
            print "Manipulator ready for episode"

        if ABB_arm.command == "read_pos":
            print ABB_arm.move_group.get_current_pose().pose
            ABB_arm.command = None

        if ABB_arm.command == "read_joint":
            print ABB_arm.move_group.get_current_joint_values()
            ABB_arm.command = None

        if ABB_arm.command == "perform_episode":

            if ABB_arm.pcd_array is not None and ABB_arm.theta is not None:
                cartesian_plan, fraction = ABB_arm.calculate_trajectory()

                ABB_arm.display_trajectory(cartesian_plan)

                ABB_arm.execute_plan(cartesian_plan)
            else:
                print "Trajectory is empty"

            ABB_arm.command = None

        # else:
        #     print "Wait for new command..."

        rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':

    ABB_arm = MoveGroupPythonIntefaceTutorial()
    while True:
        main(ABB_arm)
