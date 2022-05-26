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
from vaccum_msgs.srv import ArmCommand, ArmCommandResponse


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
        print "Run ArmCommand_service"
        self.service = rospy.Service('ArmCommand_service', ArmCommand, self.server_callback)
        rospy.init_node('arm_controller', anonymous=True)

        self.trajectory_publisher = rospy.Publisher("/trajectory_publisher", PoseArray, queue_size=1)
        rospy.Subscriber('initial_path_pcl', PointCloud2, self.callback)
        rospy.Subscriber('theta_array', Float32MultiArray, self.callback_theta)
        rospy.Subscriber('/RL_agent/RL_agent_command', String, self.callback_command)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        self.pose_publisher = rospy.Publisher("/grasp_point", PoseStamped, queue_size=1)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.main()

    def main(self):
        while (True):
            try:
                if self.pcd_array is not None and self.theta is not None:
                    self.show_plan()
                rospy.sleep(0.5)
            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return

    def server_callback(self, req):
        print (req)

        if req.command == "home":
            self.go_home()
            return ArmCommandResponse("Your Robot is Home and Ready")

        if req.command == "restart":
            self.restart_arm()
            rospy.sleep(0.5)
            # self.vaccum_on()
            print "Go forward"
            self.go_linear(y=self.v_params.in_offset)
            rospy.sleep(0.5)
            print "Go backword"
            self.go_linear(y=-self.v_params.out_offset)
            rospy.sleep(0.5)
            # self.vaccum_off()
            print "Manipulator ready for episode"
            return ArmCommandResponse("Manipulator ready for episode")

        if req.command == "read_pos":
            print self.move_group.get_current_pose().pose

        if req.command == "initial_env":
            print self.initial_env()
            return ArmCommandResponse("Environment and robot are ready to go")


        if req.command == "read_joint":
            print self.move_group.get_current_joint_values()

        if req.command == "perform_episode":

            if self.pcd_array is not None and self.theta is not None:
                cartesian_plan, fraction = self.calculate_trajectory()

                self.display_trajectory(cartesian_plan)

                self.execute_plan(cartesian_plan)
                return ArmCommandResponse("Episode has been peformed")
            else:
                print "Trajectory is empty"
                return ArmCommandResponse("Trajectory is empty")


        return ArmCommandResponse("Wrong Command")

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

        joint_goal = move_group.get_current_joint_values()
        joint_goal = [0, 0, 0, 0, 0, 0]
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
        print "Arm Position Has Been Restarted !"

    def initial_env(self):
        self.restart_arm()
        self.scene.remove_attached_object('tool0', 'vaccum_gripper')

        vaccum_gripper_mesh_pose = PoseStamped()
        vaccum_gripper_mesh_pose.header.frame_id = "base_link"

        vaccum_gripper_mesh_pose.pose.orientation.x, vaccum_gripper_mesh_pose.pose.orientation.y, vaccum_gripper_mesh_pose.pose.orientation.z, \
        vaccum_gripper_mesh_pose.pose.orientation.w = -0.7864119, 0.0045584, -0.6176752, 0.0035803  # 0.5535217, -0.5586397, 0.438775, 0.4347551

        vaccum_gripper_mesh_pose.pose.position.x, vaccum_gripper_mesh_pose.pose.position.y, \
        vaccum_gripper_mesh_pose.pose.position.z = -0.10, 0.64, 0.82  # 0.83 + 0.021, 0.10, 0.76

        vaccum_name = "vaccum_gripper"
        pkg_path = rospkg.RosPack().get_path('vaccum_rl')
        rospy.sleep(0.2)

        self.scene.add_mesh(vaccum_name, vaccum_gripper_mesh_pose, '{}/meshes/vaccum_box.stl'.format(pkg_path),
                            (0.001, 0.001, 0.001))

        rospy.sleep(0.5)
        touch_links = 'tool0'
        self.scene.attach_mesh('tool0', "vaccum_gripper", touch_links=touch_links)
        rospy.sleep(0.2)
        # TODO: add plane to sence
        # self.scene.add_plane(['xy plane', ])

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

            r = R.from_euler('y', np.deg2rad(theta[count] - 90), degrees=False).as_dcm()
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
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):
        print "Execute The trajectory"
        move_group = self.move_group
        move_group.execute(plan, wait=True)

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


if __name__ == '__main__':
    ABB_arm = MoveGroupPythonIntefaceTutorial()
