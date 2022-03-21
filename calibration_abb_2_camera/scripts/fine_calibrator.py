#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import tf_conversions
from pickommerce_msgs.srv import ClusterList, GraspPoint, PoseGoal
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
import numpy as np
import PyKDL
from datetime import datetime
import ros_numpy
import os

class FineCalibrator:

    def __init__(self):
        rospy.init_node('fine_calibrator')
        # Subscribe to the objects segmentation topic
        print("Wait for get_cluster service...")
        rospy.wait_for_service('get_cluster')
        self.cluster_service = rospy.ServiceProxy('get_cluster', ClusterList)

        print("Wait for center_of_plane_service service...")
        rospy.wait_for_service('center_of_plane_service')
        self.cop_grasp_service = rospy.ServiceProxy('center_of_plane_service', GraspPoint)

        # Subscribe to the trajectory planner service
        print("Wait for trajectory_planner service...")
        rospy.wait_for_service('trajectory_planner')
        self.trajectory_planner = rospy.ServiceProxy('trajectory_planner', PoseGoal)
        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            try:
                (trans,rot) = listener.lookupTransform('/base_link', '/camera_depth_optical_frame', rospy.Time(0))
                print(trans, rot)
                transform_base_2_camera_abb = self.toFrame(trans, rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.tune()
            rate.sleep()
    
    def tune(self):
        print("Send home pose")
        self.publish_pose_with_z_offset(self.get_home_pose(), 0.0)

        # Retrieve single product from the segmentation topic
        print("Retrieve single product from the segmentation topic")
        clusters = self.cluster_service()
        if clusters.data.__len__() == 0:
            print("No clusters found!")
            return

        # Select the highest cluster
        highest_pc = self.get_highest_pc(clusters)

        # Grasp point for vacuum and sticker grippers
        grasp_pose_res = self.cop_grasp_service(highest_pc)
        r_grasp_point = self.convert_pose_msg_to_matrix(grasp_pose_res.grasp_point)
        
        print("Send pose to the arm above the center of plane")
        self.publish_pose_with_z_offset(r_grasp_point, -0.25)

        print("Send pose to the arm close to center of plane")
        self.publish_pose_with_z_offset(r_grasp_point, -0.18)

        print("Send pose to the arm above the center of plane")
        self.publish_pose_with_z_offset(r_grasp_point, -0.25)

    def get_highest_pc(self, cluster_Object_list_msg):
        max_z = 0
        highest_pc = None

        print("Found {} products".format(len(cluster_Object_list_msg.data)))
        for i, pc_msg in enumerate(cluster_Object_list_msg.data):
            pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
            centroid = np.mean(pc, axis=0)

            if i == 0:
                max_z = centroid[2]
                highest_pc = pc_msg
            elif max_z < centroid[2]:
                max_z = centroid[2]
                highest_pc = pc_msg
        return highest_pc
    
    def convert_from_pose_msg(self, pose_msg):
        translation = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]
        rotation = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]
        return translation, rotation

    def convert_pose_msg_to_matrix(self, pose_msg):
        translation, quaternion = self.convert_from_pose_msg(pose_msg)
        r = PyKDL.Rotation.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        v = PyKDL.Vector(translation[0], translation[1], translation[2])
        return PyKDL.Frame(r, v)

    def get_pose_msg_with_z_offset(self, pose, z_offset):
        r_z_offset = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0, 0.0, 0.0, 1.0), PyKDL.Vector(0.0, 0.0, z_offset))
        return self.convert_matrix_to_pose_stamped_msg(pose * r_z_offset)

    def publish_pose_with_z_offset(self, pose, z_offset):
        pose_msg = self.get_pose_msg_with_z_offset(pose, z_offset)
        self.trajectory_planner(pose_msg)
    
    def convert_matrix_to_pose_stamped_msg(self, matrix):
        pose_msg = PoseStamped()
        
        pose_msg.header.seq = 1
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = matrix.p.x()
        pose_msg.pose.position.y = matrix.p.y()
        pose_msg.pose.position.z = matrix.p.z()

        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = matrix.M.GetQuaternion()
        return pose_msg
    
    def get_home_pose(self):
        r = PyKDL.Rotation.Quaternion(-0.403753228723, 0.914825177059, 0.00154021652319, 0.00870938852969)
        v = PyKDL.Vector(0.226, 0.2, 0.673)
        return PyKDL.Frame(r, v)
    
    def toFrame(self, translation, quaternion):
        """
        :return: New :class:`PyKDL.Frame` object
        Convert a translation and quaternion represented as arrays to a :class:`PyKDL.Frame`.
        """
        r = PyKDL.Rotation.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        v = PyKDL.Vector(translation[0], translation[1], translation[2])
        return PyKDL.Frame(r, v)


if __name__ == '__main__':
    fineCalibrator = FineCalibrator()
    