#!/usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import copy
import time
import warnings
from math import atan2, degrees
from scipy.spatial.transform import Rotation as R
from itertools import combinations
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped, PointStamped
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header


class grasp_points():
    def __init__(self):

        rospy.init_node('grasp_points')
        self.pose_publisher = rospy.Publisher("/grasp_point", PoseStamped, queue_size=1)
        self.point1_publisher = rospy.Publisher("/point1", PointStamped, queue_size=1)
        self.point2_publisher = rospy.Publisher("/point2", PointStamped, queue_size=1)
        self.point3_publisher = rospy.Publisher("/point3", PointStamped, queue_size=1)
        self.pointc_publisher = rospy.Publisher("/pcl_gal", PointCloud2, queue_size=1)


        # rospy.Subscriber('cloud_pcd', PointCloud2, self.callback)
        rospy.Subscriber('/euclidean_segmentation/claster_test', PointCloud2, self.callback)

        self.pc = None
        self.point_cloud = None
        self.need_segmentation = True

        # Main loop
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
            # rospy.spin()

    ############################################################################
    def callback(self, points):
        print("New Pcd has received")
        self.pc = points
        self.pointcloud2_to_pcd()
        if self.need_segmentation:
            self.z_segmatation(self.point_cloud, show=False, update=True)
        pose_msg, point1, point2, point3, gal_cloud = self.find_grasp_points(show=True)
        self.pose_publisher.publish(pose_msg)
        self.point1_publisher.publish(point1)
        self.point2_publisher.publish(point2)
        self.point3_publisher.publish(point3)
        self.pointc_publisher.publish(gal_cloud)


    def pointcloud2_to_pcd(self):
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud.points = o3d.utility.Vector3dVector(
            ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pc))


    @staticmethod
    def AngleBtw2Points(pointA, pointB):
        changeInX = pointB[0] - pointA[0]
        changeInY = pointB[1] - pointA[1]
        return degrees(atan2(changeInY, changeInX))

    @staticmethod
    def draw_registration_result(source, target, transformation):
        """
        This function is drawing for icp get source target and transmission matrix and print them with different
        colors
        :param source: Camera Input
        :param target: Original STL file
        :param transformation: Transformation Matrix
        :return:
        """
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.2, origin=[0, 0, 0])
        o3d.visualization.draw_geometries([source_temp, target_temp, mesh_frame])

    def z_segmatation(self, pcd, bed_level_z=0.29, show=False, update=True):
        """
        make segmentation by z value
        :param pcd: Point cloud
        :param bed_level_z: Float - the value of z level
        :param show: Bol
        :param update: Bol - update the object point cloud
        :return: the object point cloud after segmentation.
        """
        cloud_point_array = np.asarray(pcd.points)  # Transform point cloud to array (x,y,z)
        index = np.where(cloud_point_array[:, 2] > bed_level_z)
        index = index[0]
        filter_plane = pcd.select_by_index(index)
        if show:
            o3d.visualization.draw_geometries([filter_plane])
        if update:
            self.point_cloud = filter_plane
        return filter_plane

    def find_grasp_points(self, gripper_depth=0.04, gripper_max_open=0.08, safe_z_dis=0.02, show=True):

        pcl = self.point_cloud  # load pcl

        center_point, norm, idx = self.get_point_and_norm(pcl=pcl, show=show)  # get the center point, the direction
        # of the normal and the idx of the center point

        pcl_points = np.asarray(pcl.points)  # pcl to points array
        max_Z = np.amax(pcl_points[:, 2], axis=0)  # the max z value
        z_diff = max_Z - pcl_points[idx, 2]  # calculate the diff from the center of mass
        z_diff = gripper_depth - z_diff[0]  # calculate the allows offset regards the restriction of the gripper
        low_boundry = pcl_points[idx, 2] - z_diff  # calculate the low budary of z hight allowed
        effective_pcl = np.asarray(np.where(pcl_points[:, 2] > low_boundry[0]))
        effective_pcl = effective_pcl[0]

        # filter the relevant point cloud
        in_cloud = pcl.select_by_index(effective_pcl)
        in_cloud.paint_uniform_color([0.0, 1.0, 0.0])
        outlier_cloud = pcl.select_by_index(effective_pcl, invert=True)
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(  # create cordinate system in the center point
            size=0.03, origin=center_point)

        # flatten the point cloud
        points_in_cloud = np.asarray(in_cloud.points)
        points_in_cloud[:, 2] = center_point[2]
        points_in_cloud[1, 2] = points_in_cloud[1, 2] + 0.001  # patch since there is no way to flat pcl and using

        # covex hull
        flat_point_cloud = o3d.geometry.PointCloud()
        flat_point_cloud.points = o3d.utility.Vector3dVector(points_in_cloud)
        flat_point_cloud.paint_uniform_color([0.0, 0.0, 1.0])
        in_cloud = pcl.select_by_index(effective_pcl)
        in_cloud.paint_uniform_color([0.0, 1.0, 0.0])

        aabb = flat_point_cloud.get_axis_aligned_bounding_box()
        obb = flat_point_cloud.get_oriented_bounding_box()
        pp = flat_point_cloud.get_axis_aligned_bounding_box()
        hull, _ = flat_point_cloud.compute_convex_hull()
        hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
        hull_ls.paint_uniform_color((1, 0, 0))

        boundry_points_temp = np.asarray(hull_ls.points)
        boundry_points = np.asarray([[0, 0, 0]])
        boundry_points_temp1 = np.asarray([[0, 0, 0]])
        angle = 0
        for count, point in enumerate(boundry_points_temp):  # reorder the points by distance
            if not count:
                boundry_points_temp1 = np.vstack((boundry_points_temp1, point))
                boundry_points_temp = np.delete(boundry_points_temp, 0, 0)
                boundry_points_temp1 = boundry_points_temp1[1:, :]
            else:
                if count>1:
                    vector_1 = boundry_points_temp1[count - 1] - boundry_points_temp
                    vector_2 = boundry_points_temp1[count - 1] - boundry_points_temp1[count - 2]
                    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
                    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
                    dot_product = np.dot(unit_vector_1, unit_vector_2)
                    angle = np.abs(np.rad2deg(np.arccos(dot_product)))
                    bigger_then_ninety = np.where(angle > 90)
                    angle[bigger_then_ninety] = np.abs(angle[bigger_then_ninety] - 180)




                close_to_point = np.linalg.norm(boundry_points_temp - boundry_points_temp1[count - 1],
                                                axis=1)*1000 + angle # Norm to center

                idx = close_to_point.argsort()[:1]
                boundry_points_temp1 = np.vstack((boundry_points_temp1, boundry_points_temp[idx]))
                boundry_points_temp = np.delete(boundry_points_temp, idx, 0)

        grid_split = 1
        for split in range(grid_split):
            if not split:
                boundry_points = self.add_mid_points(boundry_points_temp1)
            else:
                boundry_points = self.add_mid_points(boundry_points)
            print(f' the size is {len(boundry_points)}')
        # boundry_points = boundry_points_temp1
        track_combination = list(combinations(range(len(boundry_points)), 2))
        boundry_points_combination = list(combinations(boundry_points, 2))
        print(
            f'The boundry found by covex hull algorithem contain {len(boundry_points)} points and {len(boundry_points_combination)} lines')
        dis_param = []

        center_dis_list = []
        points_dis_list = []

        for combi in boundry_points_combination:
            center_of_line = (combi[1] + combi[0]) / 2
            center_dis = np.linalg.norm(center_of_line - center_point, axis=0)
            points_dis = np.linalg.norm(combi[1] - center_point, axis=0) + np.linalg.norm(combi[0] - center_point,
                                                                                          axis=0)
            # if np.sign(combi[0][0]) == np.sign(center_point[0]) and np.sign(combi[1][0]) == np.sign(center_point[0]):
            #     center_dis = np.infty

            center_dis_list.append(center_dis)
            points_dis_list.append(points_dis)
            dis_param.append(2*center_dis + points_dis)

        dis_param = np.asarray(dis_param)
        idx = int(dis_param.argsort()[:1])
        temp = np.asarray(track_combination[idx])
        chosen_points = boundry_points[temp]
        grasp_point = (chosen_points[0] + chosen_points[1]) / 2

        if np.linalg.norm(chosen_points[1] - chosen_points[0], axis=0) > gripper_max_open:
            print(f"error value is {np.linalg.norm(chosen_points[1] - chosen_points[0], axis=0) * 1000} [mm]")
        mesh_frame_0 = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.01, origin=chosen_points[0])
        mesh_frame_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.01, origin=chosen_points[1])
        mesh_frame_c = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.01, origin=grasp_point)
        # print(np.linalg.norm(chosen_points[0] - chosen_points[1], axis=0))

        #OBB
        chosen_points[0] = obb.center[0]
        chosen_points[1] = obb.extent[1]




        angle_x = self.AngleBtw2Points(chosen_points[0], chosen_points[1])  # calculate rotation matrix
        print(f' the angke is angle_x')
        # vector_x = chosen_points[1] - chosen_points[0]
        vector_x = pp.min_bound
        normalized_x = vector_x / np.sqrt(np.sum(vector_x ** 2))
        # vector_z = -norm
        vector_z = -np.array([0, 0, 1])
        normalized_z = vector_z / np.sqrt(np.sum(vector_z ** 2))
        vector_y = np.cross(vector_z, vector_x)
        normalized_y = vector_y / np.sqrt(np.sum(vector_y ** 2))

        # build rotation matrix
        rotation_matrix = np.array([[0, 0, 0]])
        rotation_matrix = np.vstack((rotation_matrix, normalized_x))
        rotation_matrix = np.vstack((rotation_matrix, normalized_y))
        rotation_matrix = np.vstack((rotation_matrix, normalized_z))
        rotation_matrix = rotation_matrix[1:]
        rotation_matrix = rotation_matrix.T

        # rotation_matrix = np.vstack((rotation_matrix, np.array([0, 0, 0])))
        # rotation_matrix = np.hstack((rotation_matrix, np.array([[0], [0], [0], [1]])))

        temp_quaternion = R.from_matrix(rotation_matrix)
        quaternion = temp_quaternion.as_quat()
        transformation_vector = np.copy(grasp_point)
        transformation_vector[2] = max_Z + safe_z_dis

        # print(f'the rotation_matrix is \n{rotation_matrix}')
        # print(f'the transformation_vector is \n{transformation_vector}')
        # a = np.recarray((0, ), dtype=[('x', float), ('y', float), ('z', float)])


        return self.convert_to_pose_msg(transformation_vector, quaternion), self.convert_to_point_msg(chosen_points[0]),\
               self.convert_to_point_msg(chosen_points[1]), self.convert_to_point_msg(center_point), self.convert_point_cloud_msg(boundry_points)



        if show:
            o3d.visualization.draw_geometries(
                [in_cloud, outlier_cloud, flat_point_cloud, hull_ls, mesh_frame, mesh_frame_0, mesh_frame_1,
                 mesh_frame_c])
            o3d.visualization.draw_geometries([hull_ls, mesh_frame, mesh_frame_0, mesh_frame_1])

    def add_mid_points(self, boundry_points_temp1):

        boundry_points = np.asarray([[0, 0, 0]])
        for count, point in enumerate(boundry_points_temp1):

            boundry_points = np.vstack((boundry_points, point))
            if count < len(boundry_points_temp1) - 1:
                middle_point = (point + boundry_points_temp1[count + 1]) / 2
                boundry_points = np.vstack((boundry_points, middle_point))

            else:
                # The first and the last
                middle_point = (point + boundry_points_temp1[1]) / 2
                boundry_points = np.vstack((boundry_points, middle_point))

        boundry_points = boundry_points[1:, :]
        return boundry_points

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

    def convert_to_point_msg(self, point):

        point_msg = PointStamped()

        point_msg.header.seq = 1
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "base_link"

        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = point[2]

        return point_msg

    def convert_point_cloud_msg(self, xyz_arr):

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1)]

        header = Header()
        header.frame_id = "base_link"
        header.stamp = rospy.Time.now()
        temp = xyz_arr[:, 2]
        points = np.column_stack((xyz_arr, temp.T))

        # x, y = np.meshgrid(np.linspace(-2, 2, 5), np.linspace(-2, 2, 5))
        # z = 0.5 * np.sin(2 * x - count / 10.0) * np.sin(2 * y)
        # points = np.array([x, y, z, z]).reshape(4, -1).T

        pc2 = point_cloud2.create_cloud(header, fields, points)
        return pc2

    def get_point_and_norm(self, pcl, show=False, normal_treashold=1, distance_threshold=0.08):

        """
        :param pcl: point cloud object of open3d
        :param show: show open3d plots
        :param normal_treashold: not relevant
        :param distance_threshold: not relevant
        :return: the point which most close to the center of mass, the avrage norm and idx of the point in the center
        """

        pcl_points = np.asarray(pcl.points)
        center = self.point_cloud.get_center()  # Find center point
        print(f"The center of the PCL is {center}")

        # Generate new array which calculate the distance from the center
        pcl_close_to_center = np.abs(pcl_points - center)
        pcl_close_to_center = np.linalg.norm(pcl_close_to_center, axis=1)  # Norm to center

        idx_number = 30  # The primary number of points
        avrage_norm = self.Search_avg_norm(pcl_close_to_center, idx_number, normal_treashold=0.1)

        idx_number = 1
        idx = pcl_close_to_center.argsort()[:idx_number]
        center_point = pcl_points[idx]

        return center_point[0], avrage_norm, idx

    def Search_avg_norm(self, pcl_close_to_center, idx_number, normal_treashold=0.5):
        """
        :param pcl_close_to_center: array of the distance of each point from the center
        :param idx_number: how many points look around the center
        :param normal_treashold: irrelevant
        :return: the average norm
        """
        idx = pcl_close_to_center.argsort()[:idx_number]
        pcl_close_to_center = self.point_cloud.select_by_index(idx)

        # print("Recompute the normal of the down-sampled point cloud")
        pcl_close_to_center.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        normals = np.asarray(pcl_close_to_center.normals)
        idx_normals = list(np.where((normals[:, 2] > 0)))
        idx_normals = idx_normals[0]
        normals = normals[idx_normals]
        avrage_norm = np.average(normals, axis=0)
        return avrage_norm


if __name__ == "__main__":
    grasp_points()