#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import tf_conversions
from geometry_msgs.msg import TransformStamped
import numpy as np
import PyKDL
from datetime import datetime
import os

class Calibrator:

    def __init__(self):
        self.transform_camera_2_marker = False
        self.date_time = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        rospy.init_node('calibrator')

        rospy.Subscriber("/aruco_single/transform", TransformStamped, self.aruco_transform)
        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/base_link', '/marker_frame', rospy.Time(0))
                transform_base_2_marker = self.toFrame(trans, rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            if self.transform_camera_2_marker:
                transform_camera_2_base = self.transform_camera_2_marker * transform_base_2_marker.Inverse()
                q = np.array(transform_camera_2_base.M.GetQuaternion())
                p = np.array([transform_camera_2_base.p.x(), transform_camera_2_base.p.y(), transform_camera_2_base.p.z()])
                tmp = np.array([np.hstack((p, q))])
                self.save_to_logfile(tmp)
                print(tmp)
                raw_input("Press Enter to continue...")
                
            rate.sleep()

    def save_to_logfile(self, tf):
        filename = "/home/picko/pickommerce_ws/src/calibration_abb_2_camera/logs/tf_camera_2_base_log_{}.csv".format(self.date_time)
        if os.path.exists(filename):
            append_write = 'a' # append if already exists
        else:
            append_write = 'w' # make a new file if 
        with open(filename, append_write) as logfile:
            np.savetxt(logfile, tf, delimiter=",")

    # Callback of aruco transform topic. Save the aruco marker transform in global variable.
    def aruco_transform(self, data):
        self.transform_camera_2_marker = self.transformStampedtoFrame(data)

    def transformStampedtoFrame(self, msg):
        v = [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z]
        r = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
        return self.toFrame(v, r)

    def toFrame(self, translation, quaternion):
        """
        :return: New :class:`PyKDL.Frame` object
        Convert a translation and quaternion represented as arrays to a :class:`PyKDL.Frame`.
        """
        r = PyKDL.Rotation.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        v = PyKDL.Vector(translation[0], translation[1], translation[2])
        return PyKDL.Frame(r, v)


if __name__ == '__main__':
    calibrator = Calibrator()
    