#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pickle
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Header
from config import V_Params


class RL_agent:

    def __init__(self):
        rospy.init_node('RL_agent')
