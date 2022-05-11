#!/usr/bin/env python3

from geometry_msgs.msg import Pose


class V_Params:
    def __init__(self):
        self.x_start = -0.6
        self.x_end = -0.3
        self.y_start = 0.8
        self.y_end = 0.3
        self.in_out = 0.15

        self.home_pose_joints = \
            [1.570845467156957, -0.28393933560650547, 0.24377226310159317, -0.00033975924362966426,
             0.040054078960497376, 0.00019945545517782964]

        self.home_pos = Pose()
        self.home_pos.position.x, self.home_pos.position.y, self.home_pos.position.z = 0, 0.433017658097, 0.791097679131
        self.home_pos.orientation.x, self.home_pos.orientation.y, self.home_pos.orientation.z, \
        self.home_pos.orientation.w = -0.5, 0.5, 0.5, 0.5
