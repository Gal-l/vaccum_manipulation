#!/usr/bin/env python3

from geometry_msgs.msg import Pose


class V_Params:
    def __init__(self):
        self.x_start = -0.55
        self.x_end = -0.4
        self.y_start = 0.8
        self.y_end = 0.4
        self.theta_acceleration = 10

        self.home_pose_joints = \
            [1.570845467156957, -0.28393933560650547, 0.24377226310159317, -0.00033975924362966426,
             0.040054078960497376, 0.00019945545517782964]

        self.home_pos = Pose()
        self.home_pos.position.x, self.home_pos.position.y, self.home_pos.position.z = 0, 0.333016875758, 0.791097679131
        self.home_pos.orientation.x, self.home_pos.orientation.y, self.home_pos.orientation.z, \
        self.home_pos.orientation.w = -0.5, 0.5, 0.5, 0.5

        self.in_offset = 0.25
        self.out_offset = self.home_pos.position.y + self.in_offset + self.x_start