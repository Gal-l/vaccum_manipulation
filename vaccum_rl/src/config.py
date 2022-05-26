#!/usr/bin/env python3

from geometry_msgs.msg import Pose


class V_Params:
    def __init__(self):
        self.x_start = - -0.55
        self.x_end = -0.4
        self.y_start = 0.8
        self.y_end = 0.4
        self.theta_acceleration = 10

        self.home_pose_joints = \
            [1.5707943439483643, -0.5455922484397888, 0.22974181175231934, 0, 0.3158479928970337,
             0]

        self.in_offset = 0.25
        self.home_pos = Pose()
        self.home_pos.position.x, self.home_pos.position.y, self.home_pos.position.z =\
            0, 0.470967436349 - self.in_offset, 0.847238432744
        self.out_offset = self.home_pos.position.y - 0.316568685636
        self.home_pos.orientation.x, self.home_pos.orientation.y, self.home_pos.orientation.z, \
        self.home_pos.orientation.w = -0.5, 0.5, 0.5, 0.5


        # self.out_offset = self.home_pos.position.y + self.in_offset + self.x_start