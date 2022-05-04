#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int16, Int8, Float64
import ros_numpy
import numpy as np
import PyKDL


class restart_node:
    def __init__(self):

        rospy.init_node('home_finger_gripper')  # Initial ROS Node

        ######################## Initial ROS Pubs & Subs ####################################################
        self.move_motor = rospy.Publisher("/restart_controller/motor_move", Float64, queue_size=1)
        rospy.Subscriber("/restart_controller/pid_error", Float64, self.callback_pid_error)
        rospy.Subscriber("/restart_controller/switch_read", Float64, self.callback_switch_read)
        rospy.Subscriber("/restart_controller/encoder_read", Float64, self.callback_encoder_read)

        self.pid_error = 0
        self.encoder_read = 0
        self.switch_read = 1
        self.direction = -1
        self.rate = rospy.Rate(10.0)

        time.sleep(1)

        while not rospy.is_shutdown():  # Run once
            self.release_box()
            # raw_input("enter to return")
            self.return_box()
            exit()

    def callback_switch_read(self, value):
        self.switch_read = value.data

    def callback_pid_error(self, value):
        self.pid_error = value.data

    def callback_encoder_read(self, value):
        self.encoder_read = value.data

    def release_box(self):

        release_round_number = 3 * self.direction
        epsilon = 10

        print "Send motor to update location"

        self.move_motor.publish(360 * release_round_number)

        time.sleep(0.5)

        while abs(self.pid_error) > epsilon:
            print abs(self.pid_error)
            self.rate.sleep()

        print "Box has been released"
        return

    def return_box(self):

        step = -100 * self.direction

        while self.switch_read == 1:
            self.move_motor.publish(step)
            self.rate.sleep()

        self.move_motor.publish(-2 * step)
        print "Box is Home and Ready"
        return





if __name__ == "__main__":
    time.sleep(5)
    restart_node()
