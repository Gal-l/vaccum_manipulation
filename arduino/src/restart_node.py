#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int16, Int8, Float64, String
import ros_numpy
import numpy as np



class restart_node:
    def __init__(self):

        rospy.init_node('home_finger_gripper')  # Initial ROS Node

        ######################## Initial ROS Pubs & Subs ####################################################
        self.move_motor = rospy.Publisher("/restart_controller/motor_move", Float64, queue_size=1)
        rospy.Subscriber("/restart_controller/pid_error", Float64, self.callback_pid_error)
        rospy.Subscriber("/restart_controller/switch_read", Float64, self.callback_switch_read)
        rospy.Subscriber("/restart_controller/encoder_read", Float64, self.callback_encoder_read)
        rospy.Subscriber('/RL_agent/RL_restart_command', String, self.callback_command)

        self.command = None
        self.pid_error = 0
        self.encoder_read = 0
        self.switch_read = 1
        self.direction = -1
        self.rate = rospy.Rate(10.0)

        time.sleep(1)

        while not rospy.is_shutdown():  # Run once
            if self.command == "return":
                self.return_box()
                self.command = None

            elif self.command == "drop":
                self.release_box()
                self.command = None

            time.sleep(1)
            print "The Command is: ", self.command


    def callback_command(self, command):
        self.command = command.data
        print "The command is :", command.data

    def callback_switch_read(self, value):
        self.switch_read = value.data

    def callback_pid_error(self, value):
        self.pid_error = value.data

    def callback_encoder_read(self, value):
        self.encoder_read = value.data

    def release_box(self):

        release_round_number = 1.5 * self.direction
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
    restart_node()
