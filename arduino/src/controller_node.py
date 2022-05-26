#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int16, Int8, Float64, String
import ros_numpy
import numpy as np
from vaccum_msgs.srv import ControllerCommand, ControllerCommandResponse



class restart_node:
    def __init__(self):

        rospy.init_node('controller_node')  # Initial ROS Node
        print "Run ControllerCommand_service"
        self.service = rospy.Service('ControllerCommand_service', ControllerCommand, self.server_callback)
        ######################## Initial ROS Pubs & Subs ####################################################

        rospy.Subscriber("/restart_controller/pid_error", Float64, self.callback_pid_error)
        rospy.Subscriber("/restart_controller/switch_read", Float64, self.callback_switch_read)
        rospy.Subscriber("/restart_controller/encoder_read", Float64, self.callback_encoder_read)
        rospy.Subscriber('/RL_agent/RL_restart_command', String, self.callback_command)

        self.move_motor = rospy.Publisher("/restart_controller/motor_move", Float64, queue_size=1)
        self.pub_tool_changer = rospy.Publisher('tool_changer', Int8, queue_size=20)
        self.pub_main_valve = rospy.Publisher('main_valve', Int8, queue_size=20)
        self.pub_vaccum_rate = rospy.Publisher('grasp_object', Int16, queue_size=20)

        self.command = None
        self.pid_error = 0
        self.switch_is_pressed = 0
        self.encoder_read = 0
        self.switch_read = 1
        self.direction = -1
        self.rate = rospy.Rate(10.0)

        time.sleep(1)
        self.main()

    def main(self):
        while (True):
            try:
                rospy.sleep(0.001)
            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return

    def server_callback(self, req):
        print (req)

        if req.command == "return":
            self.return_box()
            return ControllerCommandResponse("Box has returned")

        elif req.command == "drop":
            if self.switch_read is not self.switch_is_pressed:
                self.release_box()
                return ControllerCommandResponse("Box has been released")
            else:
                Warning("Switch is still pressed check for problems!!")
                return ControllerCommandResponse("Switch is still pressed check for problems!!")

        else:
            return ControllerCommandResponse("Wrong massage")


    def callback_switch_read(self, value):
        self.switch_read = value.data

    def callback_pid_error(self, value):
        self.pid_error = value.data

    def callback_encoder_read(self, value):
        self.encoder_read = value.data

    def vaccum_on(self):
        self.pub_main_valve.publish(1)
        self.pub_vaccum_rate.publish(255)

    def vaccum_off(self):
        self.pub_main_valve.publish(0)

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

        while not self.switch_read == self.switch_is_pressed:
            self.move_motor.publish(step)
            self.rate.sleep()

        self.move_motor.publish(-2 * step)
        print "Box is Home and Ready"
        return





if __name__ == "__main__":
    restart_node()
