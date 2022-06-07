#!/usr/bin/env python
import os

import rospy
import time
from std_msgs.msg import Int16, Int8, Float64, String
from sensor_msgs.msg import JointState
import ros_numpy
import numpy as np
from vaccum_msgs.srv import ControllerCommand, ControllerCommandResponse
import pickle



class restart_node:
    def __init__(self):

        rospy.init_node('controller_node')  # Initial ROS Node
        print "Run ControllerCommand_service"
        self.service = rospy.Service('ControllerCommand_service', ControllerCommand, self.server_callback)
        ######################## Initial ROS Pubs & Subs ####################################################

        rospy.Subscriber("/restart_controller/pid_error", Float64, self.callback_pid_error)
        rospy.Subscriber("/restart_controller/switch_read", Float64, self.callback_switch_read)
        rospy.Subscriber("/restart_controller/encoder_read", Float64, self.callback_encoder_read)
        rospy.Subscriber('/gripper_controller/pressure_value', Int16, self.callback_pressure_value)
        rospy.Subscriber('/joint_states', JointState, self.callback_joint_states)

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
        self.pressure_value = 0
        self.rate = rospy.Rate(10.0)
        self.joint_state = np.zeros((1, 6))
        self.observation = np.zeros((1, 7))
        self.observe_record = False


        time.sleep(1)
        self.main()

    def main(self):
        self.vaccum_off()
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
            self.vaccum_off()
            self.return_box()
            return ControllerCommandResponse("Box has returned", True)

        elif req.command == "drop":
            if self.switch_read is not self.switch_is_pressed:
                self.release_box()
                return ControllerCommandResponse("Box has been released", True)
            else:
                Warning("Switch is still pressed check for problems!!")
                return ControllerCommandResponse("Switch is still pressed check for problems!!", False)

        elif req.command == "vaccum_on":
            self.vaccum_on()
            time.sleep(0.5)
            return ControllerCommandResponse("vaccum on", True)

        elif req.command == "vaccum_off":
            self.vaccum_off()
            time.sleep(0.5)
            return ControllerCommandResponse("vaccum off", True)

        elif req.command == "record_observation_start":
            self.observe_record = True
            return ControllerCommandResponse("Start recording", True)

        elif req.command == "record_observation_stop":

            self.observe_record = False
            self.observation = self.observation[1:, :]
            seconds = time.time()
            local_time = time.ctime(seconds)
            path = "/home/gal/vaccum_ws/src/vaccum_rl/data/"
            str = path + local_time + ".pkl"
            time.sleep(1)

            with open(str, "wb") as f:
                pickle.dump(self.observation, f, protocol=2)
            print("saved pickle files")
            time.sleep(1)
            self.observation = np.zeros((1, 7))
            return ControllerCommandResponse("Done recording", True)

        else:
            return ControllerCommandResponse("Wrong massage", False)


    def callback_switch_read(self, value):
        self.switch_read = value.data

    def callback_pid_error(self, value):
        self.pid_error = value.data

    def callback_joint_states(self, value):
        self.joint_state = np.asarray(value.position)

    def record_observation(self):
        if self.observe_record:
            self.temp = np.append(self.joint_state, self.pressure_value)
            self.observation = np.vstack((self.observation, self.temp))

    def callback_pressure_value(self, value):
        self.pressure_value = value.data
        if self.observe_record:
            self.record_observation()

    def callback_encoder_read(self, value):
        self.encoder_read = value.data

    def vaccum_on(self):
        print "Turn on vaccum"
        self.pub_main_valve.publish(1)
        # self.pub_vaccum_rate.publish(255)
        self.pub_vaccum_rate.publish(180)

    def vaccum_off(self):
        print "Close vaccum"
        self.pub_main_valve.publish(0)

    def release_box(self):

        release_round_number = 2.4 * self.direction
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

        self.move_motor.publish(-2.5 * step)
        print "Box is Home and Ready"
        return





if __name__ == "__main__":
    restart_node()
