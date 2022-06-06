#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8, Int16
import time

from vaccum_msgs.srv import ArmCommand, ControllerCommand


class RL_agent:

    def __init__(self):
        rospy.init_node('RL_agent')
        self.rate = rospy.Rate(10.0)
        self.command_msg = String()

        self.load_services()

        time.sleep(1)
        self.main()

    def load_services(self):
        print("Wait for ArmCommand service...")
        rospy.wait_for_service('ArmCommand_service')
        self.arm_command_srv = rospy.ServiceProxy('ArmCommand_service', ArmCommand)

        print("Wait for ControllerCommand_service...")
        rospy.wait_for_service('ControllerCommand_service')
        self.controller_command_srv = rospy.ServiceProxy('ControllerCommand_service', ControllerCommand)


    def episode(self):

        self.command_msg.data = "return"
        resp = self.controller_command_srv(self.command_msg.data)
        print resp.ans

        if resp.status is False:
            Warning("There is a bag in the restart proccess")
            return

        self.command_msg.data = "restart"
        resp = self.arm_command_srv(self.command_msg.data)
        print resp.ans

        if resp.status is False:
            Warning("There is a bag in the restart proccess")
            return

        self.command_msg.data = "go_forward"
        resp = self.arm_command_srv(self.command_msg.data)
        print resp.ans

        if resp.status is False:
            Warning("There is a bag in the go_forward command")
            return

        self.command_msg.data = "vaccum_on"
        resp = self.controller_command_srv(self.command_msg.data)
        print resp.ans

        if resp.status is False:
            Warning("There is a bag in the vaccum_pump")
            return

        self.command_msg.data = "go_backward"
        resp = self.arm_command_srv(self.command_msg.data)
        print resp.ans

        if resp.status is False:
            Warning("There is a bag in the go_backward command")
            return

        self.command_msg.data = "drop"
        resp = self.controller_command_srv(self.command_msg.data)
        print resp.ans

        if resp.status is False:
            Warning("There is a bag in the drop action")
            return

        self.command_msg.data = "perform_episode"
        resp = self.arm_command_srv(self.command_msg.data)
        print resp.ans

        if resp.status is False:
            Warning("There is a bag in the episode")
            return


        self.command_msg.data = "return"
        resp = self.controller_command_srv(self.command_msg.data)
        print resp.ans

    def main(self):

        input = raw_input("Enter 'new'  to initial simulation else enter: ")
        if input == "new":
            self.command_msg.data = "initial_env"
            resp = self.arm_command_srv(self.command_msg.data)
            print resp

        raw_input("Enter to start starining")

        while True:
            try:
                self.episode()
            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return


if __name__ == '__main__':
    RL_agent = RL_agent()
