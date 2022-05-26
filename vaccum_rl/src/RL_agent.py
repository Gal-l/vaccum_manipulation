#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8, Int16
import time

from vaccum_msgs.srv import ArmCommand


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

    def episode(self):

        self.command_msg.data = "restart"
        print self.command_msg.data
        resp = self.arm_command_srv(self.command_msg.data)
        print resp

        self.command_msg.data = "perform_episode"
        print self.command_msg.data
        resp = self.arm_command_srv(self.command_msg.data)
        print resp

        self.command_msg.data = "drop"
        resp = self.arm_command_srv(self.command_msg.data)
        print resp

        self.command_msg.data = "return"
        resp = self.arm_command_srv(self.command_msg.data)
        print resp

    def main(self):

        raw_input("Enter to initial simulation")
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
