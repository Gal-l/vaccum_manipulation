#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8, Int16
import time


class RL_agent:

    def __init__(self):
        rospy.init_node('RL_agent')
        self.rate = rospy.Rate(10.0)
        self.command_msg = String()
        self.RL_abb_command_publisher = rospy.Publisher("/RL_agent/RL_agent_command", String, queue_size=1)
        self.RL_restart_command_publisher = rospy.Publisher("/RL_agent/RL_restart_command", String, queue_size=1)

        self.pub_tool_changer = rospy.Publisher('tool_changer', Int8, queue_size=20)
        self.pub_main_valve = rospy.Publisher('main_valve', Int8, queue_size=20)
        self.pub_vaccum_rate = rospy.Publisher('grasp_object', Int16, queue_size=20)

        time.sleep(1)
        self.main()
    def vaccum_on(self):
        self.pub_main_valve.publish(1)
        self.pub_vaccum_rate.publish(255)

    def vaccum_off(self):
        self.pub_main_valve.publish(0)

    def episode(self):
        self.command_msg.data = "restart"
        print self.command_msg.data
        self.RL_abb_command_publisher.publish(self.command_msg)
        time.sleep(15)
        self.command_msg.data = "home"
        print self.command_msg.data
        self.RL_abb_command_publisher.publish(self.command_msg)
        time.sleep(3)
        self.command_msg.data = "drop"
        self.RL_restart_command_publisher.publish(self.command_msg)
        time.sleep(3)
        self.command_msg.data = "return"
        self.RL_restart_command_publisher.publish(self.command_msg)
        time.sleep(15)


    def main(self):
        while True:
            try:
                self.episode()


            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return


if __name__ == '__main__':
    RL_agent = RL_agent()
