#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import time


class RL_agent:

    def __init__(self):
        rospy.init_node('RL_agent')
        self.rate = rospy.Rate(10.0)
        self.command_msg = String()
        self.RL_abb_command_publisher = rospy.Publisher("/RL_agent/RL_agent_command", String, queue_size=1)
        self.main()

    def episode(self):
        self.command_msg.data = "restart"
        self.RL_abb_command_publisher.publish(self.command_msg)
        time.sleep(10)
        self.command_msg.data = "perform_episode"
        self.RL_abb_command_publisher.publish(self.command_msg)
        time.sleep(10)

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
