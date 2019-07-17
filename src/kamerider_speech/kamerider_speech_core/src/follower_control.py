#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from std_msgs.msg import String
import sys, select, termios, tty
from turtlebot_msgs.srv import SetFollowState

def speechCallback(msg):
    string = msg.data
    if "stop" in string:
        rospy.loginfo("Detect stop signal, Stop following")
        set_state = rospy.ServiceProxy("/turtlebot_follower/change_state", SetFollowState)
        msg = False
        response = set_state(msg)
        os.system("rosnode kill /follower_control")
    else:
        symbols = ["!", "?", ".", ",", ";", ":"]
        output = []
        for part in string.lstrip().split(","):
            for word in part.split():
                for symbol in symbols:
                    if symbol in word:
                        word = word[:-1] #去除了可能出现的句子中间的标点符号
                        output.append(word)
        output = [item.lower() for item in output]
        print (output)
        if "stop" in output:
            rospy.loginfo("Detect stop signal, Stop following")
            set_state = rospy.ServiceProxy("/turtlebot_follower/change_state", SetFollowState)
            msg = False
            response = set_state(msg)
            os.system("rosnode kill /follower_control")


def main():
    rospy.init_node("follower_control")
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service("/turtlebot_follower/change_state")
    rospy.Subscriber("/baidu_to_control", String, speechCallback)
    rospy.spin()


if __name__ == '__main__':
    main()
