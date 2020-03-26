#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class BasicTeleopNode:
    def __init__(self):
        rospy.init_node('basic_teleop')
        rospy.Subscriber('/joy', Joy, self.callback)
        self.pub = rospy.Publisher('/basic_teleop/output', Twist, queue_size=10)

    def callback(self, msg: Joy):
        output = Twist()
        output.linear.x = msg.axes[1]
        output.linear.y = 0
        output.linear.z = msg.axes[2] - msg.axes[5]
        output.angular.x = -msg.axes[3]
        output.angular.y = msg.axes[4]
        output.angular.z = msg.axes[0]
        self.pub.publish(output)


if __name__ == '__main__':
    BasicTeleopNode()
    rospy.spin()
