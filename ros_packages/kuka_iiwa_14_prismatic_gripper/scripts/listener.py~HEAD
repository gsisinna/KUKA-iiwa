#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from numpy.random import random

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('ee_pose', Float64MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
