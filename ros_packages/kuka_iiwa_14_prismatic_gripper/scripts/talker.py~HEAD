#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from numpy.random import random

def talker():
    pub = rospy.Publisher('ee_pose', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        pose = Float64MultiArray()
        pose.layout.dim.append(MultiArrayDimension(label='pose',size=6,stride=1))
        pose.data = list(random(size=7)-0.5)
        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()
        vel = Float64MultiArray()
        vel.layout.dim.append(MultiArrayDimension(label='vel',size=6,stride=1))
        vel.data = list(random(size=7)-0.5)
        rospy.loginfo(vel)
        pub.publish(vel)
        rate.sleep()
        time = Float64MultiArray()
        time.layout.dim.append(MultiArrayDimension(label='time',size=1,stride=1))
        time.data = list(random(size=1) - 0.5)
        rospy.loginfo(time)
        pub.publish(time)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
