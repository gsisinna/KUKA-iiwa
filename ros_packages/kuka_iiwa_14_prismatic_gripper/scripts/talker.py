#!/usr/bin/env python

import numpy as np
from numpy.random import random
from numpy import sin
from numpy import cos
from numpy import pi

import rospy
from kuka_iiwa_14_prismatic_gripper.msg import end_effector

# Msg structure
# Header header

# float64[] position
# float64[] orientation
# float64[] velocity
# float64 time    

def talker():
    global time
    pub = rospy.Publisher('ee_data', end_effector, queue_size=100)
    rospy.init_node('talker', anonymous=True)
    time = rospy.get_time()
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        data = end_effector()
        #data.position = [0.0, 0.0, 0.0, 0.0, 0.0, 1.305]
        data.position = [-1.0, 0.0, 0.0, 0.0, 0.5, 0.8] # angular, linear
        data.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        data.time = rospy.get_time()
        pub.publish(data)
        rospy.loginfo(data)
        rate.sleep()

def trigTrajectory(k,s):
    time = rospy.get_time()
    trajectory = [0.0, 0.0, 0, s*sin(2*pi*k*time), s*cos(2*pi*k*time), 0.7]
    return trajectory


if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass