#!/usr/bin/env python
import numpy as np
from numpy.random import random

import rospy
from kuka_iiwa_14_prismatic_gripper.msg import end_effector

sin = np.sin
cos = np.cos
pi = np.pi

# Msg structure
# Header header

# float64[] position
# float64[] orientation
# float64[] velocity
# float64 time    

def talker():
    pub = rospy.Publisher('ee_data', end_effector, queue_size=100)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        data = end_effector()
        data.position = [1.0, 1.0, 1.0, 0.0, 0.0, 1.306] # angular, linear
        #data.position = trigTrajectory(0.1)
        #data.position = Z_cos(0.5)
        data.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        data.time = rospy.get_time()
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
        
def trigTrajectory(k):
    time = rospy.get_time()
    trajectory = [0.0, 0.0, 0.0, abs(sin(2*pi*k*time)), abs(cos(2*pi*k*time)), 1]
    return trajectory

def Z_cos(k):
    time = rospy.get_time()
    trajectory = [0.0, 0.0, 0.0, 0.5, 0.5, cos(2*pi*k*time)+1 ]
    return trajectory





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass