#!/usr/bin/env python
import numpy as np
import rospy
from kuka_iiwa_14_prismatic_gripper.msg import end_effector
from numpy.random import random

# Msg structure
# Header header

# float64[] positions
# float64[] velocities

# float64 time

def talker():
    pub = rospy.Publisher('ee_data', end_effector, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        data = end_effector()
        data.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        data.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        data.time = rospy.get_rostime()
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
