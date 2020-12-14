#!/usr/bin/env python
import numpy as np
from numpy.random import random

import rospy
from kuka_iiwa_14_prismatic_gripper.msg import end_effector

# Msg structure
# Header header

# float64[] position
# float64[] orientation
# float64[] velocity
# float64 time

def talker():
    pub = rospy.Publisher('ee_data', end_effector, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 100hz
    while not rospy.is_shutdown():
        data = end_effector()
        data.position = [0.0, 0.0, 0.0, 0.0, 0.0, 1.306] # angular, linear
        #data.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # angular, linear
        #data.position = [0.0, 0.0, 0.0, 0.0, 0.0, 1.306] # angular, linear
        data.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        data.time = rospy.get_time() ## Sistemare tempo
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass