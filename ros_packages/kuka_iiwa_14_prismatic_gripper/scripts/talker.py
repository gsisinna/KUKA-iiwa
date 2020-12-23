#!/usr/bin/env python

import numpy as np
from numpy.random import random
from numpy import sin
from numpy import cos
from numpy import pi

import rospy
from kuka_iiwa_14_prismatic_gripper.msg import end_effector
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from std_msgs.msg import Duration
from std_msgs.msg import Float64

import quaternion

#RBDL model
import rbdl
model = rbdl.loadModel("/home/gabriele/Documents/NRP/GazeboRosPackages/src/kuka_iiwa_14_prismatic_gripper/scripts/iiwa.urdf")

# Msg structure
# Header header

# float64[] position
# float64[] orientation
# float64[] velocity
# float64 time


def talker():
    global time
    pub = rospy.Publisher('ee_data', end_effector, queue_size=100)
    gripper = rospy.Publisher('/iiwa/iiwa_position_controller/command', JointTrajectory, queue_size=100)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        data = end_effector()
        time = rospy.get_time()
        data.time = time
        data.position = [0,0,0,0,0,1.305]
        data.velocity = [0,0,0,0,0,0]

        
        
        rospy.loginfo(data)
        rate.sleep()

def slerp(v0, v1, t_array):
    # >>> slerp([1,0,0,0], [0,0,0,1], np.arange(0, 1, 0.001))
    t_array = np.array(t_array)
    v0 = np.array(v0)
    v1 = np.array(v1)
    
    dot = np.sum(v0 * v1)

    if dot < 0.0:
        v1 = -1 * v1
        dot = -1 * dot
    
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        result = v0[np.newaxis,:] + t_array[:,np.newaxis] * (v1 - v0)[np.newaxis,:]
        return (result.T / np.linalg.norm(result, axis=1)).T
    
    theta_0 = np.arccos(dot)
    sin_theta_0 = sin(theta_0)

    theta = theta_0 * t_array
    sin_theta = sin(theta)
    
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0[:,np.newaxis] * v0[np.newaxis,:]) + (s1[:,np.newaxis] * v1[np.newaxis,:])



if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass


        # if time < 3:
        #     data.position = [0.0, 0.0, 0.0, 0.0, 0.0, 1.305]
        #     pub.publish(data)
        # elif time >= 3 and time <10:
        #     data.position = [-1.0, 0.0, 0.0, 0.0, 0.5, 0.8]
        #     pub.publish(data)
        # elif time >= 10 and time <15:
        #     data.position = [0, 0.0, 0.0, 0.5, 0.5, 0.8]
        #     pub.publish(data)
        # else:
        #     data.position = [0, 0, 0, 0, 0, 1.305]
        #     pub.publish(data)        