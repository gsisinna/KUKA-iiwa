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
    pub = rospy.Publisher('ee_data', end_effector, queue_size=10)
    #gripper = rospy.Publisher('/iiwa/iiwa_position_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    
    q0 = np.quaternion(pi/2,0,0,0)
    q1 = np.quaternion(0,0,0,0)

    q_sl = quaternion.slerp(q0, q1, 0, 5, np.arange(0, 5, 0.01) )
    q_sl_d = np.diff(q_sl)/0.01
    omega_vect = q_sl_d

    for i in range(498):
        omega = -2 * q_sl[i].conjugate() * q_sl_d[i]
        omega_vect[i] = omega

    while not rospy.is_shutdown():
        for i in range(98):
            data = end_effector()
            time = rospy.get_time()
            data.time = time

            quat_des = quaternion.as_float_array(q_sl[i])
            omega_des = quaternion.as_float_array(omega_vect[i])

            data.position = [quat_des[0], quat_des[1], quat_des[2], 0, 0, 1.2]
            data.velocity = [omega_des[0], omega_des[1], omega_des[2], 1, 1, 1]
            
            pub.publish(data)
            rospy.loginfo(data)
            rate.sleep()


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