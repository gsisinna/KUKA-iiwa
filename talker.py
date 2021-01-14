#!/usr/bin/env python

#Mathematical libraries
import numpy as np
from numpy.random import random
from numpy import sin
from numpy import cos
from numpy import pi

#Rospy and msg definitions
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

# Msg "end_effector.msg" structure
# Header header

# float64[] orientation (euler angles) - position
# float64[] velocity
# float64 time

def talker():
    pub = rospy.Publisher('ee_data', end_effector, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 100hz

    #Quaternion for example orientation
    q0 = quaternion.from_spherical_coords(0,0)
    q1 = quaternion.from_spherical_coords(pi,0)
    q2 = quaternion.from_spherical_coords(pi/2,0)

    while not rospy.is_shutdown():
        data = end_effector()
        time = rospy.get_time()
        data.time = time
        dt = 0.1
        # #Modify next value will change end-effector position and velocity
        # data.position = [0,0,0,0,0,0.7]
        # data.velocity = [1,1,1,1,1,1]
        # pub.publish(data)
        # rospy.loginfo(data)
        # rate.sleep()

        #Some test for trajectory execution

        if time<10:
            q_sl = quaternion.slerp(q0, q1, 0, 10, time)
            q_sl_old = quaternion.slerp(q0, q1, 0, 10, time-1)
            q_sl_d = (q_sl - q_sl_old)/dt
            
            w_sl = -2 * q_sl.conjugate() * q_sl_d
            w_sl = quaternion.as_float_array(w_sl)
            
            q_sl = quaternion.as_euler_angles(q_sl)
            
            data.position = [0,0,0, 0.5, 0.5, 0.5]
            data.velocity = [10,10,10,10,10,10]
            
            pub.publish(data)
            rate.sleep()


        elif time>=10 and time<20:
            q_sl = quaternion.slerp(q1, q2, 10, 20, time)
            q_sl_old = quaternion.slerp(q1, q2, 10, 20, time-1)

            q_sl_d = (q_sl - q_sl_old)/dt
            
            w_sl = -2 * q_sl.conjugate() * q_sl_d
            w_sl = quaternion.as_float_array(w_sl)
            
            q_sl = quaternion.as_euler_angles(q_sl)
            
            data.position = [0,0,0, 0, 0, 1]
            data.velocity = [10,10,10, 10, 10, 10]
            
            pub.publish(data)
            rate.sleep()

        else:
            q_sl = quaternion.as_euler_angles(q0)

            data.position = [0,0,0, 0, 0, 1.2]
            data.velocity = [0,0,0,0,0,0]

            pub.publish(data)
            rate.sleep()


if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass