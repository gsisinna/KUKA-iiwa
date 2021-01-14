#!/usr/bin/env python

#Rospy and msg definitions
import rospy
from kuka_iiwa_14_prismatic_gripper.msg import end_effector
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from std_msgs.msg import Duration
from std_msgs.msg import Float64

#RBDL model
import rbdl
model = rbdl.loadModel("/home/gabriele/Documents/NRP/GazeboRosPackages/src/kuka_iiwa_14_prismatic_gripper/scripts/iiwa.urdf")

#Numpy and mathematical libraries
import numpy as np
from numpy import pi
from numpy.linalg import inv
from numpy.linalg import norm
from numpy.random import random

import quaternion

#Callback function for joint position update
def jointStateCallback(data):
    #global variables shared between functions
    global pose_des
    global vel_des
    global q
    global q_old
    global qd

    J = np.zeros((6,7))
    h = 0.01                                             #dt
    K = (2/h) * np.eye(6)                              #Stability for < 2/h
    EE_LOCAL_POS = np.array((0.0,0.0,0.045))             #Local offset
    k = 0.001                                           #Damped least square method
    I = np.eye(6)
    
    ## JointState msg
    ##  q = [gripper_left_joint, gripper_right_joint, iiwa_joint_1, iiwa_joint_2, iiwa_joint_3, iiwa_joint_4, iiwa_joint_5, iiwa_joint_6, iiwa_joint_7]
    
    q = data.position[2:]
    q = np.asarray(q)

    q_old = q

    qd = data.velocity[2:]
    qd = np.asarray(qd)


    #End-Effector Position
    x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS, True)  #x,y,z

    #Orientation
    R = rbdl.CalcBodyWorldOrientation(model, q, 7, True).T
    #From rotation matrix to quaternion
    quat = quaternion.from_rotation_matrix(R)
    #Input as euler angles for orientation
    quat_des = quaternion.from_euler_angles(pose_des[0], pose_des[1], pose_des[2])
    #Angular error
    err_ang = quat_des * quat.conjugate()
    err_ang = quaternion.as_float_array(err_ang)
    #Linear error
    err_lin = np.array(([pose_des[3]-x[0], pose_des[4]-x[1], pose_des[5]-x[2]]))

    #Total Error
    err = np.array(([ err_ang[1], err_ang[2], err_ang[3], err_lin[0], err_lin[1], err_lin[2] ]))
    
    # Jacobian (angular-linear)
    rbdl.CalcPointJacobian6D(model, q, 7, EE_LOCAL_POS, J)
    
    # Jacobian Inverse (Damped)
    J_inv = J.T.dot(inv( J.dot(J.transpose()) + (k**2) * I))
    
    #CLIK: Closed Loop Inverse Kinematics
    q = q_old + h * J_inv.dot( vel_des + K.dot(err) )
    qd = J_inv.dot(vel_des + K.dot(err))
    
    #rqt_plot for error
    pub_error = rospy.Publisher('/iiwa/iiwa_position_controller/error', Float64, queue_size=10)
    pub_error.publish(norm(err))
    
    #POSITION CONTROLLER
    pub = rospy.Publisher('/iiwa/iiwa_position_controller/command', JointTrajectory, queue_size=10)
    
    #JointTrajectory msg generation
    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']

    #I'll use a single waypoint with the q[] values calculated with CLIK
    point = JointTrajectoryPoint()
    point.positions = [ q[0], q[1], q[2], q[3], q[4], q[5], q[6] ]
    #point.velocities = [ qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], qd[6] ]
    point.time_from_start = rospy.Duration(1)
    joints_str.points.append(point)
    pub.publish(joints_str)

    #Gripper Controller
    pub_left = rospy.Publisher('/iiwa/gripper_left_position_controller/command', Float64, queue_size=10)
    pub_right = rospy.Publisher('/iiwa/gripper_right_position_controller/command', Float64, queue_size=10)

    #Open Position
    pub_left.publish(0)
    pub_right.publish(0)

    rate.sleep()
    q_old = q

#Callback function for end-effector desired position
def endEffectorCallback(data):
    global pose_des
    global vel_des
    pose_des = np.array([data.position[0], data.position[1], data.position[2], data.position[3], data.position[4], data.position[5] ])    
    vel_des = np.array([data.velocity[0], data.velocity[1], data.velocity[2], data.velocity[3], data.velocity[4], data.velocity[5] ])
    
if __name__ == '__main__':
    pose_des = np.zeros(6)
    vel_des = np.zeros(6)
    q = np.zeros(7)
    q_old = np.zeros(7)
    qd = np.zeros(7)

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/ee_data', end_effector, endEffectorCallback)
    rospy.Subscriber('/iiwa/joint_states', JointState, jointStateCallback)
    rate = rospy.Rate(100)
    rate.sleep()
    rospy.spin()

    