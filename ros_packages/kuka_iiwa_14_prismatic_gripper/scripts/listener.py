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

def jointStateCallback(data):
    global pose_des
    global vel_des
    global q
    global q_old
    global qd

    J = np.zeros((6,7))
    alpha = 10
    h = 0.01                                           #dt
    K = (1.5/h) * np.diag(np.full(6,1))                  #Stability for eigenvalus < 2/h
    EE_LOCAL_POS = np.array((0.0,0.0,0.045))
    k = 0.0001                                          #Damped least square method

    # q = [gripper_left_joint, gripper_right_joint, iiwa_joint_1, iiwa_joint_2, iiwa_joint_3, iiwa_joint_4, iiwa_joint_5, iiwa_joint_6, iiwa_joint_7]
    
    q = data.position[2:]
    q = np.asarray(q)

    q_old = q

    qd = data.velocity[2:]
    qd = np.asarray(qd)


    #CALCOLO POSIZIONE DELL'END-EFFECTOR
    x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS)  #x,y,z
    xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS)   #vx,vy,vz,wx,wy,wx

    #Orientation
    R = rbdl.CalcBodyWorldOrientation(model, q, 7).T
    quat = quaternion.from_rotation_matrix(R)
    quat_des = quaternion.from_euler_angles(pose_des[0], pose_des[1], pose_des[2])
    quat_des.normalized()
    err_quat = quat_des * quat.conjugate()
    err_quat = quaternion.as_float_array(err_quat)
    
    err_lin = np.array(([pose_des[3]-x[0], pose_des[4]-x[1], pose_des[5]-x[2]]))

    #ERRORE (angular-linear)
    err = np.array(([ err_quat[1], err_quat[2], err_quat[3], err_lin[0], err_lin[1], err_lin[2] ]))
    rospy.loginfo("ERRORE: %s", norm(err) )
    
    # CALCOLO JACOBIANO ANGULAR_LINEAR
    rbdl.CalcPointJacobian6D(model, q, 7, EE_LOCAL_POS, J)
    
    # INVERSA + DAMPING (IMPROVED)
    l = 0.2 #lunghezza caratteristica dei link
    w = 1e-4 * l**3
    I = 1 * np.diag(np.full(6,1))
    
    J_inv = J.T.dot(inv( J.dot(J.transpose()) + (k**2) * I + w * np.diag(np.full(6,1))))
    
    #J_inv = J.T.dot(inv( J.dot(J.transpose()) + (k**2) * I ))

    
    #CLIK
    q = q + h * alpha * J_inv.dot( vel_des + K.dot(err) )
    #qd = J_inv.dot(vel_des + K.dot(err))
    qd = (q - q_old) / h
    
    #rqt_plot for the error
    pub_error = rospy.Publisher('/iiwa/iiwa_position_controller/error', Float64, queue_size=10)
    pub_error.publish(norm(err))
    pub = rospy.Publisher('/iiwa/iiwa_position_controller/command', JointTrajectory, queue_size=10)
    
    #POSITION CONTROLLER
    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']

    point = JointTrajectoryPoint()
    point.positions = [ q[0], q[1], q[2], q[3], q[4], q[5], q[6] ]
    point.velocities = [ qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], qd[6] ]
    point.time_from_start = rospy.Duration(5)
    joints_str.points.append(point)
    pub.publish(joints_str)
    rospy.loginfo(joints_str)

    q_old = q


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

    