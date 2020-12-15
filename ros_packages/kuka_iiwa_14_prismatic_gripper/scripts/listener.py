#!/usr/bin/env python

import rospy
from kuka_iiwa_14_prismatic_gripper.msg import end_effector
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from std_msgs.msg import Duration
from std_msgs.msg import Float64

import rbdl
model = rbdl.loadModel("/home/gabriele/Documents/NRP/GazeboRosPackages/src/kuka_iiwa_14_prismatic_gripper/scripts/iiwa.urdf")

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from numpy import matmul
from numpy.random import random
import quaternion

pi = np.pi

pose_des = np.zeros(6)

def jointStateCallback(data):
    global pose_des
    
    J = np.zeros((6,7))
    h = 0.001
    K = (1/h) * np.diag(np.full(6,1))
    EE_LOCAL_POS = np.array((0.0,0.0,0.045))
    k = 0.0001

    #Test giunti pesati diversamente per proiettore nel nullspace
    I = np.ones((6,6))
    I[0,0] = 100
    I[1,1] = 50
    I[2,2] = 25
    I[3,3] = 10
    I[4,4] = 5
    I[5,5] = 0
    
    # q = [gripper_left_joint, gripper_right_joint, iiwa_joint_1, iiwa_joint_2, iiwa_joint_3, iiwa_joint_4, iiwa_joint_5, iiwa_joint_6, iiwa_joint_7]
    q = data.position[2:]
    q = np.asarray(q)

    qd = data.velocity[2:]
    qd = np.asarray(qd)

    #CALCOLO POSIZIONE DELL'END-EFFECTOR
    x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS)
    xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS)

    #rospy.loginfo("Posizione End-Effector: %s", x)

    #ORIENTAMENTO --> utilizzare definizione diversa, jacobiano analitico
    R = rbdl.CalcBodyWorldOrientation(model, q, 7).T
    
    quat = quaternion.from_rotation_matrix(R)
    quat_des = np.quaternion(1, pose_des[0], pose_des[1], pose_des[2])
    quat_des = quat_des ## devo prendere il modulo per rendere il quaternione unitario?
    err_quat = quat_des * quat.conjugate()
    #err_quat = quat * quat_des.conjugate()
    err_quat = quaternion.as_float_array(err_quat)
    err_lin = np.array(([pose_des[3]-x[0], pose_des[4]-x[1], pose_des[5]-x[2]]))

    #ERRORE (angular-linear)
    err = np.array(([ err_quat[1], err_quat[2], err_quat[3], err_lin[0], err_lin[1], err_lin[2] ]))
    rospy.loginfo("ERRORE: %s", norm(err) )
    
    # CALCOLO JACOBIANO ANGULAR_LINEAR
    rbdl.CalcPointJacobian6D(model, q, 7, EE_LOCAL_POS, J)
    
    # INVERSA + DAMPING
    J_inv = J.T.dot(inv( J.dot(J.transpose()) + (k**2) * np.ones((6,6) ) ) )
    
    #Giunti pesati diversamente con matrice I
    #J_inv = J.T.dot(inv( J.dot(J.transpose()) + (k**2) * I ) )

    # EULERO IN AVANTI PER Q: CLIK
    q = q + h * J_inv.dot( xd + K.dot(err) )  #err -> 3 parte vettoriale quat e 3 posizione

    pub = rospy.Publisher('/iiwa/iiwa_position_controller/command', JointTrajectory, queue_size=100)
    pub_error = rospy.Publisher('/iiwa/iiwa_position_controller/error', Float64, queue_size=100)
    
    #POSITION CONTROLLER
    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
    point = JointTrajectoryPoint()
    #point.positions = [0, 0, 0, 0, 0, 0, 0]
    point.positions = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    point.time_from_start = rospy.Duration(2)
    joints_str.points.append(point)
    pub.publish(joints_str)
    pub_error.publish(norm(err))
    rospy.loginfo("COMANDO INVIATO: %s", joints_str)


def endEffectorCallback(data):
    global pose_des
    pose_des = np.array([data.position[0], data.position[1], data.position[2], data.position[3], data.position[4], data.position[5] ])    

    
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/ee_data', end_effector, endEffectorCallback)
    rospy.Subscriber('/iiwa/joint_states', JointState, jointStateCallback)
    rospy.spin()