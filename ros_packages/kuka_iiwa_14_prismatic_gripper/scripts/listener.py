#!/usr/bin/env python

import rospy
from kuka_iiwa_14_prismatic_gripper.msg import end_effector
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

import rbdl
model = rbdl.loadModel("/home/gabriele/Documents/NRP/GazeboRosPackages/src/kuka_iiwa_14_prismatic_gripper/scripts/iiwa.urdf")

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from numpy import matmul
from numpy.random import random

#import quaternion

def callback(data):
    rospy.loginfo('Joints Positions: %s', data.position[2:])
    rospy.loginfo('Joints Velocity: %s', data.velocity[2:])
    rospy.loginfo('Time: %s', rospy.get_time() )
    

    it = 0
    h = 0.01
    th = 1e-5
    eps = 1e-3
    K_err = (1/h)*np.diag(np.full(6,1))

    EE_LOCAL_POS = np.array((0.0,0.0,0.045))

    #I valori dei giunti mi verranno forniti dal topic /iiwa/joint_states, per ora uso np.zeros()

    # [gripper_left_joint, gripper_right_joint, iiwa_joint_1, iiwa_joint_2, iiwa_joint_3,
    #   iiwa_joint_4, iiwa_joint_5, iiwa_joint_6, iiwa_joint_7]

    q = data.position[2:]
    q = np.asarray(q) 
    q_old = q

    qd = data.velocity[2:]
    qd = np.asarray(qd) 
    qd_old = qd

    qdd = np.zeros(7)  
    qdd_old = qdd

    #Questi invece mi vengono dati dall'altro nodo che genera la traiettoria per l'end effector, talker.py
    x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS) # 3 componenti
    rospy.loginfo('EndEffector Position: %s', x)
    x_des = np.array([0, 0, 1.306])

    xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS) # vettore di 6 componenti
    rospy.loginfo('EndEffector Velocity: %s', xd)
    xd_des = np.zeros(6)

    R = rbdl.CalcBodyWorldOrientation(model, q, 7).T #matrice di rotazione  

    #Orientamento RPY
    roll = np.arctan2(R[1,0], R[0,0])
    pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
    yaw = np.arctan2(R[2,1], R[2,2])

    pose = np.array([x[0], x[1], x[2], roll, pitch, yaw])
    pose_des = np.array([x_des[0], x_des[1], x_des[2], roll, pitch, yaw])

    rospy.loginfo('EndEffector Pose: %s', pose)

    err = pose_des - pose

    J = np.zeros((6,7))
    k=0.001 #damping factor?

    while(1):
        rbdl.CalcPointJacobian6D(model, q, 7, EE_LOCAL_POS, J)
        rospy.loginfo('Jacobian: %s', J)
        J_inv = matmul(J.transpose(), inv(matmul(J, J.transpose())+(k**2)*np.ones((6,6))))
        q = q_old + h * matmul(J_inv, (xd+matmul(K_err,err))) 
    
        rbdl.UpdateKinematics(model, q, qd, qdd)
        x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS)
        xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS)
    
        R = rbdl.CalcBodyWorldOrientation(model, q, 7).T

        roll = np.arctan2(R[1,0], R[0,0])
        pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
        yaw = np.arctan2(R[2,1], R[2,2])

        pose = np.array([x[0], x[1], x[2], roll, pitch, yaw])
        pose_des = np.zeros(6)
        rospy.loginfo('Pose: %s', pose)
        
        err = pose_des - pose
    
        qd_old = qd
        q_old = q
        #time = rospy.get_rostime()
        it+=1
    
        print("Q: ", q)
        print("Error: ", norm(err))
        print("Iteration: ", it)
        print("----------------------------------------------------------------------------------------")
        
        

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/iiwa/joint_states', JointState, callback)
    rospy.spin()

#QUATERNIONI
# teta = np.arccos((R[0,0]+R[1,1]+R[2,2]-1)/2)
# omega = (0.5*np.sin(teta))*np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
# pose = np.quaternion(teta, omega[0], omega[1], omega[2])
# pose = pose / np.absolute(pose)
# pose_des = np.quaternion(1, 1, 1, 1)
# pose_des = pose_des / np.absolute(pose_des)
# err = np.multiply(pose_des, np.conjugate(pose))

if __name__ == '__main__':
    listener()