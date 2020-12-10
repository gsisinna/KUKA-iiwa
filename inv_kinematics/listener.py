#!/usr/bin/env python

import rospy
from kuka_iiwa_14_prismatic_gripper.msg import end_effector
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from std_msgs.msg import Duration

import rbdl
model = rbdl.loadModel("/home/gabriele/Documents/NRP/GazeboRosPackages/src/kuka_iiwa_14_prismatic_gripper/scripts/iiwa.urdf")

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from numpy import matmul
from numpy.random import random
#import quaternion

pose_des = np.zeros(6)
command = np.zeros(7)

def jointStateCallback(data):
    global pose_des
    global command
    global q
    global q_old

    #rospy.loginfo('Joints Positions: %s', data.position)
    #rospy.loginfo('Joints Velocity: %s', data.velocity)
    
    J = np.zeros((6,7))
    h = 0.01
    it = 0
    K = (1/h) * np.diag(np.full(6,1))
    EE_LOCAL_POS = np.array((0.0,0.0,0.045))
    k=0.001
    
    # q = [gripper_left_joint, gripper_right_joint, iiwa_joint_1, iiwa_joint_2, iiwa_joint_3, iiwa_joint_4, iiwa_joint_5, iiwa_joint_6, iiwa_joint_7]
    # VALORI DEI GIUNTI DAL TOPIC /JOINT_STATES
    q = data.position[2:]
    q = np.asarray(q)
    q_old = q

    qd = data.position[2:]
    qd = np.asarray(qd)
    #CALCOLO POSIZIONE DELL'END-EFFECTOR
    x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS)
    xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS)
    #ORIENTAMENTO
    R = rbdl.CalcBodyWorldOrientation(model, q, 7).T
    roll = np.arctan2(R[1,0], R[0,0])
    pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
    yaw = np.arctan2(R[2,1], R[2,2])
    #POSA DELL'END-EFFECTOR
    pose = np.array([x[0], x[1], x[2], roll, pitch, yaw])
    #ERRORE (VETTORIALE)
    err = pose_des - pose

    #QUI INIZIAVA IL VECCHIO WHILE CHE MI MANDAVA IN LOOP IL PROGRAMMA
    # CALCOLO JACOBIANO ANGULAR_LINEAR
    rbdl.CalcPointJacobian6D(model, q, 7, EE_LOCAL_POS, J)
    rospy.loginfo('Jacobian: %s', J)
    # INVERSA + DAMPING
    J_inv = matmul(J.transpose(), inv(matmul(J, J.transpose())+(k**2)*np.ones((6,6))))
    # EULERO IN AVANTI PER Q
    q = q_old + h * matmul(J_inv, (xd+matmul(K,err)))
    rospy.loginfo("Comando da inviare al controllore (q_des): %s", q)
         
    command = q  #questa e'' la variabile globale con le posizioni dei giunti che pubblichero' sul topic del controllore
    q_old = q
    it+=1

    pub = rospy.Publisher('/iiwa/iiwa_position_controller/command', JointTrajectory, queue_size=100)
    
    #TEST CON EFFORT CONTROLLER
    # cmd = Float64MultiArray()
    # cmd.layout.dim.append(MultiArrayDimension(label='torques',size=7,stride=1))
    # cmd.data = list(command)
    # pub.publish(cmd)
    # rospy.loginfo("Comando inviato")

    #TEST CON POSITION CONTROLLER
    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    #joints_str.joint_names = ['gripper_left_joint', 'gripper_right_joint', 'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
    joints_str.joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
    point = JointTrajectoryPoint()
    #point.positions = [0.0, 0.0, command[0], command[1], command[2], command[3], command[4], command[5], command[6]]
    point.positions = [0.0]
    #point.velocities = [0.0, 0.0, command[0], command[1], command[2], command[3], command[4], command[5], command[6]]
    point.velocities = [0.0]
    point.time_from_start = rospy.Duration(5)
    joints_str.points.append(point)
    pub.publish(joints_str)
    rospy.loginfo("COMANDO INVIATO: %s", joints_str)

def endEffectorCallback(data):
    global pose_des 
    rospy.loginfo('End-Effector desired position: %s', data.position)
    pose_des = np.array([data.position[0], data.position[1], data.position[2], data.position[3], data.position[4], data.position[5] ])

     
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/ee_data', end_effector, endEffectorCallback)
    rospy.Subscriber('/iiwa/joint_states', JointState, jointStateCallback)
    

    
if __name__ == '__main__':
    listener()
    rospy.spin()

#QUATERNIONI
# teta = np.arccos((R[0,0]+R[1,1]+R[2,2]-1)/2)
# omega = (0.5*np.sin(teta))*np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
# pose = np.quaternion(teta, omega[0], omega[1], omega[2])
# pose = pose / np.absolute(pose)
# pose_des = np.quaternion(1, 1, 1, 1)
# pose_des = pose_des / np.absolute(pose_des)
# err = np.multiply(pose_des, np.conjugate(pose))