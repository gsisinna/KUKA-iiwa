#!/usr/bin/env/python
import rbdl
import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from numpy import matmul
#import quaternion

time=0
h=0.001

#errore
th = 1e-5
eps = 1e-3
K_err = (1/h)*np.diag(np.full(6,1))
model = rbdl.loadModel("iiwa.urdf")
EE_LOCAL_POS = np.array((0.0,0.0,0.045))

#Per ora li genero casualmente, ma questi saranno forniti dal topic /iiwa/joint_states
q = np.random.rand(7)
q_old = q

qd = np.random.rand(7)
qd_old = qd

qdd = np.zeros(7)  
qdd_old = qdd

#Questi invece mi vengono dati dall'altro nodo che genera la traiettoria per l'end effector
x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS)
x_des = np.array([0, 0, 1.306])

xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS)
xd_des = np.zeros(6)

R = rbdl.CalcBodyWorldOrientation(model, q, 7).T

roll = np.arctan2(R[1,0], R[0,0])
pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
yaw = np.arctan2(R[2,1], R[2,2])

pose = np.array([x[0], x[1], x[2], roll, pitch, yaw])
pose_des = np.zeros(6)

err = pose_des - pose

J = np.zeros((6,7))
k=0.001

while(norm(err)>th and time<15 and norm(q-q_old)<eps):
    rbdl.CalcPointJacobian6D(model, q, 7, EE_LOCAL_POS, J)
    #J_inv = np.linalg.pinv(J) #Penrose Pseudoinverse--> (6,7)
    J_inv = matmul(J.transpose(), inv(matmul(J, J.transpose())+(k**2)*np.ones((6,6))))
    q = q_old + h * matmul(J_inv, (xd+matmul(K_err,err))) 
    
    rbdl.UpdateKinematics(model, q, qd, qdd)
    x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS)
    xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS)
    
    R = rbdl.CalcBodyWorldOrientation(model, q, 7).T
    err = pose_des-pose
    
    qd_old = qd
    q_old = q
    time+=1
    
    print("Q: ", q)
    print("Error: ", err)
    print("Iteration: ", time)
    print("----------------------------------------------------------------------------------------")


#QUATERNIONI
# teta = np.arccos((R[0,0]+R[1,1]+R[2,2]-1)/2)
# omega = (0.5*np.sin(teta))*np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
# pose = np.quaternion(teta, omega[0], omega[1], omega[2])
# pose = pose / np.absolute(pose)
# pose_des = np.quaternion(1, 1, 1, 1)
# pose_des = pose_des / np.absolute(pose_des)
# err = np.multiply(pose_des, np.conjugate(pose))
