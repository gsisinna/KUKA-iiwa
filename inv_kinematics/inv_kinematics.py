#!/usr/bin/env/python
import rbdl
import numpy as np

time=0
h=0.01#errore
err = np.Inf
th = 1e-3
Ke=0.001*np.ones(6)

model = rbdl.loadModel("iiwa.urdf")

#Per ora li genero casualmente, ma questi saranno forniti dal topic /iiwa/joint_states
q = np.random.rand(7)
q_old = q
#qd = np.random.rand(7) 
qd = np.zeros(7)
qd_old = qd
qdd = np.zeros(7)  
qdd_old = qdd

EE_LOCAL_POS = np.array((0.0,0.0,0.045))

#Questi invece mi vengono dati dall'altro nodo che genera la traiettoria per l'end effector
x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS)
x_des = x
xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS)
xd_des = xd

J_angular_linear = np.zeros((6,7))
while(err>th and time<1000):
    rbdl.CalcPointJacobian6D(model, q, 7, EE_LOCAL_POS, J_angular_linear)
    J_inv = np.linalg.pinv(J_angular_linear) #Penrose Pseudoinverse--> (6,7)
    q = q_old + h * np.dot(J_inv, (xd+Ke)) 
    x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS)
    err = np.linalg.norm(x_des - x)
    rbdl.UpdateKinematics(model, q, qd, qdd)
    xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS)
    #xd = np.dot(J_angular_linear, qd)
    qd_old = qd
    q_old = q
    time = time+1
    print("Q: ", q)
    print("Error: ", err)
    print("Time: ", time)
