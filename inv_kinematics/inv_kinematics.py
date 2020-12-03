#!/usr/bin/env/python
import rbdl
import numpy as np

time=0
h=0.1

#errore
err = np.Inf
th = 1e-3
#Ke=(1/h)*np.diag(np.full(6,1))
Ke = (2/h) * np.ones(6)
model = rbdl.loadModel("iiwa.urdf")

#Per ora li genero casualmente, ma questi saranno forniti dal topic /iiwa/joint_states
q = np.zeros(7)
q_old = q
qd = np.zeros(7)
qd_old = qd
qdd = np.zeros(7)  
qdd_old = qdd

EE_LOCAL_POS = np.array((0.0,0.0,0.045))

#Questi invece mi vengono dati dall'altro nodo che genera la traiettoria per l'end effector
x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS)
x_des = np.random.rand(3)
xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS)
xd_des = np.random.rand(3)

J = np.zeros((6,7))

while(err>th and time<10000):
    rbdl.CalcPointJacobian6D(model, q, 7, EE_LOCAL_POS, J)
    J_inv = np.linalg.pinv(J) #Penrose Pseudoinverse--> (6,7)
    q = q_old + h * np.matmul(J_inv, (xd+Ke)) 
    rbdl.UpdateKinematics(model, q, qd, qdd)
    x = rbdl.CalcBodyToBaseCoordinates(model, q, 7, EE_LOCAL_POS)
    xd = rbdl.CalcPointVelocity6D(model, q, qd, 7, EE_LOCAL_POS)
    err = np.linalg.norm(x_des - x)
    qd_old = qd
    q_old = q
    time+=1
    print("Q: ", q)
    print("Error: ", err)
    print("Iteration: ", time)
    print("----------------------------------------------------------------------------------------")
