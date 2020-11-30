#!/usr/bin/env/python

import rbdl
import numpy as np
import forwardKinematics as fk

time=0
h=0.1

#errore
err = np.Inf
th = 0.001
#Ke = np.array([[2/h-1, 0, 0], [0,2/h-1, 0], [0, 0, 2/h-1]])
Ke=19*np.ones(6)

model = rbdl.loadModel("iiwa.urdf")

#Per ora li genero casualmente, ma questi saranno forniti dal topic /iiwa/joint_states
q = np.random.rand(7)
q_old = q

qd = np.random.rand(7) 
qd_old = q

qdd = np.zeros(7)  
qdd_old = qdd

#Questi invece mi vengono dati dall'altro nodo che genera la traiettoria per l'end effector
x = np.random.rand(6)
x_des = np.random.rand(6)

xd = np.random.rand(6)
xd_des = np.random.rand(6)

J = np.zeros((6,7))
J_angular_linear = np.zeros((6,7))

EE_LOCAL_POS = np.array((0.0,0.0,0.045))

rbdl.CalcPointJacobian6D(model, q, 7, EE_LOCAL_POS, J_angular_linear)
J_inv = np.linalg.pinv(J_angular_linear) #Penrose Pseudoinverse--> (6,7)

while(err>th):
    #qd = np.dot(J_inv, (xd + np.dot(Ke,(x_des-x))) + (np.ones((7,7)) - np.dot(J_inv, J_angular_linear))) * qd_old
    q = q_old + h*np.dot(J_inv, (xd_des+Ke))
    [x, R] = fk.forwardKinematics(q)
    err = np.linalg.norm(x_des - x)
    
    qd_old = qd
    q_old = q
    time = time+1
    print("Q: ", q)
    print("Error: ", err)


# q = [global_base_position, global_base_quaternion_x, global_base_quaternion_y, global_base_quaternion_z, joint_positions, global_base_quaternion_w]
# v = [global_base_velocity_linear, local_base_velocity_angular, joint_velocities]
# a = [global_base_acceleration_linear, local_base_acceleration_angular, joint_accelerations]

