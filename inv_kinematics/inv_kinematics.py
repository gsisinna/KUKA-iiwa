#!/usr/bin/env/python

import rbdl
import numpy as np

h=0.1
Ke = 2/h * np.ones((6,7)) #Stabilità per valori minori di 2/h

#errore
err_old = np.Inf
err = err_old

#threshold
th = 0.001

model = rbdl.loadModel("iiwa.urdf")

q = np.random.rand(model.q_size)
qd = np.random.rand(model.qdot_size) 
qdd = np.zeros(model.qdot_size)  

q_des = q
pos = np.random.rand(3)

J = np.zeros((6,7))
J_angular_linear = np.zeros((6,7))

rbdl.CalcPointJacobian6D(model, q, 7, np.zeros(3), J_angular_linear)
rbdl.InverseKinematics(model, q, np.ones(7), np.zeros(3), pos, q_des)

# q = [global_base_position, global_base_quaternion_x, global_base_quaternion_y, global_base_quaternion_z, joint_positions, global_base_quaternion_w]
# v = [global_base_velocity_linear, local_base_velocity_angular, joint_velocities]
# a = [global_base_acceleration_linear, local_base_acceleration_angular, joint_accelerations]

