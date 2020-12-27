import numpy as np
from numpy import pi

import quaternion

q0 = np.quaternion(pi/2,0,0,0)
q1 = np.quaternion(pi,0,0,0)

q_sl = quaternion.slerp(q0, q1, 1, 10, np.arange(1,10,1) )
q_sl_d = np.diff(q_sl) / 0.01
omega_vect = q_sl_d

print(q_sl)
print(q_sl_d)

for i in range(8):
    omega = -2 * q_sl[i].conjugate() * q_sl_d[i]
    omega_vect[i] = omega

print(omega_vect)