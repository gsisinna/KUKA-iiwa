import numpy as np
from numpy.random import random
from numpy import sin
from numpy import cos
from numpy import pi
import quaternion

def mySlerp(t,q0,qf):
    teta = np.linalg.norm(quaternion.as_rotation_vector( (q0.conjugate() * qf )))
    t0 = 0
    tf = 10
    dt = (t-t0)/(tf-t0)
    quat_slerp = (sin(teta/2*(1-dt))*q0 + sin(teta/2*dt)*qf)/sin(teta/2)
    return quat_slerp
    
q0=np.quaternion(0,0,0,0)
qf=np.quaternion(pi/2,0,0,0)

q = mySlerp(10,q0,qf)
print(q)