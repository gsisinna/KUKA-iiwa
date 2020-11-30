#!/usr/bin/env/python

import numpy as np
from numpy import array

cos = np.cos
sin = np.sin
pi = np.pi

# Parametri DH
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = 0, -pi/2, -pi/2, -pi/2, -pi/2, -pi/2, -pi/2
a0, a1, a2, a3, a4, a5, a6 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
d1, d2, d3, d4, d5, d6, d7, d_EE = 0.1575, 0.2025, 0.2045, 0.2155, 0.1845, 0.2155, 0.081, 0.045

# Variabili di giunto e test
#q1, q2, q3, q4, q5, q6, q7 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
#q = np.random.rand(7)


def createMatrix(alpha, a, q, d):
    mat = np.matrix([[cos(q),             -sin(q)*cos(alpha),           sin(q)*sin(alpha),              a*cos(q)],
                     [sin(q),             cos(q)*cos(alpha),           -cos(q)*sin(alpha),              a*sin(q)],
                     [     0,                    sin(alpha),                   cos(alpha),                     d],
                     [     0,                             0,                            0,                     1]])

    return mat


def forwardKinematics(q):

    # Homogeneous Transforms
    T0_1 = createMatrix(alpha0, a0, q[0], d1)
    T1_2 = createMatrix(alpha1, a1, q[1], d2)
    T2_3 = createMatrix(alpha2, a2, q[2], d3)
    T3_4 = createMatrix(alpha3, a3, q[3], d4)
    T4_5 = createMatrix(alpha4, a4, q[4], d5)
    T5_6 = createMatrix(alpha5, a5, q[5], d6)
    T6_7 = createMatrix(alpha6, a6, q[6], d7)
    T7_EE = createMatrix(0, 0, 0, d_EE)

    # Composition of Homogenous Transforms
    T0_2 = T0_1 * T1_2  # base_link to link 2
    T0_3 = T0_2 * T2_3  # base_link to link 3
    T0_4 = T0_3 * T3_4  # base_link to link 4
    T0_5 = T0_4 * T4_5  # base_link to link 5
    T0_6 = T0_5 * T5_6  # base_link to link 6
    T0_7 = T0_6 * T6_7  # base_link to link 7
    T0_EE = T0_7 * T7_EE

    #print(T0_EE)
    x = T0_EE[0:3, 3]
    R = T0_EE[0:3, 0:3]
    
    return(x, R)

if __name__ == '__main__':
    forwardKinematics()