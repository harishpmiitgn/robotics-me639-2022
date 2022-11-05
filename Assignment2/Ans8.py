import math
from math import sin, cos
import numpy as np

def J(a1,a2,q1,q2): #a1 and a2 are arm lengths and q1 and q2 are required joint variables (angles in degrees).
    q1 = math.radians(q1)
    q2 = math.radians(q2)
    mat = np.zeros((6,3))
    mat[0][0] = -(a2*sin(q1+q2) + a1*sin(q1))
    mat[0][1] = -(a2*sin(q1+q2))
    mat[1][0] = (a2*cos(q1+q2) + a2*cos(q1))
    mat[1][0] = a2*cos(q1+q2) 
    mat[2][2] = -1
    mat[5][0] = 1
    mat[5][1] = 1
    return mat

Jacobian = J(3,4,30,60)
print(Jacobian)