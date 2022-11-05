import math
from math import sin, cos
import numpy as np

def J(a1,a2,a3,q1,q2,q3): #a1 and a2 are arm lengths and q1 and q2 are required joint variables (angles in degrees).
    q1 = math.radians(q1)
    q2 = math.radians(q2)
    q3 = math.radians(q3)
    mat = np.zeros((6,3))
    mat[0][0] = -(a3*sin(q1+q2+q3) + a2*sin(q1+q2) + a1*sin(q1))
    mat[0][1] = -(a3*sin(q1+q2+q3) + a2*sin(q1+q2))
    mat[0][2] = -a3*sin(q1+q2+q3) 
    mat[1][0] = (a3*cos(q1+q2+q3) + a2*cos(q1+q2) + a2*cos(q1))
    mat[1][0] = a3*cos(q1+q2+q3) + a2*cos(q1+q2)
    mat[1][2] = a3*cos(q1+q2+q3)
    mat[5][0] = 1
    mat[5][1] = 1
    mat[5][2] = 1
    return mat

Jacobian = J(3,4,5,30,60,-30)
print(Jacobian)