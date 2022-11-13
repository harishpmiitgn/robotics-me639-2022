#SCARA manipulator
import numpy as np
import math
import matplotlib.pyplot as plt
from sympy import *

#input q1, q2, l1, l2, x_dot

#Jacobian for SCARA
J=np.array([[-l2*math.sin(q1+q2)-l1*math.sin(q1), -l2*math.sin(q1+q2), 0, 0], [l2*math.cos(q1+q2)+l1*math.cos(q1), l2*math.cos(t1+t2), 0, 0], [0, 0, 0, -1], [0, 0, 0, 0],[0, 0, 0, 0], [1, 1, 1, 0]])
q_dot=numpy.linalg.inv(J)*x_dot