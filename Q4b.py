import numpy as np
import math
import matplotlib.pyplot as plt
from sympy import *

#input q1, q2, l1, l2, x_dot

#Jacobian 
J=np.array([[-math.sin(q1)*math.cos(q2)*math.cos(q3)-math.cos(q1)*math.sin(q3)-math.sin(q1)*math.cos(q2)-math.sin(q1), -math.cos(q1)*math.sin(q2)*math.cos(q3)-math.cos(q1)*math.sin(q2), -math.cos(q1)*math.cos(q2)*math.sin(q3)-math.sin(q1)*math.cos(q3)], [math.cos(q1)*math.cos(q2)*math.cos(q3)-math.sin(q1)*math.sin(q3)+math.cos(q1)*math.cos(q2)+math.cos(q1), -math.sin(q1)*math.sin(q2)*math.cos(q3)-math.sin(q1)*math.sin(q2), -math.sin(q1)*math.cos(q2)*math.sin(q3)+math.cos(q1)*math.cos(q3)], [0, -math.cos(q2)*math.cos(q3)-math.cos(q2), math.sin(q2)*math.sin(q3)], [0, -math.sin(q1), math.cos(q1)*math.sin(q2)],[0, math.cos(q1), math.sin(q1)*math.sin(q2)], [1, 0, math.cos(q2)]])
q_dot=numpy.linalg.inv(J)*x_dot
print(q_dot)