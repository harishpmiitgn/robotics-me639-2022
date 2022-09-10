import numpy as np
import math
import matplotlib.pyplot as plt

q1=30*np.pi/360
q2=60*np.pi/360
a1=2
a2=2.5

#Jacobian for SCARA manipulator
J= ([[-a1*math.sin(q1)-a2*math.sin(q1)*math.sin(q2), -a2*math.sin(q1)*math.sin(q2), 0, 0], [a1*math.cos(q1)+a2*math.cos(q1)*math.cos(q2), a2*math.cos(q1)*math.cos(q2), 0, 0], [0, 0, -1, 0], [0, 0, 0, 0], [0, 0, 0, 0], [1, 1, 0, -1]])
print(J)