#SCARA manipulator
import numpy as np
import math
import matplotlib.pyplot as plt
from sympy import *

dx=5
dy=6
a1=2
a2=2
cos_alpha=0.5
sin_alpha=0.86
#Let rotation matrix obtained from forward kinematics be
R=np.array([[cos_alpha, sin_alpha, 0],[sin_alpha, -cos_alpha, 0], [0, 0, -1]])
alpha=math.atan(sin_alpha/cos_alpha)
r=((dx**2+dy**2-a1**2-a2**2)/(2*a1*a2))**(1/2)
theta2=math.atan(r/(1-r**2)**(1/2))
theta1=math.atan(dy/dx)-math.atan((a2*math.sin(theta2))/(a1+a2*math.cos(theta2)))
theta4=theta1+theta2-alpha
print(theta1)
print(theta2)
print(theta4)