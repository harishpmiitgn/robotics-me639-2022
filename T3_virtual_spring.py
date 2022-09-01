import numpy as np
import math
import matplotlib.pyplot as plt

L1=3 #m for first robotic arm
L2=2.5 #m for second robotic arm

# mean position
x0=1
y0=2

# Point after disturbance
y=3
x=4

k=1.8 # N/m (spring constant)

theta=math.acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))
q1=math.atan(y/x)-math.atan((L2*math.sin(theta))/(L1+L2*math.cos(theta)))
q2=q1+theta
Fx=k*(L1*math.cos(q1)+L2*math.cos(q2)-x0)
Fy=k*(L1*math.sin(q1)+L2*math.sin(q2)-y0)
T1=-Fx*L1*math.sin(q1)+Fy*L1*math.cos(q1)
T2=-Fx*L2*math.sin(q2)+Fy*L2*math.cos(q2)
print(q1*180/math.pi)
print(q2*180/math.pi)

#Torque generated to bring the spring back to x0,y0.
print(T1)
print(T2)