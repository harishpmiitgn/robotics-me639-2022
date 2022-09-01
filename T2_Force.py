import numpy as np
import math
import matplotlib.pyplot as plt

L1=3 #m for first robotic arm
L2=2.5 #m for second robotic arm

N_x=5 #no of divisions in x for simulation

y=3
x=4

x1=[] #x coordinate for link 1
x2=[] #x coordinate for link 2
y1=[] #y coordinate for link 1
y2=[] #y coordinate for link 2

x0=0
y0=0
theta=0

F=20 # Newtons

theta=math.acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))
q1=math.atan(y/x)-math.atan((L2*math.sin(theta))/(L1+L2*math.cos(theta)))
q2=q1+theta
x1=L1*math.cos(q1)
y1=L1*math.sin(q1)
x2=x1+L2*math.cos(q2)
y2=y1+L2*math.sin(q2)
Fx=F*math.cos(q2)
Fy=F*math.sin(q2)
#T1 and T2 are torque required for generation of required force
T1=-Fx*L1*math.sin(q1)+Fy*L1*math.cos(q1)
T2=-Fx*L2*math.sin(q2)+Fy*L2*math.cos(q2)
print(q1*180/math.pi)
print(q2*180/math.pi)
print(T1)
print(T2)
   
plt.xlim([0,6])
plt.ylim([-1,4])
plt.plot(x,y,'o')
plt.plot([x0,x1],[y0,y1])
plt.plot([x1,x2],[y1,y2])
plt.show()
   