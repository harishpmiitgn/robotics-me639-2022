import numpy as np
import math
import matplotlib.pyplot as plt

L1=3 #m for first robotic arm
L2=2.5 #m for second robotic arm

N_x=5 #no of divisions in x for simulation

theta_start=35
theta_end=145

x1=[] #x coordinate for link 1
x2=[] #x coordinate for link 2
y1=[] #y coordinate for link 1
y2=[] #y coordinate for link 2

x0=0
y0=0
theta=0

for q1 in range(theta_start,theta_end,1):
    for q2 in range(theta_start,theta_end,1):
        #Forward Kinematics
        x=L1*math.cos(q1/180*np.pi)+L2*math.cos(q2/180*np.pi)
        y=L1*math.sin(q1/180*np.pi)+L2*math.sin(q2/180*np.pi)
        print(x)
        print(y)
        plt.xlim([-12,12])
        plt.ylim([-6,6])
        plt.plot(x,y,'o')
        plt.show()

