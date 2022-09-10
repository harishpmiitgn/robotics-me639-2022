import numpy as np
import math
import matplotlib.pyplot as plt

L1=3 #m for first robotic arm
L2=2.5 #m for second robotic arm

N_x=5 #no of divisions in x for simulation

x_start=1
x_end=6

x1=[] #x coordinate for link 1
x2=[] #x coordinate for link 2
y1=[] #y coordinate for link 1
y2=[] #y coordinate for link 2

x0=0
y0=0
theta=0

figno=1

for x in range(x_start,x_end+1,1):
    y=math.sin(x)
    theta=math.acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))
    temp_q1=math.atan(y/x)-math.atan((L2*math.sin(theta))/(L1+L2*math.cos(theta)))
    temp_q2=temp_q1+theta
    temp_x1=L1*math.cos(temp_q1)
    temp_y1=L1*math.sin(temp_q1)
    temp_x2=temp_x1+L2*math.cos(temp_q2)
    temp_y2=temp_y1+L2*math.sin(temp_q2)
    print(temp_x2)
    #x1.append(temp_x1)
    #x2.append(temp_x2)
    #y1.append(temp_y1)
    #y2.append(temp_y2)
    #print(y2)
    #filename=str(figno)+'.jpg'
    #figno=figno+1
    #plt.figure()
    plt.xlim([0,12])
    plt.ylim([-3,1.5])
    t=np.arange(0,3.5*np.pi,0.1)
    y_sin=np.sin(t)
    plt.plot(t,y_sin)
    plt.plot([x0,temp_x1],[y0,temp_y1])
    plt.plot([temp_x1,temp_x2],[temp_y1,temp_y2])
    plt.show()
    #plt.savefig(filename)