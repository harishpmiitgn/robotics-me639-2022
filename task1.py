# -*- coding: utf-8 -*-
"""
Created on Sat Aug  6 21:09:59 2022

@author: turpa
"""
import math
import lmfit
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
x=5
y=3
l1=4
l2=3
#x=int(input("enter the value of x="))
#y=int(input("enter the value of y="))
#l1=int(input("enter the value of l1="))
#l2=int(input("enter the value of l2="))
def anglecalc(x,y,l1,l2):
    D=((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
    theta=math.degrees(math.atan(math.sqrt(1-D**2)/D))
    q1=math.degrees(math.atan(y/x))-math.degrees(math.atan((l2*math.sin(math.radians(theta)))/(l1+l2*math.cos(math.radians(theta)))))
    q2=q1+theta
    return(q1,q2,theta)

q1,q2,theta = anglecalc(x, y, l1, l2)
xout=l1*math.cos(math.radians(q1))+l2*math.cos(math.radians(q2))
yout=l1*math.sin(math.radians(q1))+l2*math.sin(math.radians(q2))
a=[]
b=[]
q1dis=np.linspace(0, q1,10)
for i in range(0,10):
    x=l1*math.cos(math.radians(q1dis[i]))+l2*math.cos(math.radians(q1dis[i]))
    a.append(x)
    y=l1*math.sin(math.radians(q1dis[i]))+l2*math.sin(math.radians(q1dis[i]))
    b.append(y)
q2dis=np.linspace(0,q2,10)
for i in range(0,10):
    x=l1*math.cos(math.radians(q1))+l2*math.cos(math.radians(q2dis[i]))
    a.append(x)
    y=l1*math.sin(math.radians(q1))+l2*math.sin(math.radians(q2dis[i]))
    b.append(y)
plt.plot(a,b)
plt.show()