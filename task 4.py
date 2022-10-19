
import math
import lmfit
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
x=5
y=3
l1=4
l2=3
x0,y0 = 0,0
q1=35
q2=35
x1=l1*math.cos(math.radians(q1))
x2=x1+l2*math.cos(math.radians(q2))
y1=l1*math.sin(math.radians(q1))
y2=y1+l2*math.sin(math.radians(q2))
xout=l1*math.cos(math.radians(q1))+l2*math.cos(math.radians(q2))
yout=l1*math.sin(math.radians(q1))+l2*math.sin(math.radians(q2))
a=[]
b=[]
q1dis=np.linspace(35, 135,101)
for i in range(0,100):
    x=l1*math.cos(math.radians(q1dis[i]))+l2*math.cos(math.radians(q1dis[i]))
    a.append(x)
    y=l1*math.sin(math.radians(q1dis[i]))+l2*math.sin(math.radians(q1dis[i]))
    b.append(y)
xn=a[np.size(a)-1]
yn=b[np.size(a)-1]
plt.figure()
plt.plot([x0,x1],[y0,y1],linewidth=2)
plt.plot([x1,x2],[y1,y2],linewidth=2)
plt.plot(a,b,linewidth=2)
plt.plot(x0,y0,"ko",linewidth=4)
plt.plot([x0,xn],[y0,yn],linewidth=2)
#plt.plot([x0,a[np.size(a)-1]],[y0,np.size(b)-1],line=2)
plt.plot(x1,y1,"ko",linewidth=4)
plt.plot(x2,y2,"r*",linewidth=4)
