# Intro To Robotics (Mini-Project)
# written by Kush Patel (20110131)

### Without Dynamics ###

import numpy as np
import math
import matplotlib.pyplot as plt

l1 = 6
l2 = 5
ti = 0
tf = 1
N = 1
t  = np.linspace(ti,tf,N)
xd = 5
yd = 8
x = np.linspace(l1-l2,xd,50)
y = []
for i in range(len(x)):
    # y.append(yd + ((yd-0)/(xd-(l1-l2)))*(x[i] - xd))
    y.append(yd + (x[i]*x[i]-xd*xd)*((-yd)/((l1-l2)*(l1-l2)-xd*xd)))

slop = 145 # in degree w.r.t to (+x) ditection

xc = np.linspace(-10,10,100)
yc = math.tan(math.radians(slop))*xc + yd - (math.tan(math.radians(slop))*xd)
F = 10
if slop<= 90:
    Fx = -F*math.sin(math.radians(slop))
    Fy = F*math.cos(math.radians(slop))
else:
    Fx = -F*math.sin(math.radians(slop))
    Fy = -F*math.cos(math.radians(slop))


for i in range(len(t)):
  for j in range(len(x)):
    try:
      theta = math.degrees(math.acos(((x[j]*x[j]) + (y[j]*y[j]) - (l1*l1) - (l2*l2))/(2*l1*l2)))
      if x[j]==0:
        x[j]=0.0000001
      else:
        pass

      Q1 = math.degrees((math.atan(y[j]/x[j]))) - math.degrees(math.atan((l2*math.sin(math.radians(theta)))/(l1 + (l2*math.cos(math.radians(theta))))))
      Q2 = Q1 + theta
      if (x[j]>=0 and y[j]>=0):
        Q1
        Q2
      elif (x[j]<=0 and y[j]>=0):
        Q1 = 180 + Q1
        Q2 = 180 + Q2
      elif (x[j]<=0 and y[j]<=0):
        Q1 = -180 + Q1
        Q2 = -180 + Q2
      else:
        Q1
        Q2      

      x0 = 0.0
      y0 = 0.0
      Q1 = math.radians(Q1)
      Q2 = math.radians(Q2)

      x1 = l1*math.cos(Q1)
      y1 = l1*math.sin(Q1)

      x2 = x1 + (l2*math.cos(Q2))
      y2 = y1 + (l2*math.sin(Q2))
      X = [x0,x1,x2]
      Y = [y0,y1,y2]
      a = math.sqrt(x2*x2 + y2*y2)
      if a>(l1+l2):
        pass
      else:
        plt.plot(xc, yc, color='black', label='Wall')
        plt.plot([x0,x1], [y0,y1],'r')
        plt.plot([x1,x2], [y1,y2],'b')
        plt.plot(xd,yd,'*',color='red')
        plt.plot(x0,y0,'.',color='red')
        plt.plot(x1,y1,'.',color='red')
        plt.plot(x2,y2,'.',color="green")
        plt.xlim(-10, 12)
        plt.ylim(-8, 11) 
        plt.legend(loc='upper left')
        plt.grid()
        plt.pause(0.01)
        plt.clf()
    except:
      print("Math Error")
      pass
tau1 = -Fx*l1*math.sin(Q1) + Fy*l1*math.cos(Q1)
tau2 = -Fx*l2*math.sin(Q2) + Fy*l2*math.cos(Q2)
print("torque-1 is =", tau1,"Nm")
print("torque-2 is =", tau2,"Nm")
plt.show()


