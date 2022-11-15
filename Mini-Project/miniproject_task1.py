# Intro To Robotics (Mini-Project)
# written by Kush Patel (20110131)


       ### MODIFIED CODE ###
import numpy as np
import math
import matplotlib.pyplot as plt

x = np.linspace(-7,6,50)
y = []
for i in range(len(x)):
  # y.append(3*math.sin(3*x[i]) - 6)
  y.append(3*math.sin(3*x[i]) + 6)
#   y.append(math.sqrt(81 - x[i]*x[i]))
  # y.append(0*x[i] + 6)
l1 = 6
l2 = 5
ti = 0
tf = 1
N = 1
t  = np.linspace(ti,tf,N)
Q =[[],[]]

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
      Q[0].append(Q1)
      Q[1].append(Q2)
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
        plt.plot(x,y,':')
        plt.plot([x0,x1], [y0,y1],'r')
        plt.plot([x1,x2], [y1,y2],'b')
        plt.xlim(-10, 12)
        plt.ylim(-10, 11) 
        plt.plot(x0,y0,'.',color='red')
        plt.plot(x1,y1,'.',color='red')
        plt.plot(x2,y2,'.',color="black")
        plt.grid()
        plt.pause(0.1)
        plt.clf()
    except:
      print("Trajectory is not feasible")
      pass

plt.show()



        ### OLD CODE ###
# import numpy as np
# import math
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation

# x = np.linspace(0,6,50)
# y = []
# for i in range(len(x)):
#   y.append(3*math.sin(5*x[i]) + 6)
#   # y.append(math.sqrt(81 - x[i]*x[i]))
#   # y.append(0*x[i] + 6)
# l1 = 6
# l2 = 5
# ti = 0
# tf = 1
# t  = np.linspace(ti,tf,1)
# Q =[[],[]]

# for i in range(len(t)):
#   for j in range(len(x)):
#     try:
#       theta = math.degrees(math.acos(((x[j]*x[j]) + (y[j]*y[j]) - (l1*l1) - (l2*l2))/(2*l1*l2)))
#       if x[j]==0:
#         x[j]=0.0000001
#       else:
#         pass
#       Q1 = math.degrees(abs(math.atan(y[j]/x[j]))) - math.degrees(math.atan((l2*math.sin(math.radians(theta)))/(l1 + (l2*math.cos(math.radians(theta))))))
#       Q2 = Q1 + theta
#       # print(Q1,Q2)
#       x0 = 0.0
#       y0 = 0.0
#       Q1 = math.radians(Q1)
#       Q2 = math.radians(Q2)
#       Q[0].append(Q1)
#       Q[1].append(Q2)
#       x1 = l1*math.cos(Q1)
#       y1 = l1*math.sin(Q1)

#       x2 = x1 + (l2*math.cos(Q2))
#       y2 = y1 + (l2*math.sin(Q2))
#       X = [x0,x1,x2]
#       Y = [y0,y1,y2]
#       # print(X)
#       # print(Y)
#       a = math.sqrt(x2*x2 + y2*y2)
#       # print(a)
#       if a>(l1+l2):
#         pass
#       else:
#         plt.plot(x,y,':')
#         plt.plot([x0,x1], [y0,y1],'r')
#         plt.plot([x1,x2], [y1,y2],'b')
#         plt.xlim(-6, 12)
#         plt.ylim(-7, 11) 
#         plt.plot(x0,y0,'.',color='red')
#         plt.plot(x1,y1,'.',color='red')
#         plt.plot(x2,y2,'.',color="green")
#         plt.grid()
#         plt.pause(0.1)
#         plt.clf()
#     except:
#       print("Trajectory is not feasible")
#       pass

# plt.show()
# # print(Q)

