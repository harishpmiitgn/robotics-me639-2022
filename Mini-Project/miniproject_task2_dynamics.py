# Intro To Robotics (Mini-Project)
# written by Kush Patel (20110131)


import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def get_Qs(x,y):
    Q = [[],[]]
    for j in range(len(x)):
        try:
            theta = math.degrees(math.acos(((x[j]*x[j]) + (y[j]*y[j]) - (l1*l1) - (l2*l2))/(2*l1*l2)))
            if x[j]==0:
                x[j]=0.000001
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
            Q1 = math.radians(Q1)
            Q2 = math.radians(Q2)
            Q[0].append(Q1)
            Q[1].append(Q2)
        except:
            print("Trajectory is not feasible")
            pass
    return Q

def get_Qdots(Q):
    Qdot = [[],[]]
    for i in range(n-1):
        Qdot[0].append((Q[0][i+1]-Q[0][i])/dt)
        Qdot[1].append((Q[1][i+1]-Q[1][i])/dt)
    Qdot[0].append(-2.8)
    Qdot[1].append(-6)
    return Qdot

def get_Qdoubledot(Qdot):
    Qddot = [[],[]]
    for i in range(n-1):
        Qddot[0].append((Qdot[0][i+1]-Qdot[0][i])/dt)
        Qddot[1].append((Qdot[1][i+1]-Qdot[1][i])/dt)
    Qddot[0].append(-1)
    Qddot[1].append(-1)
    return Qddot

# q = [q1, q2, q1_dot, q2_dot, q1_doubledot, q2_doubledot]
def get_tau(q):
    tau1 = q[4]*(m1*l1**2/3 + m2*l1**2) + q[5]*(0.5*m2*l1*l2*math.cos(q[0]-q[1])) + 0.5*q[3]**2*m2*l1*l2*math.sin(q[0]-q[1]) + 0.5*m1*l1*g*math.cos(q[0]) + m2*g*l1*math.cos(q[0])
    tau2 = q[5]*(m2*l2**2/3) + q[4]*(0.5*m2*l1*l2*math.cos(q[0]-q[1])) - 0.5*q[2]**2*m2*l1*l2*math.sin(q[0]-q[1]) + 0.5*m2*g*l2*math.cos(q[1])
    return [tau1, tau2]


# ans = [q1, q2, q1_dot, q2_dot]
def model(ans,t,tau1,tau2):
    tau = [tau1,tau2]
    p = (1/(a*f - b*h*np.cos(ans[0]-ans[1])**2))*(f*tau[0] - c*f*ans[3]**2*np.sin(ans[0]-ans[1]) - f*d*np.cos(ans[0]) - f*e*np.cos(ans[0]) - b*tau[1]*np.cos(ans[0]-ans[1]) - b*j*ans[2]**2*np.sin(ans[0]-ans[1])*np.cos(ans[0]-ans[1]) + b*k*np.cos(ans[0]-ans[1])*np.cos(ans[1]))
    q = (1/f)*(tau[1] - p*h*np.cos(ans[0]-ans[1]) + ans[2]**2*j*np.sin(ans[0]-ans[1]) - k*np.cos(ans[1]))
    return [ans[2], ans[3], p, q]

l1 = 6
l2 = 5
ti = 0
tf = 1
N = 1
t  = np.linspace(ti,tf,N)
dt = (tf-ti)/N
xd = 5
yd = 8
n = 10
x = np.linspace(l1-l2,xd,n)
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

m1 = 1
m2 = 1
g = 9.81

a = (m1*l1**2/3 + m2*l1**2)
b = (0.5*m2*l1*l2)
c = 0.5*m2*l1*l2
d = 0.5*m1*l1*g
e = m2*g*l1
f = m2*l2**2/3
h = 0.5*m2*l1*l2
j = 0.5*m2*l1*l2
k = 0.5*m2*g*l2

Q = get_Qs(x,y)
Qdot = get_Qdots(Q)
Qddot = get_Qdots(Qdot)
# Qdot = [[(Q[0][1]-Q[0][0])/dt, -2.8], [(Q[1][1]-Q[1][0])/dt,-6]]
# Qddot = [[(Qdot[0][1]-Qdot[0][0])/dt, 0],[(Qdot[1][1]-Qdot[1][0])/dt,-1]]

z = [Q[0][0], Q[1][0], 0, 0] # initial input = [q1, q2, q1_dot, q2_dot]
Qr = [[Q[0][0]],[Q[1][0]]]

for j in range(n-1):
    v = [Q[0][j], Q[1][j], Qdot[0][j], Qdot[1][j], Qddot[0][j], Qddot[1][j]]
    torque = get_tau(v)
    w = odeint(model,z,[0,dt], args= (torque[0],torque[1]))
    # print(w)
    z = w[1]
    Qr[0].append(z[0])
    Qr[1].append(z[1])


for i in range(n):
  try:
      x0 = 0.0
      y0 = 0.0

      Q1 = Qr[0][i]
      Q2 = Qr[1][i]

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
          plt.xlim(-10, 12)
          plt.ylim(-9, 11) 
          plt.plot(x0,y0,'.',color='red')
          plt.plot(x1,y1,'.',color='red')
          plt.plot(x2,y2,'.',color="green")
          plt.legend(loc='upper left')
          plt.grid()
          plt.pause(0.1)
          plt.clf()
  except:
      print("Trajectory is not Feasible")
      pass
plt.show()
tau1 = -Fx*l1*math.sin(Q1) + Fy*l1*math.cos(Q1)
tau2 = -Fx*l2*math.sin(Q2) + Fy*l2*math.cos(Q2)
print("torque-1 is =", tau1,"Nm")
print("torque-2 is =", tau2,"Nm")