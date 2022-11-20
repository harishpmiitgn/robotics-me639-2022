from math import cos, sin, atan, acos, pi
import numpy as np
import matplotlib.pyplot as plt


r = np.linspace(-5, 5, 100)
t = np.linspace(0,2*pi,100)

xd = 2.5*np.cos(t)
yd = 2.5*np.sin(t)

Xd = [xd, yd]

x_dot = - 2.5*np.sin(t)
y_dot = 2.5*np.cos(t)
X_dot = [x_dot,y_dot]

X = [[0]*len(t),[0]*len(t)]


def psuedo_Jinv (q):
  dh = [[0, q[0], 1, 0],[0, q[1], 1, 0],[0, q[2], 1, 0]] 
  H = [0]*4 
  H[0] = np.identity(4)
  H0 = np.identity(4) 

  for i in range(3):
    d = dh[i][0]
    theta = dh[i][1]
    r = dh[i][2]
    alpha = dh[i][3]

    Z = [[cos(theta), -1*sin(theta), 0, 0],
         [sin(theta), cos(theta), 0, 0],
         [0, 0, 1, d],
         [0, 0, 0, 1]]
    
    X = [[1, 0, 0, r],
         [0, cos(alpha), -1*sin(alpha), 0],
         [0, sin(alpha), cos(alpha), 0],
         [0, 0, 0, 1]]
    
    H[i+1] = np.matmul(Z,X) 
    H0 = H0@H[i+1] 

  O_n0 = [H0[0][3],H0[1][3],H0[2][3]] 
  
  O = np.zeros(3) 
  Jv = np.zeros((2,3)) 
  h = np.identity(4) 
  
  for i in range(3):
    h = h@H[i]
    Z = [h[0][2], h[1][2], h[2][2]] 
    O_i0 = [h[0][3], h[1][3], h[2][3]] 

    O[0] = O_n0[0] - O_i0[0] 
    O[1] = O_n0[1] - O_i0[1]
    O[2] = O_n0[2] - O_i0[2]

    Zx_Oi = (Z[1]*O[2]) - (O[1]*Z[2]) 
    Zx_Oj = (O[0]*Z[2]) - (Z[0]*O[2])
    Zx_Ok = (Z[0]*O[1]) - (O[0]*Z[1])

    Zx = [Zx_Oi, Zx_Oj, Zx_Ok] 
    
    Jv[0][i] = Zx[0]
    Jv[1][i] = Zx[1]

  Jvt = np.transpose(Jv)
  Jvplus = Jvt@np.linalg.inv(Jv@Jvt)
  
  return Jvplus


x0 = float(input())
y0 = float(input())

X[0][0] = x0
X[1][0] = y0

x2 = x0 - 1
y2 = y0

q2_0 = acos((x2**2 + y2**2 - 2)/2)
q1_0 = atan(y2/x2) - atan(sin(q2_0)/(1+cos(q2_0)))
q3_0 = 2*pi - (q1_0 + q2_0)
q_0 = [q1_0,q2_0,q3_0]

q = [0]*len(t)
q[0] = q_0
kp = 4

for k in range(len(t)):
  X[0][k] = cos(q[k][0]+q[k][1]+q[k][2]) + cos(q[k][0]+q[k][1]) + cos(q[k][0])
  X[1][k] = sin(q[k][0]+q[k][1]+q[k][2]) + sin(q[k][0]+q[k][1]) + sin(q[k][0])
  
  error = [Xd[0][k] - X[0][k], Xd[1][k] - X[1][k]]
  x_dot = [X_dot[0][k],X_dot[1][k]]
  
  Jvp = psuedo_Jinv(q[k])
  
  term1 = Jvp@x_dot
  term2 = Jvp@error
  delq = term1 + kp*term2
  
  q[k+1] = q[k]+ delq

plt.plot(X[0],X[1], color = 'red',label = 'actual') #actual path 
plt.plot(xd,yd, color = 'blue', label = 'desired') #desired path
plt.xlim([-2,2])
plt.ylim([-2,2])
plt.show()