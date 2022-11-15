# HPM's Intro To Robotics (Assignment-4&5)
# written by Kush Patel (20110131)
# Question - 11

import numpy as np
import math
import matplotlib.pyplot as plt 
from scipy.integrate import solve_ivp

def End_Effector_position_PUMA(q1,q2,q3):
    T = np.eye(4)
    DH_parameters = np.array([[q1,0,l1,1.57],[q2,l2,0,0],[q3,l3,0,0]])
    for i in range(3):
        A = np.array([[np.cos(DH_parameters[i,0]), -np.sin(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), np.sin(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.cos(DH_parameters[i,0])],
                      [np.sin(DH_parameters[i,0]), np.cos(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), -np.cos(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.sin(DH_parameters[i,0])],
                      [0,np.sin(DH_parameters[i,3]),np.cos(DH_parameters[i,3]),DH_parameters[i,2]],\
                      [0,0,0,1]])
        T = np.dot(T,A)
    return(T[0:3,3])

def IK_PUMA(x,y,z):
    r = (x**2 + y**2)**0.5
    s = z - l1
    theta1 = np.arctan2(y,x)
    d = (r**2 + s**2 - l2**2 - l3**2)/(2*l2*l3)
    theta3 = np.arctan2(((1-d**2)**0.5),d)
    theta2 = np.arctan2(s,r) - np.arctan2(l3*np.sin(theta3),l2 + l3*np.cos(theta3))
    return(theta1,theta2,theta3)

l1 = 1
l2 = 1
l3 = 1
g = 9.81
m1 = 1
m2 = 1
m3 = 1
r = 0.01
q1 = 0
q2 = 0
q3 = 0
q1_Initial = 0
q2_Initial = 0
q3_Initial = 0
w1 = 0
w2 = 0
w3 = 0
I1 = m3*r*r/2 
I2 = m2*l2*l2/12
I3 = m3*l3*l3/12


pi = np.array(list(map(float, input("Initial End-effector position:").split())))
pf = np.array(list(map(float, input("Final End-effector position:").split())))
# print(pi[0])
# print(pi[1])
# print(pi[2])
# print(pi)

qi = IK_PUMA(pi[0],pi[1],pi[2])
qf = IK_PUMA(pf[0],pf[0],pf[2])
# print(qi)
# print(qf)

def PUMAmanipulator(t, y):
    q1_I = y[0]
    q2_I = y[1]
    q3_I = y[2]
    q1 = y[3]
    q2 = y[4]
    q3 = y[5]
    omega1 = y[6]
    omega2 = y[7]
    omega3 = y[8]

    d11 = (m2*l2*l2*(np.cos(q2))**2)/4 + m3*(l2*np.cos(q2) + l3*np.sin(q2+q3)/2)**2 +I1 + I2 + I3
    d22 = (m2*l2**2)/4 + m3*l2**2 + (m3*l3**2)/4 + m3*l2*l3*np.cos(q3) + I2 + I3
    d23 = m3*(l3*l3/4 + l2*l3*np.cos(q3)/2) + I3
    d32 = m3*(l3*l3/4 + l2*l3*np.cos(q3)/2) + I3
    d33 = m3*l3*l3/4 + I3

    A = -m3*l3*omega1*omega3*(l3*np.cos(q2)+l3*np.cos(q2+q3))*np.sin(q2+q3) + omega1*omega2*(-l2*l2*m2*np.sin(q2)*np.cos(q2) + m3*(-2*l3*np.sin(q2) - l3*np.sin(q2+q3))*(l3*np.cos(q2) + l3*np.cos(q2+q3)/2)) 
    B = -l2*l3*m3*np.sin(q3)*omega2*omega3 - l2*l3*m3*omega3*omega3*np.sin(q3)/2 + omega1*omega1*(l2*l2*m2*np.sin(q2)/4 - m3*(-l3*np.sin(q2) - l3*np.sin(q2+q3)/2)*(l3*np.cos(q2)+l3*np.cos(q2+q3)/2)) - g*l2*m2*np.cos(q2) - g*m3*(l2*np.cos(q2)+l3*np.cos(q2+q3)/2)
    C = l2*l3*m3*np.sin(q3)*omega2*omega2 + l3*m3*omega1*omega1*(l3*np.cos(q2) + l3*np.cos(q2+q3)/2)*np.sin(q2+q3)/2 - g*l3*m3*np.cos(q2+q3)/2
    K2 = 100
    K1 = 0.1
    K = -100
    k = 10
    output = [q1,q2, q3, omega1, omega2, omega3, -A/(d11-I1) - K*(qf[0] - q1) - K2*omega1-K1*q1_I, (C*d23/(d33-I3)-B)/(d22 - I2 - d32/(d33-I3)) - K*(qf[1]-q2) - K1*(q2_I) -K2*omega2,(-C-d32*((C*d23/(d33-I3)-B)/(d22 - I2 - d32/(d33-I3))))/(d33-I3) -K*(qf[2]-q3) - k*(q3_I) -K2*omega3]
    
    return output

q1 = qi[0]
q2 = qi[1]
d3 = qi[2]
    
initial_state = [q1_Initial,q2_Initial,q3_Initial,q1,q2, d3, w1, w2, w3]
time = np.linspace(0, 10, 1000) 

solution = solve_ivp(PUMAmanipulator, [0, time[-1]], initial_state, t_eval= time) 


qs1 = solution.y[3]
qs2 = solution.y[4]
ds3 = solution.y[5]
omegas1 = solution.y[6]
omegas2 = solution.y[7]
ds3_dot = solution.y[8]
time = solution.t

Xt = []
Yt= []
Zt = []

for i in range(len(qs1)):
    P = End_Effector_position_PUMA(qs1[i],qs2[i],ds3[i])
    Xt.append(P[0])
    Yt.append(P[1])
    Zt.append(P[2])

plt.plot(time,Yt,'r',time,Xt,'g',time,Zt,'b')
plt.title('End-effector Position vs time')
plt.xlabel('Time')
plt.ylabel("position of end effector")
plt.legend(['y','x','z'])
plt.show()

# 2D Animation
plt.ion()
plt.show()

for i in range(len(qs1)):
    q1 = qs1[i]
    q2 = qs2[i]
    q3 = q3[i]
    link1 = (l2*np.cos(q1)*np.cos(q2),l2*np.sin(q1)*np.cos(q2))
    link2 = ((l2*np.cos(q2) + l3*np.cos(q2+q3))*np.cos(q1), (l2*np.cos(q2) + l2*np.cos(q2+q3))*np.sin(q1))
    plt.clf() 
    plt.xlim([-1, 1])
    plt.ylim([-1, 1])
    joint = (0, 0)
    plt.plot([joint[0], link1[0], link2[0]], [joint[1], link1[1], link2[1]], '-o')
    plt.pause(0.0001)

plt.ioff()
plt.show()