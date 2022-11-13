import roboticstoolbox as rtb
from spatialmath import SE3
import cv2
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt
import spatialmath.base as base
import numpy as np
from scipy.integrate import odeint
from ITRfunctions import *
from integratorSCARA import *
#####################################################################################################################
## DEFINE DH PARAMETERS

# link1
alpha1 = 0
d1 = 0.2
a1 = 0.5
# link2
alpha2 = np.deg2rad(180)
d2 = 0
a2 = 0.5
# link3
alpha3 = np.deg2rad(0)
theta3 = 0
a3 = 0
d3 = 0
DH = [[0, alpha1, d1, a1], [0, alpha2, d2, a2], [theta3, alpha3, 0, a3]]
# plot scara
ax = plotrobotscara(DH) # to see plot need to do plt.show()

## OBTAIN JOINT TRAJECTORY USING END EFFECTOR TRAJECTORY
q0 = scara_inverse_kinematics(DH,0.4,0.01,0.1)
q1 = scara_inverse_kinematics(DH,0.4,0.06,0.1)
q1_poly = cubic(q0[0], q1[0], 0, 0, 0, 1)
q2_poly = cubic(q0[1], q1[1], 0, 0, 0, 1)
q3_poly = cubic(q0[2], q1[2], 0, 0, 0, 1)

# time from (t0,tfinal,no)
t=np.linspace(0,1,100)

q = np.transpose(np.zeros(6))
i = 0
alpha1 = 0
d1 = 0.2
a1 = 0.5
# link2
alpha2 = np.deg2rad(180)
d2 = 0
a2 = 0.5
# link3
alpha3 = np.deg2rad(0)
theta3 = 0
a3 = 0
d3 = 0
DH = [[0, alpha1, d1, a1], [0, alpha2, d2, a2], [theta3, alpha3, 0, a3]]  # sequence [theta,alpha,d,a]
# initial configuration
Xi = 0.4
Yi = 0.01
Zi = 0.1
qi = scara_inverse_kinematics(DH,Xi,Yi,Zi)
#start state for integrator [q,dq] last three values indicate joint rate
q0 = [qi[0],qi[1],qi[2],0,0,0]
m1=1 ; m2=1 ; l1=1;l2=1; l3=1; I1=1/12; I2=1/12; I3=1/12; lc1=l1/2; lc2=l2/2; lc3=l3/2; g=9.81
# Jm1 =0.4*10**-4; Jm2 =0.4*10**-4; Jm3=0.4*10**(-4); Bm1 = 4*10**(-5); Bm2 = 4*10**-5; Bm3=4*10**-5; Km1 =2*10**-2; Km2 =2*10**-2;Km3 =2*10**-2
# Kb1 = 0.02;Kb2 = 0.02;Kb3=0.02;R1 = 0.3;R2 = 0.3;R3 = 0.3;r1 = 100;r2 = 100; r3=100; zeta = 1; wn = 50; 
K1 = 1; K2 = 1; K3 = 1

i = 0
sum = np.array([[0],[0],[0]])
### integrate the robot dynamics and control
q = odeint(modelscara,q0,t,args=(i, I1, I2, l1, l2, l3, lc1, lc2, lc3, m1, m2, K1, K2, K3,q1_poly,q2_poly,q3_poly))

xe = 0.4 * np.ones(100) # end-effector path only for plotting purpose
ye = np.linspace(0.01, 0.06, 100)
ze = 0.1 * np.ones(100)
plt.figure(1)
tt = np.linspace(0,1,100)

# generate actual vs desired plots here

# Animate in 3D
for i in range(100):
    # substitute joint variables obtained from integration
    DH = [[q[i][0], alpha1, d1, a1], [q[i][1], alpha2, d2, a2], [theta3, alpha3, q[i][2], a3]] # for scara
    ax = plotrobotscara(DH)
    ax.plot3D(xe, ye, ze, '-m', label=r'$desired end-effector trajectory$')
    plt.draw()
    ax.text(0.5, 0.5, 0, 'step no ='+str(i), color='red')
    plt.pause(0.001)

