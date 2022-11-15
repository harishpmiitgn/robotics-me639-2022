# Intro To Robotics (Assignment-2)
# written by Kush Patel (20110131)

import numpy as np
import sympy as sp
from sympy import *

def get_EOM(n,D,V):
    phi = [0 for i in range(n)]
    for i in range(n):
        phi[i] = (sp.diff(V,q[i]))

    C = [[[0 for i in range(n)] for j in range(n)] for k in range(n)]
    for k in range(n):
        for j in range(n):
            for i in range(n):
                C[i][j][k] = 0.5 * (sp.diff(D[k][j],q[i]) + sp.diff(D[k][i],q[j]) - sp.diff(D[i][j],q[k]))

    tau = [0 for i in range(n)]
    a = 0
    c = 0
    for k in range(n):
        a = a + phi[k]
        for j in range(n):
            a = a + (D[k][j]*q_ddot[j])
            for i in range(n):
                c = c + (C[i][j][k]*q_dot[j]*q_dot[i])
        tau[k] = a + c
        print("Torque" + str(k+1) + " = " + str(tau[k]))
        print()
        a = 0
        c = 0

n=2
q1 = sp.symbols('q1')
q2 = sp.symbols('q2')
q1_dot = sp.symbols('q1_dot')
q2_dot = sp.symbols('q2_dot')
q1_ddot = sp.symbols('q1_ddot')
q2_ddot = sp.symbols('q2_ddot')
q = []
q.append(q1)
q.append(q2)
q_dot = []
q_dot.append(q1_dot)
q_dot.append(q2_dot)
q_ddot = []
q_ddot.append(q1_ddot)
q_ddot.append(q2_ddot)
m1 = sp.symbols('m1')
m2 = sp.symbols('m2')
l1 = sp.symbols('l1')
l2 = sp.symbols('l2')
g = sp.symbols('g')

D = np.array([[m2*(l1**2) + m1*(l1**2)/3 , 0.5*m2*l1*l2*sp.cos(q2-q1)],
              [0.5*m2*l1*l2*sp.cos(q2-q1) , m2*(l2**2)/3]])
V = 0.5*m1*g*l1*sp.sin(q1) + m2*g*l1*sp.sin(q1) + 0.5*m2*g*l2*sp.sin(q2)

get_EOM(n,D,V)
