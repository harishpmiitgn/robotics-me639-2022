import math 
import numpy as np
import sympy as sp

m1 = sp.symbols('m1')
m2 = sp.symbols('m2')
lc1 = sp.symbols('lc1')
lc2 = sp.symbols('lc2')
l1 = sp.symbols('l1')
l2 = sp.symbols('l2')
p1 = sp.symbols('p1')
p2 = sp.symbols('p2')

g = sp.symbols('g')
m1 = sp.symbols('m1')
p1_dot = sp.symbols('p1_dot')
p2_dot = sp.symbols('p2_dot')
p1_double_dot = sp.symbols('p1_double_dot')
p2_double_dot = sp.symbols('p2_double_dot')
I1 = sp.symbols('I1')
I2 = sp.symbols('I2')

D_p = np.array([[m1*lc1**2 + m2*l1**2 + I1, m2*l1*lc2*sp.cos(p2-p1)],[m2*l1*lc2*sp.cos(p2-p1), m2*lc2**2 + I2]])
p = np.array([[p1],[p2]])
p_dot = np.array([[p1_dot],[p2_dot]])
p_double_dot = np.array([[p1_double_dot],[p2_double_dot]])
V_p = m1*g*lc1*sp.sin(p1) + m2*g*(l1*sp.sin(p1) + lc2*sp.sin(p2))

def C(D_q,q):
    N  = len(q)
    C_mat = np.array([[[None]*N]*N]*N)
    for i in range(len(D_q)):
        for j in range(len(D_q[i])):
            for k in range(N):
                dkj = D_q[k][j]
                dki = D_q[k][i]
                dij = D_q[i][j]
                qi = q[i][0]
                qj = q[j][0]
                qk = q[k][0]
                C_mat[i][j][k] = (0.5)*((dkj.diff(qi)) + dki.diff(qj) - dij.diff(qk))
    return C_mat

def phi(V_q,q):
    N = len(q)
    phi_mat = np.array([None]*N)
    for k in range(N):
        qk = q[k][0]
        phi_mat[k] = V_q.diff(qk)

    return phi_mat 


def Tau(D_q,C_q_qdot,phi_q,q,q_dot,q_double_dot):
    N = len(q)
    Tau_mat = np.array([None]*N)
    
    for k in range(N):
        Tau_k = 0
        for j in range(N):
            Tau_k += q_double_dot[j][0]*D_q[k][j]
        for i in range(N):
            for j in range(N):
                Tau_k += q_dot[i][0]*q_dot[j][0]*C_q_qdot[i][j][k]
        Tau_k += phi_q[k]
        Tau_mat[k] = Tau_k
    
    return Tau_mat

C_p = C(D_p,p)
phi_p = phi(V_p,p)
Tau_p = Tau(D_p,C_p,phi_p,p,p_dot,p_double_dot)

for i in range(len(Tau_p)):
    print('Tau'+ str(i+1) + ' = ')
    print(Tau_p[i])
    print()