import sympy as sym
import numpy as np
from sympy import *
from numpy import pi, absolute, arccos, arctan, sign, sin, cos, sqrt
import matplotlib.pyplot as plt

q3 = pi/4

xw = x - l3*cos(q3)
yw = y - l3*sin(q3)
l1,l2,l3 = 1,1,1
r,x,y = 1.5,1.5,0
s1,s2= sym.symbols('s1,s2')
c1,c2 = sym.symbols('c1,c2')
eqn_1 = sym.Eq((xw-l1*c1)**2 + (yw-l1*s1)**2,l2**2)
eqn_2 = sym.Eq((xw-l2*c2)**2 + (yw-l2*s2)**2,l1**2)
eqn_3 = sym.Eq(c1**2+s1**2,1)
eqn_4 = sym.Eq(c2**2+s2**2,1)
result = sym.solve([eqn_1,eqn_2,eqn_3,eqn_4],s1,s2,c1,c2)
(s1,s2,c1,c2) = result[1]
print(result[0])
q1,q2 = float(asin(s1)),float(asin(s2))
theta = np.linspace(0,2*pi,100)
theta_k = np.append(theta[1:100],2*pi)
p_x,p_y = r*cos(theta), r*sin(theta)
pxk,pyk = r*cos(theta_k), r*sin(theta_k)
del_x,del_y = pxk-p_x,pyk-p_y
J = Matrix([[-l1*sin(q1), -l2*sin(q2), -l3*sin(q3)],[l1*cos(q1), l2*cos(q2), l3*cos(q3)]])
A = J*np.transpose(J)
qks = [Matrix([q1,q2])] #q3
for i in range (0,len(del_x)):
    #print(Matrix.multiply(A.inv(),Matrix([del_x[i],del_y[i]])))
    qk = qks[i] + Matrix.multiply(A.inv(),Matrix([del_x[i],del_y[i]]))
   
    qks.append(qk)  #print(qk)
    #print(l1*cos(float(qk[0]))+l2*cos(float(qk[1]))+l3*cos(float(q3)))
    plt.scatter(l1*cos(float(qk[0]))+l2*cos(float(qk[1])),l1*sin(float(qk[0]))+l2*sin(float(qk[1])))#+l3*cos(float(q3))+l3*sin(q3)
plt.show()