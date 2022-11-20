import sympy as sym
import numpy as np
from sympy import *
from numpy import pi, absolute, arccos, arctan, sign, sin, cos, sqrt
import matplotlib.pyplot as plt

l1,l2 = 0.25,0.25
m,mi = 0.8, 0.005
(Ax,Ay,Az),(Bx,By,Bz),(Cx,Cy,Cz),(Dx,Dy,Dz) = (0.45,0.075,0.1),(0.45,-0.075,0.1),(0.25,-0.075,0.1),(0.25,0.075,0.1)
q1 = np.linspace(0,2*pi,100)
q2 = np.linspace(0,2*pi,100)
pxs,pys =[],[]
for i in range(0,100):
    for j in range(0,100):
        px,py = l1*cos(q1[i])+l2*cos(q2[j]),l1*sin(q1[i])+l2*sin(q2[j])
        pxs.append(px)
        pys.append(py)
plt.scatter(pxs,pys)
plt.scatter([Ax,Bx,Cx,Dx],[Ay,By,Cy,Dy])
plt.show()