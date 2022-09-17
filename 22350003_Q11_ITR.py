import numpy as np
import math
import matplotlib.pyplot as plt
from sympy import *
D, d11, d12, d21, d22, V =symbols('D d11 d12 d21 d22 V')

Q(i)=qi
Q(k)=qk
Q(j)=qj

#For two link manipulaator
D=np.array([[d11, d12], [d21, d22]])
n_start=1
n_end=n #n=2

#program to print Cijk
for k in range(n_start-1,n_end-1,1):
    for i in range(n_start-1,n_end-1,1):
        for j in range(n_start-1,n_end-1,1):
            Cijk=0.5(diff(D(k,j)),q(i)+diff(D(k,i)),q(j)-diff(D(i,j)),q(k))
            print('C',i,j,k,'=',Cijk)
        end
    end
end

#program to print phi(k)
for k in range(n_start-1,n_end-1,1):
    dV=diff(V,q(k))
    print('Q(k)')
end

#Equation of motion
print(D*,'d2q',+Cijk*,'dq',+dV,'=0')