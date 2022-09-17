import numpy
import sympy
n=2
for i in range(n):
    globals()['q%s' % str(i+1)] = sympy.Symbol('q'+str(i+1))
    globals()['q_dot%s' % str(i+1)] = sympy.Symbol('q_dot'+str(i+1))
    globals()['q_dot_dot%s' % str(i+1)] = sympy.Symbol('q_dot_dot'+str(i+1))
m1,m2,l1,l2,g=[sympy.Symbol('m1'),sympy.Symbol('m2'),sympy.Symbol('l1'),sympy.Symbol('l2'),sympy.Symbol('g')]
D=numpy.array([[m1*l1**2/3+m2*l1**2, m2*l1*l2*sympy.cos(q2-q1)/2],[m2*l1*l2*sympy.cos(q2-q1)/2, m2*l2**2/3]])
V=m1*g*l1/2*sympy.sin(q1)+m2*g*(l1*sympy.sin(q1)+l2/2*sympy.sin(q2))
phi=numpy.array([[sympy.diff(V, q1)],[sympy.diff(V, q2)]])
q=numpy.array([[]])
for i in range(n):
    q=numpy.append(q,[[globals()['q%s' % str(i+1)]]],axis=1)
q=numpy.transpose(q)
q_dot=numpy.array([[]])
for i in range(n):
    q_dot=numpy.append(q_dot,[[globals()['q_dot%s' % str(i+1)]]],axis=1)
q_dot=numpy.transpose(q_dot)
q_dot_dot=numpy.array([[]])
for i in range(n):
    q_dot_dot=numpy.append(q_dot_dot,[[globals()['q_dot_dot%s' % str(i+1)]]],axis=1)
q_dot_dot=numpy.transpose(q_dot_dot)
C=[0]*n
for k in range(n):
    for i in range(n):
        for j in range(n):
            C[k]+=0.5*(sympy.diff(D[k][j], "q"+str(i+1))+sympy.diff(D[k][i], "q"+str(j+1))-sympy.diff(D[i][j], "q"+str(k+1)))*sympy.Symbol("q_dot"+str(i+1))*sympy.Symbol("q_dot"+str(j+1))
EL=D@q_dot_dot+phi+numpy.transpose([C])
for i in range(len(EL)):
    print(EL[i][0],end="")
    print(" = tau_"+str(i+1))