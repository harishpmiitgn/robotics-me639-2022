import numpy as np
import sympy as sp

def Dynamics(D):
    n = sp.shape(D)[0]
    tau = sp.symarray('tau',n)
    q = sp.symarray('q',n)
    qdot = sp.symarray('qdot',n)
    qddot = sp.symarray('qddot',n)
    Cijk =  [[[0 for x in range(n)] for x in range(n)] for x in range(n)]

    # Cijk = [[[0]*n]*n]*n
    # C = [[0]*n]*n
    for k in range(n):
        for j in range(n):
            for i in range(n):
                #Cijk[i][j][k] = 0.5*(sp.diff(D[k][i],q[j])+sp.diff(D[k][j],q[i])-sp.diff(D[j,j],q[k]))
                Cijk[i][j][k] = 0.5*(sp.diff(D[k][j],q[i])+sp.diff(D[k][i],q[j])-sp.diff(D[i][j],q[k]))
    # print("Cijk")
    # print(Cijk)
    Inertia = [0 for x in range(n)]
    for k in range(n):
        for j in range(n):
            Inertia[k] += D[k][j]*qddot[j]
    # print("Inertia")
    # print(Inertia[0])
    Coriolis = [0 for x in range(n)]
    Eq = [0 for x in range(n)]
    phi = [0 for x in range(n)]


    for k in range(n):
        for j in range(n):
            for i in range(n):
                #C[k][j] = C[k][j] + Cijk[k][j][i]*qdot[i]
                Coriolis[k] += Cijk[i][j][k]*qdot[i]*qdot[j]
    # print("Coriolis")
    # print(Coriolis)
    Equation = []
    for k in range(n):
        phi[k] = sp.diff(V,q[k])
        Eq[k] = Inertia[k]+Coriolis[k]+phi[k]-tau[k]
        Equation.append(sp.Eq(Eq[k]))
    
    return Equation

n = 2
m = sp.symarray('m',n)
l = sp.symarray('l',n)
lc = sp.symarray('lc',n)
I = sp.symarray('I',n)
g = sp.symbols('g')


q = sp.symarray('q',n)
qdot = sp.symarray('qdot',n)
qddot = sp.symarray('qddot',n)
D11 = m[0]*(lc[0])**2+m[1]*(l[0]**2+lc[1]**2+2*l[0]*lc[1]*sp.cos(q[1]))+I[0]+I[1]
D12 = m[1]*(lc[1]**2+l[0]*lc[1]*sp.cos(q[1])+I[1])
D21 = m[1]*(lc[1]**2+l[0]*lc[1]*sp.cos(q[1])+I[1])
D22 = m[1]*(lc[1]**2)+I[1]
D = np.array([[D11,D12],[D21,D22]])

V = (m[0]*lc[0]+m[1]*l[0])*g*sp.sin(q[0])+m[1]*lc[1]*g*sp.sin(q[0]+q[1])

Equation = Dynamics(D)
i = 0
for e in Equation:
    print(i)
    print(e)
    i = i+1
