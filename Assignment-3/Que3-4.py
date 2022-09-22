# Intro To Robotics (Assignment-2)
# written by Kush Patel (20110131)

import numpy as np
import math

def DH_para(n,M):
    T = np.zeros([n,4,4])
    A = np.identity(4)
    for i in range(n):
        theta = math.radians(M[i][3])
        alpha = math.radians(M[i][1])
        Rz = np.array([[np.cos(theta),-np.sin(theta),0,0],
                       [np.sin(theta),np.cos(theta),0,0],
                       [0,0,1,0],
                       [0,0,0,1]])

        Tz = np.array([[1,0,0,0],
                       [0,1,0,0 ],
                       [0,0,1,M[i][2]],
                       [0,0,0,1]])

        Tx = np.array([[1,0,0,M[i][0]],
                       [0,1,0,0 ],
                       [0,0,1,0],
                       [0,0,0,1]])

        Rx = np.array([[1,0,0,0],
                       [0,np.cos(alpha),-np.sin(alpha),0],
                       [0,np.sin(alpha),np.cos(alpha),0],
                       [0,0,0,1]])
                
        ans = np.matmul(Rz,np.matmul(Tz,np.matmul(Tx,Rx)))
        A = np.matmul(A,ans)
        T[i]= A
    np.set_printoptions(precision=3)
    return T              # T = [T10 , T20 , T30 , ... , Tn0]

def SkewSym(m):
    s = np.array([[0,-m[2],m[1]],
                  [m[2],0,-m[0]],
                  [-m[1],m[0],0]])
    return s

def Manipulator_Jacobian(n,T,type="kush"):
    if type == "kush":
        notype = True
    else:
        notype = False

    # For Z
    if notype == True:
        Z = np.zeros([n,4])
        Z[0][2] = 1
        for j in range(n-1):
            Z[j+1] = np.matmul(T[j],np.array([0,0,1,0]))
        Z = np.delete(Z,3,1)
    else:
        Z = np.zeros([n,4])
        Z[0][2] = 1
        for i in range(n):
            if type[i] == "P":
                Z[i] = [0,0,1,0]
            else:
                Z[i] = np.matmul(T[i],np.array([0,0,1,0]))
        Z = np.delete(Z,3,1)

    # For O
    O = np.zeros([n+1,4])
    for i in range(n):
        O[i+1] = np.matmul(T[i],np.array([0,0,0,1]))
    O = np.delete(O,3,1)

    # For J
    if notype == True:
        J = np.zeros([6,n])
        for k in range(n):
            Jv = np.matmul(SkewSym(Z[k]),O[n]-O[k])
            Jw = (Z[k])
            J[:,k] = np.hstack((Jv,Jw))
        np.set_printoptions(precision=3)
    else:
        J = np.zeros([6,n])
        for i in range(n):
            if type[i]=="P":
                Jv = Z[i]
                Jw = [0, 0, 0]
                J[:,i] = np.hstack((Jv,Jw))
            else:
                Jv = np.matmul(SkewSym(Z[i]),O[n]-O[i])
                Jw = (Z[i])
                J[:,i] = np.hstack((Jv,Jw))
        np.set_printoptions(precision=3)
    return O,Z,J

n = 3
# x = DH_para(n,[[0,-90,1,0],[0,-90,1,-90],[0,0,1,0]])        # (ax,alpha,dz,theta)
# x = DH_para(n,[[0,0,1,0],[1,0,0,-90],[1,0,0,0]])       
x = DH_para(n,[[1,0,0,0],[1,0,0,0],[0,-180,0,0]])     # For RRP  
# x = DH_para(n,[[1,0,0,0],[1,0,0,0],[1,0,0,0]])      
# x = DH_para(n,[[1,0,0,0],[1,0,0,0]])       
# x = DH_para(n,[[1,0,0,30],[1,180,0,30],[0,0,30,0]])
y = Manipulator_Jacobian(n,x,["R","R","P"])
print(x)
print(y)


### NOTE ###

# The above the code is satisfied the two common RRP configurations 
# of Stanford manipulator and SCARA manipulator.

# If the user will not give any input about the cofiguration then 
# the system will automatially take Revolute joints as default.

# The user can give an input by simply adding the array which contains its
# configuration in terms of R and P.