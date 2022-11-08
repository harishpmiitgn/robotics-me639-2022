#DH parameters and endeffector properties for a STANFORD Type manipultor

import numpy as np
n =  4  #number of links

#defing Cos and Sin as functions for simplicity.
def c(t):
    c= np.cos(t*np.pi/180)
    return c

def s(t):
    s= np.sin(t*np.pi/180)
    return s

# DH parameters for SCARA manipulator
DH = np.array([[10, 0, 0 , 0],
               [10, 180, 0, 0],
               [0, 0, 0, 0,],
               [0, 0, 5, 0]])

#Initializing the T matrix
T = np.array([[1,0,0,0],
              [0,1,0,0],
              [0,0,1,0],
              [0,0,0,1]])

B = np.array([[1],[2],[3],[1]]) #given end effector position
W = np.array([[5],[6],[4],[1]])
# making the T matrix and matrix multiplying all the H matrices.
for i in range(0,4):
    H = np.array([[c(DH[i,3]), -s(DH[i,3])*c(DH[i,1]), s(DH[i,3])*s(DH[i,1]), DH[i,0]*c(DH[i,3])],
                  [s(DH[i,3]) , c(DH[i,3])*c(DH[i,1]), c(DH[i,3])*s(DH[i,1]), DH[i,0]*s(DH[i,3])],
                  [0, s(DH[i,1]), c(DH[i,1]), DH[i,1]],
                  [0, 0, 0, 1]])
    T = np.dot(T,H)
    
# T is the Jacobian manipulator.

print('Manipulator Jacobian is')
print(T)
E = np.dot(T, B)
print('EndEffector position is ', E[0],E[1],E[2] )
#E gives the end effector postion with respect to base frame.

V = np.dot(T,W)
print('EndEffector velocity is ', V[0],V[1],V[2] )
# V gives the end effector velocity with respect to base frame