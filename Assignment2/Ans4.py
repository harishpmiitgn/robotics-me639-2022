import math
from operator import matmul
import numpy as np

#Defining rotation matrix calculation function
def R(axis, angle): # Input axis as 'x', 'y' or 'z' strings for x, y or z axis respectively. Input angle in degrees
    angle = math.radians(angle)
    if axis == 'x':
        mat = np.array([[1,0,0],[0,math.cos(angle),-math.sin(angle)],[0,math.sin(angle),math.cos(angle)]])
    elif axis == 'y':
        mat = np.array([[math.cos(angle),0,math.sin(angle)],[0,1,0],[-math.sin(angle),0,math.cos(angle)]])
    elif axis == 'z':
        mat = np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
    return mat

#Defining H matrix calculation function
def H(R,d_):
    matH = np.zeros((4,4))
    for i in range(3):
        for j in range(3):
            matH[i][j] = R[i][j]
    for i in range(3):
        matH[i][3] = d_[i]
    matH[3][3] = 1
    return matH

#defining the length of the first 2 arms of the robot.

a1 = 4
a2 = 5

# taking input for joint space (in this case 2 angles and 1 displacement)
print('Input q1:') 
q1 = -float(input())
print('Input q2:')
q2 = -float(input())
print('Input d3:') 
d3 = float(input())

# Calculating R matrices for Standford Robot 
R01 = R('z',q1)
R12 = R('y',q2)
R23 = R('z',0)

# Calculating d vectors for Standford Robot
d01 = np.array([0,0,0])
d12 = np.array([0,0,a1])
d23 = np.array([0,a2,0])

# Calculating the Coordinate of the end effector with respect to the last coordinate frame 
P3 = np.array([0,0,-d3])
P3_ = np.array([P3[0],P3[1],P3[2],1])

# Calculating H matrices for SCARA
H01 = H(R01,d01)
H12 = H(R12,d12)
H23 = H(R23,d23)

# Calculating P0 Vector
P0_ = matmul(matmul(H01,H12),matmul(H23,P3_))
P0 = P0_[:3]
#print the P0 vector
print(P0) 