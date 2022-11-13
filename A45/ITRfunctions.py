import roboticstoolbox as rtb
from spatialmath import SE3
import cv2
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt
import spatialmath.base as base
import numpy as np

def scara_inverse_kinematics(DH,xc,yc,zc):
   d1=DH[0][2]
   a1 = DH[0][3]
   a2 = DH[1][3]
   D = (xc**2+yc**2-a1**2-a2**2 )/(2*a1*a2)
   theta2 = np.arctan2(np.sqrt(1-D**2),D)
   theta1 = np.arctan2(yc,xc)-np.arctan2(a2*np.sin(theta2),(a1+a2*np.cos(theta2)))
   d3 = d1-zc
   return theta1,theta2,d3

def homogenous_matrix(DH1): # from textbook
    ct = np.cos(DH1[0])
    ca = np.cos(DH1[1])
    st = np.sin(DH1[0])
    sa = np.sin(DH1[1])
    a = DH1[1]
    d = DH1[2]
    r = DH1[3]
    homogenous_matrix =np.array([[ct, -st*ca,  st*sa,   r*ct],
                                [st,   ct*ca,  -ct*sa,  r*st],
                                [0,    sa,     ca,         d],
                                [0,    0,      0,          1]])
    return homogenous_matrix

def forward_kinematics(DH): # returns all link endpoints
    A = np.identity(4)
    n = np.shape(DH)[0]
    ox = [0]
    oy = [0]
    oz = [0]
    for i in range(n):
        Anext = homogenous_matrix(DH[i])
        A = np.matmul(A,Anext)

        ox.append(A[0][3])
        oy.append(A[1][3])
        oz.append(A[2][3])
    return ox,oy,oz

def forward_kinematics2(DH): # returns only endeffector
    A = np.identity(4)
    n = np.shape(DH)[0]
    ox = [0]
    oy = [0]
    oz = [0]
    for i in range(n):
        Anext = homogenous_matrix(DH[i])
        A = np.matmul(A,Anext)

    ox = (A[0][3])
    oy = (A[1][3])
    oz = (A[2][3])
    return ox,oy,oz

def plotrobotscara(DH): # plot in 3D 3 DOF robot from DH param. base link added

    ox,oy,oz = forward_kinematics(DH)

    link0x = [0,0]
    link0y = [0,0]
    link0z = [0,DH[0][2]]

    link1x = [ox[0],ox[1]]
    link1y = [oy[0],oy[1]]
    link1z = [oz[1],oz[1]]

    link2x = [ox[1],ox[2]]
    link2y = [oy[1],oy[2]]
    link2z = [oz[1],oz[2]]

    link3x = [ox[2],ox[3]]
    link3y = [oy[2],oy[3]]
    link3z = [oz[2],oz[3]]

    ax = plt.axes(projection='3d')

    ax.set_zlabel('z')
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    elev = 45
    azim = 45
    ax.view_init(elev, azim)
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.5)

    #print('verification of endpoint by FK', ox[3], oy[3], oz[3])

    v = ax.plot3D(link0x, link0y, link0z, '-g',label='base', linewidth=2) #base link
    a = ax.plot3D(link1x, link1y, link1z, '-r',label='link1', linewidth=2)
    b = ax.plot3D(link2x, link2y, link2z, '-b',label='link2', linewidth=2)
    c = ax.plot3D(link3x, link3y, link3z, '-k',label='link3', linewidth=2)

    return ax



def cubic(x0, xf, x0dot, xfdot, t0, tf): # gives cubic polynomial coeff for single joint

    T = np.array([[1, t0, t0**2, t0**3],[0, 1, 2 * t0, 3 * t0**2],[1, tf, tf**2, tf**3],[0, 1, 2 * tf, 3 * tf**2]])
    x = np.array([[x0],[x0dot],[xf],[xfdot]])
    a = np.matmul(np.linalg.inv(T),x)
    return a

