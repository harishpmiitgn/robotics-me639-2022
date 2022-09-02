from cmath import cos, sin
import numpy
def SCARA(l1,l2,l3,d,theta_1,theta_2):
    theta_1=theta_1*numpy.pi/180
    theta_2=theta_2*numpy.pi/180
    H_0_1=numpy.array([[cos(theta_1),-sin(theta_1),0,0],[sin(theta_1),cos(theta_1),0,0],[0,0,1,l1],[0,0,0,1]])
    H_1_2=numpy.array([[cos(theta_2),-sin(theta_2),0,l2],[sin(theta_2),cos(theta_2),0,0],[0,0,1,0],[0,0,0,1]])
    H_2_3=numpy.array([[1,0,0,l3],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    P1=numpy.array([[l2],[0],[0],[1]])
    P2=numpy.array([[l3],[0],[0],[1]])
    P3=numpy.array([[0],[0],[-d],[1]])
    O0=numpy.array([[0],[0],[0]])
    O1=(H_0_1.dot(P1))[:-1]
    O2=(H_0_1.dot(H_1_2.dot(P2)))[:-1]
    O3=(H_0_1.dot(H_1_2.dot(H_2_3.dot(P3))))[:-1]
    z0=numpy.array([[0],[0],[1]])
    z1=numpy.array([[numpy.cos(theta_1), -numpy.sin(theta_1), 0],[numpy.sin(theta_1), numpy.cos(theta_1), 0],[0, 0, 1]]).dot(z0)
    z2=numpy.array([[numpy.cos(theta_2), -numpy.sin(theta_2), 0],[numpy.sin(theta_2), numpy.cos(theta_2), 0],[0, 0, 1]]).dot(z1)
    j=numpy.concatenate((numpy.concatenate((numpy.cross(z0.T,(O3-O0).T).T,numpy.cross(z1.T,(O3-O1).T).T,numpy.cross(z2.T,(O3-O2).T).T),axis=1),numpy.concatenate((z0,z1,z2),axis=1)),axis=0)
    return j.real


#Enter the arguements below (Enter the angles theta_1 and theta_2 in degrees)
l1=1
l2=2
l3=3
d=0.5
theta_1=60
theta_2=45
print(SCARA(l1,l2,l3,d,theta_1,theta_2))