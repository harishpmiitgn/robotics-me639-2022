from cmath import cos, sin
import numpy
def SCARA(l1,l2,l3,d,theta_1,theta_2):
    theta_1=theta_1*numpy.pi/180
    theta_2=theta_2*numpy.pi/180
    p_3=numpy.array([[0],[0],[-d],[1]])
    H_0_1=numpy.array([[cos(theta_1),-sin(theta_1),0,0],[sin(theta_1),cos(theta_1),0,0],[0,0,1,l1],[0,0,0,1]])
    H_1_2=numpy.array([[cos(theta_2),-sin(theta_2),0,l2],[sin(theta_2),cos(theta_2),0,0],[0,0,1,0],[0,0,0,1]])
    H_2_3=numpy.array([[1,0,0,l3],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    p_0=H_0_1.dot(H_1_2.dot(H_2_3.dot(p_3)))
    P0=[[p_0[0][0].real],[p_0[1][0].real],[p_0[2][0].real]]
    return P0

#Enter the arguements below (Enter the angles theta_1 and theta_2 in degrees)
l1=1
l2=2
l3=3
d=0.5
theta_1=60
theta_2=45
print(SCARA(l1,l2,l3,d,theta_1,theta_2))