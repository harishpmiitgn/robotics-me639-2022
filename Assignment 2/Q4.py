from cmath import cos, sin
import numpy
def Stanford(l1,l2,d,theta_1,theta_2):
    theta_1=theta_1*numpy.pi/180
    theta_2=theta_2*numpy.pi/180
    p_3=numpy.array([[0],[-d],[0],[1]])
    H_0_1=numpy.array([[cos(theta_1),-sin(theta_1),0,0],[sin(theta_1),cos(theta_1),0,0],[0,0,1,0],[0,0,0,1]])
    Ry=numpy.array([[0, 0, 1],[0, 1, 0],[-1, 0, 0]])
    Rz=numpy.array([[numpy.cos(theta_2), -numpy.sin(theta_2), 0],[numpy.sin(theta_2), numpy.cos(theta_2), 0],[0, 0, 1]])
    R_1_2=Ry.dot(Rz)
    d_1_2=numpy.array([[0],[0],[l1]])
    H_1_2=numpy.concatenate((numpy.concatenate((R_1_2,d_1_2),axis=1),numpy.array([[0,0,0,1]])),axis=0)
    H_2_3=numpy.array([[1,0,0,0],[0,1,0,0],[0,0,1,l2],[0,0,0,1]])
    p_0=H_0_1.dot(H_1_2.dot(H_2_3.dot(p_3)))
    P0=[[p_0[0][0].real],[p_0[1][0].real],[p_0[2][0].real]]
    return(P0)

#Enter the arguements below (Enter the angles theta_1 and theta_2 in degrees)
l1=1
l2=2
d=0.5
theta_1=60
theta_2=45
print(Stanford(l1,l2,d,theta_1,theta_2))