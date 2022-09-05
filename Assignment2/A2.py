
import numpy as np


def Rotx(theta1):
    return np.array([[1,0,0,0],[0,np.cos(theta1),-np.sin(theta1),0],[0,np.sin(theta1),np.cos(theta1),0],[0,0,0,1]])

def Roty(theta1):
    return np.array([[np.cos(theta1),0,np.sin(theta1),0],[0,1,0,0],[-np.sin(theta1),0,np.cos(theta1),0],[0,0,0,1]])

def Rotz(theta1):
    return np.array([[np.cos(theta1),-np.sin(theta1),0,0],[np.sin(theta1),np.cos(theta1),0,0],[0,0,1,0],[0,0,0,1]])

def TranslateX(x):
    return np.array([[1,0,0,x],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

def TranslateY(y):
    return np.array([[1,0,0,0],[0,1,0,y],[0,0,1,0],[0,0,0,1]])

def TranslateZ(z):
    return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,z],[0,0,0,1]])

def Homogenous_Matrix_without_DH(thetax,thetay,thetaz,x,y,z):  ## Only to be used for Assignment 2
    #H = np.dot(np.dot(np.dot(np.dot(np.dot(Rotz(thetaz),Roty(thetay)),Rotx(thetax)),TranslateX(x)),TranslateY(y)),TranslateZ(z))
    H1 = Rotz(thetaz)
    H2 = Roty(thetay)
    H3 = Rotx(thetax)
    H4 = TranslateX(x)
    H5 = TranslateY(y)
    H6 = TranslateZ(z)
    H = np.matmul(H1,np.matmul(H2,np.matmul(H3,np.matmul(H4,np.matmul(H5,H6)))))
    return H

def Robot3DOF_without_DH(table):   ## without DH there will be 6 parameters for each joint
    H1 = Homogenous_Matrix_without_DH(table[0][0],table[0][1],table[0][2],table[0][3],table[0][4],table[0][5])
    H2 = Homogenous_Matrix_without_DH(table[1][0],table[1][1],table[1][2],table[1][3],table[1][4],table[1][5])
    H3 = Homogenous_Matrix_without_DH(table[2][0],table[2][1],table[2][2],table[2][3],table[2][4],table[2][5])
    H4 = Homogenous_Matrix_without_DH(table[3][0],table[3][1],table[3][2],table[3][3],table[3][4],table[3][5])
    H5 = Homogenous_Matrix_without_DH(table[4][0],table[4][1],table[4][2],table[4][3],table[4][4],table[4][5])
    H6 = Homogenous_Matrix_without_DH(table[5][0],table[5][1],table[5][2],table[5][3],table[5][4],table[5][5])
    H = np.matmul(H1,np.matmul(H2,np.matmul(H3,np.matmul(H4,np.matmul(H5,H6)))))
    po = H[0:4,3]
    return po


    

######## exec code starts here ############
l1 = 1
l2 = 1
l3 = 1 # prismatic joint variable
theta1 = 0
theta2 = 0
theta3 = 0


scara_table = np.array([[0,0,theta1,0,0,0],[0,0,0,l1,0,0],[0,0,theta2,0,0,0],[0,0,0,l2,0,0],[np.pi,0,0,0,0,0],[0,0,0,0,0,l3]])  # non DH Table Stanford
stanford_table = np.array([[0,0,theta1,0,0,0],[0,0,0,0,0,l1],[-np.pi/2,0,0,0,0,0],[0,0,theta2,0,0,0],[0,np.pi/2,0,0,0,0],[0,0,0,0,0,l2+l3]])  # non DH Table Stanford


print(Robot3DOF_without_DH(scara_table))
print(Robot3DOF_without_DH(stanford_table))

# jacobian question

a1 = 1
a2 = 1
a3 = 1 # prismatic joint variable
theta1 = 0
theta2 = 0
theta3 = 0


def jacobian_3dof_scara(theta1,theta2,a1,a2):
    th12 = theta1+theta2
    J = np.array([[-a2*np.sin(th12)-a1*np.sin(theta1),-a2*np.sin(th12),0],[a2*np.cos(th12)+a1*np.cos(theta1),a2*np.cos(th12),0],[0,0,-1],[0,0,0],[0,0,0],[1,1,0]])
    return J

def jacobian_3R_planar(theta1,theta2,theta3,a1,a2,a3):
    th12 = theta1+theta2
    th13 = theta1+theta2+theta3
    J = np.array([[-a3*np.sin(th13)-a2*np.sin(th12)-a1*np.sin(theta1),-a2*np.sin(th12)-a1*np.sin(theta1),-a1*np.sin(theta1)],[a3*np.cos(th13)+a2*np.cos(th12)+a1*np.cos(theta1),a2*np.cos(th12)+a1*np.cos(theta1),a1*np.cos(theta1)],[0,0,0],[0,0,0],[0,0,0],[1,1,1]])
    return J

print(jacobian_3dof_scara(theta1,theta2,a1,a2))
print(jacobian_3R_planar(theta1,theta2,theta3,a1,a2,a3))
